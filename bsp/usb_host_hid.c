/*******************************************************************************
* usb_host_hid.c — USB Host HID 层
* 职责: USB 物理层初始化 / 设备枚举 / HID 接口配置 /
*       报告解析(键盘+多媒体) / 按键事件队列 / 端点轮询与统计
* 依赖: CH57x USB Host 库 (CH57x_usbhostBase/Class), 打印函数见 usb_host_hid.h
*******************************************************************************/
#include "CH57x_common.h"
#include "CH579UFI.H"
#include "usb_host_hid.h"
__align(4) UINT8  RxBuffer[ MAX_PACKET_SIZE ];  // IN, must even address
__align(4) UINT8  TxBuffer[ MAX_PACKET_SIZE ];  // OUT, must even address

/*******************************************************************************
* 按键事件环形队列
*******************************************************************************/
static key_event_t  evq[ KEY_EV_QUEUE_SIZE ];
static UINT8  evq_head = 0, evq_tail = 0, evq_cnt = 0;
static UINT32 ev_drop = 0;

static UINT8 kbd_ev_push( UINT8 type, UINT8 mods, UINT8 ifidx, UINT16 usage )
{
    if( evq_cnt >= KEY_EV_QUEUE_SIZE ) { ev_drop ++; return 0; }
    evq[ evq_head ].type  = type;
    evq[ evq_head ].mods  = mods;
    evq[ evq_head ].ifidx = ifidx;
    evq[ evq_head ].usage = usage;
    evq_head = ( UINT8 )( ( evq_head + 1 ) % KEY_EV_QUEUE_SIZE );
    evq_cnt ++;
    return 1;
}

UINT8 usb_hid_ev_pop( key_event_t *ev )
{
    if( evq_cnt == 0 ) return 0;
    *ev = evq[ evq_tail ];
    evq_tail = ( UINT8 )( ( evq_tail + 1 ) % KEY_EV_QUEUE_SIZE );
    evq_cnt --;
    return 1;
}

UINT32 usb_hid_ev_drop( void )
{
    return ev_drop;
}

void usb_hid_ev_clear( void )
{
    evq_head = evq_tail = evq_cnt = 0;
    ev_drop = 0;
}

/*******************************************************************************
* 轮询统计, 每个心跳窗口清零
*******************************************************************************/
static UINT32  diag_poll = 0;   /* 发起的 IN 次数 */
static UINT32  diag_ok   = 0;   /* 成功收到数据 */
static UINT32  diag_nak  = 0;   /* 设备空闲 NAK */
static UINT32  diag_err  = 0;   /* 其它错误 */

UINT32 usb_hid_diag_poll( void ) { return diag_poll; }
UINT32 usb_hid_diag_ok( void )   { return diag_ok; }
UINT32 usb_hid_diag_nak( void )  { return diag_nak; }
UINT32 usb_hid_diag_err( void )  { return diag_err; }

void usb_hid_diag_reset( void )
{
    diag_poll = diag_ok = diag_nak = diag_err = 0;
}

/*****************************************************************************
* 在配置描述符中收集 HID 中断 IN 端点：
*   GpVar[0]  = 键盘接口(bInterfaceSubClass==1,Boot)的端点
*   GpVar[1]  = 其它接口的端点
*   GpVar[2]  = 键盘接口的 bInterfaceNumber, 0xFF 表示未找到
*****************************************************************************/
static void FindHID_IN_Endeps( void )
{
    UINT16  total = ( (PUSB_CFG_DESCR)Com_Buffer )->wTotalLength;
    UINT16  i = ( (PUSB_CFG_DESCR)Com_Buffer )->bLength;   /* skip config header */
    UINT8   bLen, bType;
    UINT8   kbd_if = 0;                                    /* 当前是否处于 HID 键盘接口内 */

    ThisUsbDev.GpVar[ 0 ] = 0;
    ThisUsbDev.GpVar[ 1 ] = 0;
    ThisUsbDev.GpVar[ 2 ] = 0xFF;

    while( i < total )
    {
        bLen  = Com_Buffer[ i ];
        bType = Com_Buffer[ i + 1 ];
        if( bType == USB_DESCR_TYP_INTERF )              /* interface descriptor 0x04 */
        {
            UINT8 icls  = Com_Buffer[ i + 5 ];              /* bInterfaceClass */
            UINT8 isub  = Com_Buffer[ i + 6 ];              /* bInterfaceSubClass */
            UINT8 iprot = Com_Buffer[ i + 7 ];              /* bInterfaceProtocol */
            /* 标准键盘: HID + Boot子类(1) + protocol 0/1
               排除 Consumer/非Boot接口, 防止把组合设备的第一个中断IN误当成键盘端点 */
            kbd_if = ( icls == USB_DEV_CLASS_HID ) && ( isub == 0x01 ) && ( iprot <= 1 ) ? 1 : 0;
            if( kbd_if && ThisUsbDev.GpVar[ 2 ] == 0xFF )
                ThisUsbDev.GpVar[ 2 ] = Com_Buffer[ i + 2 ];   /* bInterfaceNumber */
        }
        else if( bType == USB_DESCR_TYP_ENDP )
        {
            if( ( Com_Buffer[ i + 2 ] & USB_ENDP_DIR_MASK )              &&  /* IN  */
                ( ( Com_Buffer[ i + 3 ] & USB_ENDP_TYPE_MASK ) == USB_ENDP_TYPE_INTER ) )
            {
                if( kbd_if && ThisUsbDev.GpVar[ 0 ] == 0 )
                    ThisUsbDev.GpVar[ 0 ] = Com_Buffer[ i + 2 ];  /* keyboard ep */
                else if( ThisUsbDev.GpVar[ 1 ] == 0 )
                    ThisUsbDev.GpVar[ 1 ] = Com_Buffer[ i + 2 ];  /* other ep */
            }
        }
        if( bLen == 0 ) break;
        i += bLen;
    }
}

/* 发送 HID SET_IDLE 到键盘接口, 通知设备上报节奏(×4ms, 0=仅变化时上报) */
static UINT8 HID_SetIdle( UINT8 infc, UINT8 period )
{
    UINT8 s;
    UINT8 setidle[ 8 ];
    /* bmRequestType=0x21(接口类,OUT) bRequest=HID_SET_IDLE(0x0A) wValue=period<<8 */
    setidle[ 0 ] = 0x21; setidle[ 1 ] = HID_SET_IDLE;
    setidle[ 2 ] = period; setidle[ 3 ] = 0x00;
    setidle[ 4 ] = 0x00; setidle[ 5 ] = 0x00;
    setidle[ 6 ] = 0x00; setidle[ 7 ] = 0x00;
    CopySetupReqPkg( (PCHAR)setidle );
    pSetupReq -> wIndex = infc;
    s = HostCtrlTransfer( NULL, NULL );
    return( s );
}

/* 发送 HID SET_PROTOCOL 到键盘接口: proto=1(boot) / 0(report) */
static UINT8 HID_SetProtocol( UINT8 infc, UINT8 proto )
{
    UINT8 s;
    UINT8 setproto[ 8 ];
    /* bmRequestType=0x21(接口类,OUT) bRequest=HID_SET_PROTOCOL(0x0B) wValue=proto */
    setproto[ 0 ] = 0x21; setproto[ 1 ] = HID_SET_PROTOCOL;
    setproto[ 2 ] = proto; setproto[ 3 ] = 0x00;
    setproto[ 4 ] = 0x00; setproto[ 5 ] = 0x00;
    setproto[ 6 ] = 0x00; setproto[ 7 ] = 0x00;
    CopySetupReqPkg( (PCHAR)setproto );
    pSetupReq -> wIndex = infc;
    s = HostCtrlTransfer( NULL, NULL );
    return( s );
}

/* 获取 HID 报告描述符,bmRequestType=0x81(IN,标准,接口) bRequest=GET_DESCRIPTOR
   wValue=REPORT(0x22)<<8, wIndex=interface, 仅用于"碰一下"接口激活其上报引擎 */
static UINT8 HID_GetReportDescr( UINT8 infc, UINT8 *retlen )
{
    UINT8  s, len;
    /* bmRequestType=0x81 bRequest=0x06 wValue=0x2200 wLength=64 */
    const UINT8 getrep[] = { 0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0x40, 0x00 };
    CopySetupReqPkg( (PCHAR)getrep );
    pSetupReq -> wIndex = infc;
    s = HostCtrlTransfer( Com_Buffer, &len );
    *retlen = len;
    return( s );
}

/*******************************************************************************
* 键盘报告解析: 与上次按下集合对比生成 按下/释放 事件
* 报告格式: [0]=修饰键 [1]=保留 [2..7]=按键码
* 修饰字节位: bit0 LCtrl bit1 LShift bit2 LAlt bit3 LGui
*             bit4 RCtrl bit5 RShift bit6 RAlt bit7 RGui
*******************************************************************************/
static const UINT8 mod_usage[ 8 ] = { 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7 };

/* 修饰键位变化 → 按下/释放事件 */
static void emit_mod_events( UINT8 old_m, UINT8 new_m )
{
    UINT8 b;
    for( b = 0; b < 8; b ++ )
    {
        UINT8 ob = ( old_m >> b ) & 1, nb = ( new_m >> b ) & 1;
        if( ob != nb )
            kbd_ev_push( nb ? KEV_PRESS : KEV_RELEASE, new_m, 0, mod_usage[ b ] );
    }
}

static void parse_kbd_report( UINT8 *buf, UINT8 len )
{
    static UINT8 last_keys[ 6 ];
    static UINT8 last_mods = 0;
    UINT8  i, j, mods, found;

    if( len < 3 ) return;
    mods = buf[ 0 ];
    emit_mod_events( last_mods, mods );

    /* 释放: 上次有而本次无 */
    for( i = 0; i < 6; i ++ )
    {
        if( last_keys[ i ] == 0 ) continue;
        found = 0;
        for( j = 2; j < len; j ++ )
            if( buf[ j ] == last_keys[ i ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_RELEASE, last_mods, 0, last_keys[ i ] );
    }
    /* 按下: 本次有而上次无 */
    for( j = 2; j < len; j ++ )
    {
        if( buf[ j ] == 0 ) continue;
        found = 0;
        for( i = 0; i < 6; i ++ )
            if( last_keys[ i ] == buf[ j ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_PRESS, mods, 0, buf[ j ] );
    }
    /* 更新快照 */
    for( i = 0; i < 6; i ++ ) last_keys[ i ] = 0;
    for( i = 0, j = 2; j < len && i < 6; j ++ ) last_keys[ i ++ ] = buf[ j ];
    last_mods = mods;
}

/*******************************************************************************
* Consumer 报告解析: 接口1 报告带 ReportID(0x01), 格式 [ID][usage16 小端]
* 与上次对比生成 按下/释放 事件; ReportID=2 的系统控制位不解析
*******************************************************************************/
static void parse_consumer_report( UINT8 *buf, UINT8 len )
{
    static UINT16 last_cu[ 4 ];
    static UINT8  last_n = 0;
    UINT16 cur[ 4 ];
    UINT8  n = 0, i, j, found;

    if( len < 3 ) return;
    if( buf[ 0 ] != 0x01 ) return;      /* 仅解析 Consumer 报告(ReportID=1) */
    for( j = 1; j + 1 < len && n < 4; j += 2 )
    {
        UINT16 u = ( UINT16 )( buf[ j ] | ( buf[ j + 1 ] << 8 ) );
        if( u ) cur[ n ++ ] = u;
    }
    /* 释放 */
    for( i = 0; i < last_n; i ++ )
    {
        found = 0;
        for( j = 0; j < n; j ++ )
            if( cur[ j ] == last_cu[ i ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_RELEASE, 0, 1, last_cu[ i ] );
    }
    /* 按下 */
    for( j = 0; j < n; j ++ )
    {
        found = 0;
        for( i = 0; i < last_n; i ++ )
            if( last_cu[ i ] == cur[ j ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_PRESS, 0, 1, cur[ j ] );
    }
    for( i = 0; i < n; i ++ ) last_cu[ i ] = cur[ i ];
    last_n = n;
}

static void dump_bytes( const UINT8 *p, UINT8 n )
{
    UINT8 i;
    for( i = 0; i < n; i ++ ) { up_puthex( p[ i ] ); up_puts( " " ); }
    up_puts( "\r\n" );
}

/* 轮询两个 IN 端点; 按来源接口分发到对应解析器 */
static void PollHIDEndpoints( void )
{
    UINT8  i, ep, s, len;

    for( i = 0; i < 2; i ++ )
    {
        ep = ( i == 0 ) ? ThisUsbDev.GpVar[ 0 ] : ThisUsbDev.GpVar[ 1 ];
        if( ep == 0 ) continue;

        diag_poll ++;
        s = USBHostTransact( ( USB_PID_IN << 4 ) | ep, RB_UH_R_AUTO_TOG, 50 );
        if( s != ERR_SUCCESS )
        {
            if( s == ( USB_PID_NAK | ERR_USB_TRANSFER ) ) { diag_nak ++; continue; }
            diag_err ++;
            if( s == ERR_USB_DISCON ) up_puts( "\r\ndev out\r\n" );
            continue;
        }

        diag_ok ++;
        len = R8_USB_RX_LEN;
        if( len > 8 ) len = 8;
        if( i == 0 ) parse_kbd_report( RxBuffer, len );
        else         parse_consumer_report( RxBuffer, len );
    }
}

/*******************************************************************************
* 接口实现
*******************************************************************************/
void usb_hid_init( void )
{
    /* DMA pointers consumed by USB_HostInit -- must be assigned first */
    pHOST_RX_RAM_Addr = RxBuffer;
    pHOST_TX_RAM_Addr = TxBuffer;

    /* 使能 USB 模拟引脚/物理层 */
    R16_PIN_ANALOG_IE |= RB_PIN_USB_IE;

    USB_HostInit();
}

void usb_hid_poll( void )
{
    UINT8  s;
    UINT8  rlen;

    s = AnalyzeRootHub();
    if( s == ERR_USB_CONNECT )
    {
        /* 枚举+HID 配置为阻塞流程, 耗时可能超过看门狗 1s 超时, 期间暂停防饿狗 */
        WWDG_ResetCfg( DISABLE );
        up_puts( "dev in, enum...\n" );
        s = InitRootDevice();
        up_printf( "InitRootDevice=%x\r\n", s );
        if( s == ERR_SUCCESS )
        {
            UINT8  kbd_ifnum;
            FindHID_IN_Endeps();
            up_puts( "VID=" );
            up_printf( "%x", ThisUsbDev.DeviceVID );
            up_puts( " PID=" );
            up_printf( "%x\r\n", ThisUsbDev.DevicePID );
            up_puts( "ep0=" );
            up_printf( "%x", ThisUsbDev.GpVar[ 0 ] );
            up_puts( " ep1=" );
            up_printf( "%x\r\n", ThisUsbDev.GpVar[ 1 ] );

            /* 诊断: 枚举后的速度状态(0=低速 1=全速) */
            up_puts( "spd=" );
            up_printf( "%x", ThisUsbDev.DeviceSpeed );
            up_puts( " UC_LS=" );
            up_printf( "%x", ( R8_USB_CTRL & RB_UC_LOW_SPEED ) ? 1 : 0 );
            up_puts( " UH_LS=" );
            up_printf( "%x\r\n", ( R8_UHOST_CTRL & RB_UH_LOW_SPEED ) ? 1 : 0 );

            kbd_ifnum = ( ThisUsbDev.GpVar[ 2 ] == 0xFF ) ? 0 : ThisUsbDev.GpVar[ 2 ];

            up_puts( "if0: " );
            up_printf( "%x %x ",
                ( UINT8 )HID_SetIdle( kbd_ifnum, 0x0A ),
                ( UINT8 )HID_SetProtocol( kbd_ifnum, 0 ) );
            s = HID_GetReportDescr( kbd_ifnum, &rlen );
            up_printf( "%x\r\n", s );
            if( s == ERR_SUCCESS ) { up_puts( "rep0: " ); dump_bytes( Com_Buffer, rlen ); }

            if( ThisUsbDev.GpVar[ 1 ] != 0 )
            {
                up_puts( "if1: " );
                up_printf( "%x %x ",
                    ( UINT8 )HID_SetIdle( 1, 0x0A ),
                    ( UINT8 )HID_SetProtocol( 1, 0 ) );
                s = HID_GetReportDescr( 1, &rlen );
                up_printf( "%x\r\n", s );
                if( s == ERR_SUCCESS ) { up_puts( "rep1: " ); dump_bytes( Com_Buffer, rlen ); }
            }
        }
        /* 枚举完成, 恢复看门狗并重置计数 */
        WWDG_ResetCfg( ENABLE );
        WWDG_SetCounter( 250 );
    }
}

UINT8 usb_hid_device_ready( void )
{
    if( ThisUsbDev.DeviceStatus != ROOT_DEV_SUCCESS ) return 0;
    if( ThisUsbDev.DeviceType == DEV_TYPE_KEYBOARD ||
        ThisUsbDev.GpVar[ 0 ] != 0 ||
        ThisUsbDev.DeviceType == DEV_TYPE_UNKNOW ) return 1;
    return 0;
}

void usb_hid_poll_endpoints( void )
{
    PollHIDEndpoints();
}

UINT8 usb_hid_dev_status( void ) { return ThisUsbDev.DeviceStatus; }
UINT8 usb_hid_dev_type( void )   { return ThisUsbDev.DeviceType; }
UINT8 usb_hid_ep0( void )        { return ThisUsbDev.GpVar[ 0 ]; }
UINT8 usb_hid_ep1( void )        { return ThisUsbDev.GpVar[ 1 ]; }
UINT8 usb_hid_attach( void )     { return ( R8_USB_MIS_ST & RB_UMS_DEV_ATTACH ) ? 1 : 0; }
UINT16 usb_hid_vid( void )       { return ThisUsbDev.DeviceVID; }
UINT16 usb_hid_pid( void )       { return ThisUsbDev.DevicePID; }
UINT8 usb_hid_speed( void )      { return ThisUsbDev.DeviceSpeed; }
