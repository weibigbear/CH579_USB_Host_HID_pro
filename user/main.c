
#include <stdarg.h>
#include "CH57x_common.h"
#include "CH579UFI.H"

__align(4) UINT8  RxBuffer[ MAX_PACKET_SIZE ];  // IN, must even address
__align(4) UINT8  TxBuffer[ MAX_PACKET_SIZE ];  // OUT, must even address

/* 轮询统计, 每个心跳窗口清零 */
static UINT32  diag_poll = 0;   /* 发起的 IN 次数 */
static UINT32  diag_ok   = 0;   /* 成功收到数据 */
static UINT32  diag_nak  = 0;   /* 设备空闲 NAK */
static UINT32  diag_err  = 0;   /* 其它错误 */

/*******************************************************************************
* Local UART1 printf (via THR polling) -- independent of library fputc linkage
*******************************************************************************/
static void putc1( char c )
{
    while( R8_UART1_TFC == UART_FIFO_SIZE )
        ;
    R8_UART1_THR = c;
}

static void up_puts( const char *s )
{
    while( *s ) putc1( *s++ );
}

static void up_puthex( UINT32 v )
{
    static const char hx[] = "0123456789ABCDEF";
    char tmp[9]; int i = 8; tmp[8] = 0;
    do { tmp[--i] = hx[v & 0xF]; v >>= 4; } while( i > 0 );
    up_puts( &tmp[i] );
}

static void up_putdec( UINT32 v )
{
    char tmp[12]; int i = 11; tmp[11] = 0;
    do { tmp[--i] = (char)( '0' + ( v % 10 ) ); v /= 10; } while( v );
    up_puts( &tmp[i] );
}

/*******************************************************************************
* 简易串口 printf -> UART1 THR，不依赖标准 printf/fputc 链接
* 支持格式：%s 字符串、%d/%u 十进制、%x 十六进制、其余原样输出
*******************************************************************************/
static void up_printf( const char *fmt, ... )
{
    va_list ap; char ch;
    va_start( ap, fmt );
    while( (*fmt) )
    {
        ch = *fmt++;
        if( ch != '%' ) { putc1( ch ); continue; }
        ch = *fmt++;
        if( ch == 's' )                up_puts( va_arg( ap, const char* ) );
        else if( ch == 'd' || ch == 'u' ) up_putdec( va_arg( ap, int ) );
        else if( ch == 'x' )           up_puthex( va_arg( ap, unsigned ) );
        else putc1( ch );
    }
    va_end( ap );
}

/*******************************************************************************
* HID keyboard scancode -> printable string.
* Covers MK5 numeric keypad usage codes 0x53-0x63 + top-row 0-9 fallback.
*******************************************************************************/
static const char *scancode_to_str( UINT8 code )
{
    switch( code )
    {
        case 0x53: return "NumLock";
        case 0x54: return "/";
        case 0x55: return "*";
        case 0x56: return "-";
        case 0x57: return "+";
        case 0x58: return "Enter";
        case 0x59: return "1";
        case 0x5A: return "2";
        case 0x5B: return "3";
        case 0x5C: return "4";
        case 0x5D: return "5";
        case 0x5E: return "6";
        case 0x5F: return "7";
        case 0x60: return "8";
        case 0x61: return "9";
        case 0x62: return "0";
        case 0x63: return ".";

        /* top-row fallback */
        case 0x1E: return "1";
        case 0x1F: return "2";
        case 0x20: return "3";
        case 0x21: return "4";
        case 0x22: return "5";
        case 0x23: return "6";
        case 0x24: return "7";
        case 0x25: return "8";
        case 0x26: return "9";
        case 0x27: return "0";

        case 0x28: return "Enter";
        case 0x29: return "Esc";
        case 0x2A: return "Back";
        case 0x2B: return "Tab";
        case 0x2C: return "Space";
        default:   return "?";
    }
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
static UINT8 HID_GetReportDescr( UINT8 infc )
{
    UINT8  s, len;
    /* bmRequestType=0x81 bRequest=0x06 wValue=0x2200 wLength=64 */
    const UINT8 getrep[] = { 0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0x40, 0x00 };
    CopySetupReqPkg( (PCHAR)getrep );
    pSetupReq -> wIndex = infc;
    s = HostCtrlTransfer( Com_Buffer, &len );
    return( s );
}

/* 轮询两个 IN 端点; 收到报告且与上次不同才打印(去重)
   报告格式: [0]=修饰键 [1]=保留 [2..7]=按键码 */
static void PollKeyboard( void )
{
    static UINT8  last[ 8 ];
    static UINT8  last_len = 0;
    static UINT8  last_ep  = 0;
    UINT8  n, i, len, ep, s;
    UINT8  same;

    for( i = 0; i < 2; i ++ )
    {
        ep = ( i == 0 ) ? ThisUsbDev.GpVar[ 0 ] : ThisUsbDev.GpVar[ 1 ];
        if( ep == 0 ) continue;

        diag_poll ++;
        s = USBHostTransact( ( USB_PID_IN << 4 ) | ep, RB_UH_R_AUTO_TOG, 50 );
        if( s != ERR_SUCCESS )
        {
            if( s == ( USB_PID_NAK | ERR_USB_TRANSFER ) ) { diag_nak ++; continue; } /* 空闲NAK */
            diag_err ++;
            if( s == ERR_USB_DISCON ) up_puts( "\r\ndev out\r\n" );
            continue;
        }

        diag_ok ++;
        len = R8_USB_RX_LEN;
        if( len > 8 ) len = 8;

        /* 去重: 同端点且内容与上次相同则跳过 */
        same = ( ep == last_ep ) && ( len == last_len );
        if( same )
        {
            for( n = 0; n < len; n ++ )
                if( RxBuffer[ n ] != last[ n ] ) { same = 0; break; }
        }
        if( same ) continue;

        up_puts( "KEY: " );
        {
            UINT8  any = 0, k;
            for( k = 2; k < len; k ++ )
            {
                if( RxBuffer[ k ] )
                {
                    if( any ) up_puts( "+" );
                    up_puts( scancode_to_str( RxBuffer[ k ] ) );
                    any = 1;
                }
            }
            if( !any ) up_puts( "-" );   /* 全零: 松开 */
        }
        up_puts( " [" );
        for( n = 0; n < len; n ++ )
        {
            if( n ) up_puts( " " );
            up_puthex( RxBuffer[ n ] );
        }
        up_puts( "]\r\n" );

        for( n = 0; n < len; n ++ ) last[ n ] = RxBuffer[ n ];
        last_len = len;
        last_ep  = ep;
    }
}

int main()
{
    UINT8  s;

    SetSysClock( CLK_SOURCE_HSE_32MHz );                 /* 外部32M晶振 */
    PWR_UnitModCfg( ENABLE, UNIT_SYS_PLL );              /* 开PLL */
    mDelaymS( 5 );

    GPIOA_SetBits( GPIO_Pin_9 );                                 /* TXD idle high */
    GPIOA_ModeCfg( GPIO_Pin_8,  GPIO_ModeIN_PU );                /* RXD */
    GPIOA_ModeCfg( GPIO_Pin_9,  GPIO_ModeOut_PP_5mA );           /* TXD */
    UART1_DefInit();                                             /* 115200 */

    up_puts( "\r\nMK5 USB-HID Host start\r\n" );

    /* DMA pointers consumed by USB_HostInit -- must be assigned first */
    pHOST_RX_RAM_Addr = RxBuffer;
    pHOST_TX_RAM_Addr = TxBuffer;

    /* 使能 USB 模拟引脚/物理层 */
    R16_PIN_ANALOG_IE |= RB_PIN_USB_IE;

    USB_HostInit();

    while(1)
    {
        s = AnalyzeRootHub();
        if( s == ERR_USB_CONNECT )
        {
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

                /* 验证假设(临时): 恢复设备实际速度, 库修复后移除 */
                SetUsbSpeed( ThisUsbDev.DeviceSpeed );

                kbd_ifnum = ( ThisUsbDev.GpVar[ 2 ] == 0xFF ) ? 0 : ThisUsbDev.GpVar[ 2 ];

                /* 关键: 组合接收器必须两个 HID 接口都初始化(SetIdle/SetProto/读报告描述符)
                   后才启动上报引擎, 只碰键盘接口会导致永久 NAK */
                up_puts( "if0: " );
                up_printf( "%x %x %x\r\n",
                    ( UINT8 )HID_SetIdle( kbd_ifnum, 0x0A ),
                    ( UINT8 )HID_SetProtocol( kbd_ifnum, 0 ),
                    ( UINT8 )HID_GetReportDescr( kbd_ifnum ) );
                if( ThisUsbDev.GpVar[ 1 ] != 0 )
                {
                    up_puts( "if1: " );
                    up_printf( "%x %x %x\r\n",
                        ( UINT8 )HID_SetIdle( 1, 0x0A ),
                        ( UINT8 )HID_SetProtocol( 1, 0 ),
                        ( UINT8 )HID_GetReportDescr( 1 ) );
                }
            }
        }

        if( ThisUsbDev.DeviceStatus == ROOT_DEV_SUCCESS )
        {
            if( ThisUsbDev.DeviceType == DEV_TYPE_KEYBOARD ||
                ThisUsbDev.GpVar[ 0 ] != 0 ||
                ThisUsbDev.DeviceType == DEV_TYPE_UNKNOW )
            {
                PollKeyboard();
            }
        }

/* 串口命令: 'p' 打印状态(其余字符忽略) */
        if( R8_UART1_LSR & RB_LSR_DATA_RDY )
        {
            if( R8_UART1_RBR == 'p' )
            {
                up_puts( "st=" );
                up_printf( "%x", ThisUsbDev.DeviceStatus );
                up_puts( " type=" );
                up_printf( "%x", ThisUsbDev.DeviceType );
                up_puts( " ep0=" );
                up_printf( "%x", ThisUsbDev.GpVar[ 0 ] );
                up_puts( " ep1=" );
                up_printf( "%x", ThisUsbDev.GpVar[ 1 ] );
                up_puts( " attach=" );
                up_printf( "%x", ( R8_USB_MIS_ST & RB_UMS_DEV_ATTACH ) ? 1 : 0 );
                up_puts( "\r\n" );
            }
        }

/* 心跳: 每秒打印一次轮询统计 */
        {
            static UINT8 sec = 0;
            if( ++sec >= 250 )
            {
                sec = 0;
                up_puts( "H: poll=" );
                up_putdec( diag_poll );
                up_puts( " ok=" );
                up_putdec( diag_ok );
                up_puts( " nak=" );
                up_putdec( diag_nak );
                up_puts( " err=" );
                up_putdec( diag_err );
                up_puts( "\r\n" );
                diag_poll = diag_ok = diag_nak = diag_err = 0;
            }
        }

        mDelaymS( 2 );
    }
}
