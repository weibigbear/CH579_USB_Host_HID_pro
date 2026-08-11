/*******************************************************************************
* modbus_rtu.c — Modbus RTU 从机协议层实现(UART3)
*
* 协议能力: 功能码 0x03 读保持寄存器(数据区 40001~40128 + 配置区 0x0080/0x0081)
*   与 0x06 写单个寄存器(仅配置区, DataFlash 掉电保存)。
*   地址/波特率为运行时配置, 来自 modbus_cfg 模块。
*
* Modbus RTU 请求帧格式:
*   [从机地址 1B][功能码 1B][数据 nB][CRC16 低字节][CRC16 高字节]
*   本从机的 0x03 请求数据 4B: 起始寄存器地址(高/低) + 寄存器数量(高/低)。
*
* 应答帧格式(0x03 正常):
*   [从机地址][0x03][数据字节数 1B = 数量*2][寄存器高字节][寄存器低字节]...[CRC]
*   寄存器高字节恒为 0, 低字节为对应位置的 ASCII 码(来自 ascii_frame_get)。
*
* 异常应答帧:
*   [从机地址][功能码|0x80][异常码 1B][CRC]
*   0x01 非法功能(非 0x03/0x06) / 0x02 非法数据地址或数量 / 0x03 非法数据值。
*
* 帧边界判定: Modbus RTU 要求帧间静默 >= 3.5 字符时间(规范 11bit/字符);
*   由 TIM1 100μs tick 中断按当前波特率精确计时(9600→4.1ms, 115200→0.4ms),
*   UART3 接收中断(RECV_RDY)逐字节收帧, 连续无字节达 3.5T 即判帧结束。
*   波特率经 0x06 切换时, 帧边界阈值同步联动。
*
* 中断依赖: TMR1(100μs tick) + UART3(接收) 两个 NVIC 中断;
*   USB host 为轮询模式, 其中断源已在 usb_hid_init 中关闭(R8_USB_INT_EN=0),
*   故开启全局中断不影响 USB 功能。
*
* 硬件依赖: UART3 (TXD3=PA5 / RXD3=PA4), PA6 作 RS485 收发方向控制。
*******************************************************************************/
#include "CH57x_common.h"
#include "modbus_rtu.h"
#include "modbus_cfg.h"
#include "ascii_frame.h"
#include "uart_debug.h"

/* 接收/发送缓冲大小: 最大合法 0x03 请求 8B, 最大正常应答 128*2+5=261B。
   Modbus 单次最多读 125 寄存器 → 应答 253B; 缓冲 256 留够余量。 */
#define RX_BUF_SIZE   256
#define TX_BUF_SIZE   256

/* TIM1 空闲计时 tick 周期: 100μs (系统时钟 32MHz → CNT_END=3200)。
   帧边界判定精度 100μs, 115200 下 3.5T≈334μs 仍可分辨。 */
#define MODBUS_IDLE_TICK_US  100u

/* 3.5 字符时间阈值表(按规范 11bit/字符, 换算成 100μs tick 数, 向上取整):
   9600→3.5*11/9600=4.01ms→41; 19200→2.01ms→21; 38400→1.00ms→11;
   57600→0.67ms→7; 115200→0.33ms→4。与 modbus_baud_table 索引一一对应。 */
static const UINT16 idle_thresh_tab[ MODBUS_BAUD_NUM ] = { 41, 21, 11, 7, 4 };

/* 双缓冲接收: TMR1 ISR 判定一帧结束时翻转 rx_buf_sel, 新帧写入另一半区,
   poll 处理翻转前的半区 —— 无拷贝且天然隔离, 处理期间不会被新帧覆盖。
   最坏情况(背靠背两帧在 poll 处理窗口内先后完成): 丢弃旧帧保留新帧, 由主站超时重发。 */
static UINT8   rbuf[ 2 ][ RX_BUF_SIZE ];
static volatile UINT8  rx_buf_sel = 0;    /* 当前接收半区(0/1), 帧完成时翻转 */
static UINT16  rx_cnt = 0;                /* 当前帧已收字节数(ISR 维护, 帧完成时清零) */
static volatile UINT8  rx_frame_done = 0; /* 帧完成标志(TIM1 ISR 置位, poll 消费) */
static volatile UINT16 rx_frame_len = 0;  /* 已完帧长度快照(防止 ISR 新数据覆盖) */
static volatile UINT16 rx_idle_cnt = 0;   /* 无新字节的空闲 tick 计数(TIM1 100μs/tick) */
static volatile UINT16 g_idle_thresh = 41;/* 当前波特率 3.5T 对应 tick 数(波特率切换时更新) */
static UINT8  tbuf[ TX_BUF_SIZE ];        /* 应答帧缓冲(含 CRC) */
static UINT8  g_addr = MODBUS_DEF_ADDR;   /* 运行时从机地址(唯一写者: 0x06 处理与 init 加载) */
static UINT8  g_baud = MODBUS_DEF_BAUD;   /* 运行时波特率索引(唯一写者: 0x06 处理与 init 加载) */

/*******************************************************************************
* CRC16 Modbus (Poly 0xA001, 逐位运算, 无查表)
* dataIn 指向待校验数据, length 为数据长度(不含 CRC 两字节)。
* 算法: 初值 0xFFFF; 每字节先与低 8 位异或; 右移 1 位时若移出位为 1
*   则再与多项式 0xA001 异或。返回结果低字节在前发送。
*******************************************************************************/
static UINT16 CRC16( const UINT8 *dataIn, UINT16 length )
{
    UINT16 crc = 0xFFFF;
    UINT16 i;
    UINT8  j;
    for( i = 0; i < length; i ++ )
    {
        crc ^= dataIn[ i ];             /* 与当前字节低 8 位异或 */
        for( j = 0; j < 8; j ++ )       /* 逐位处理: 右移 1 位, 移出 1 则异或 0xA001 */
            crc = ( crc & 1 ) != 0 ? ( ( crc >> 1 ) ^ 0xA001 ) : ( crc >> 1 );
    }
    return crc;
}

/*******************************************************************************
* 0x03 读保持寄存器应答。返回应答长度; 请求非法返回 0(由上层发异常帧)。
* 寄存器映射: Modbus 地址 offset 0~127 ↔ 40001~40128 ↔ ascii_frame_get(index)。
* pRec 指向已校验过 CRC 的请求, pAck 指向应答缓冲。
*******************************************************************************/
static UINT16 modbus_cmd03_ack( const UINT8 *pRec, UINT8 *pAck )
{
    UINT16 i, RegAddr, Cnt;             /* 寄存器起始地址、寄存器数量 */
    UINT16 AckLen;                      /* 应答长度(不含 CRC 前) */
    UINT16 CrcTmp;

    /* 请求字段: 字节2~3=起始地址(高前低后), 字节4~5=数量 */
    RegAddr = ( UINT16 )( ( pRec[ 2 ] << 8 ) | pRec[ 3 ] );
    Cnt     = ( UINT16 )( ( pRec[ 4 ] << 8 ) | pRec[ 5 ] );

    if( Cnt == 0 || Cnt > 125 ) return 0;                       /* 数量非法(限 125) */

    /* 组装应答头: 地址 + 功能码 + 数据字节数(=寄存器数*2) */
    pAck[ 0 ] = g_addr;
    pAck[ 1 ] = 0x03;
    pAck[ 2 ] = ( UINT8 )( Cnt * 2 );                           /* 数据字节数 */
    AckLen = 3;

    /* 配置区(0x0080/0x0081): 仅支持单寄存器读取 */
    if( RegAddr >= MODBUS_CFG_ADDR_REG )
    {
        UINT8 val = 0;
        if( Cnt != 1 ) return 0;                                /* 配置区只允许数量=1 */
        if( RegAddr == MODBUS_CFG_ADDR_REG ) val = g_addr;
        else if( RegAddr == MODBUS_CFG_BAUD_REG ) val = g_baud;
        else return 0;                                          /* 未知配置地址→异常 0x02 */
        pAck[ AckLen ++ ] = 0x00;
        pAck[ AckLen ++ ] = val;
        goto ack_done;
    }

    /* 数据区 0x0000~0x007F: 越界检查 */
    if( ( UINT32 )RegAddr + Cnt > ASCII_FRAME_SIZE ) return 0;  /* 越界(寄存器组只有 128) */

    /* 逐寄存器: 高字节恒 0, 低字节取对应 ascii_frame 值 */
    for( i = 0; i < Cnt; i ++ )
    {
        pAck[ AckLen ++ ] = 0x00;                               /* 高字节 */
        pAck[ AckLen ++ ] = ascii_frame_get( ( UINT8 )( RegAddr + i ) );  /* 低字节 ASCII */
    }

ack_done:
    /* 追加 CRC(低字节在前) */
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}

/*******************************************************************************
* 异常应答: [地址][func|0x80][code][CRC]. 返回应答长度。
* func 为触发异常的功能码, code 为 Modbus 异常码。
*******************************************************************************/
static UINT16 modbus_exception( UINT8 func, UINT8 code, UINT8 *pAck )
{
    UINT16 CrcTmp, AckLen;
    pAck[ 0 ] = g_addr;
    pAck[ 1 ] = ( UINT8 )( func | 0x80 );       /* 异常标志: 最高位置 1 */
    pAck[ 2 ] = code;                           /* 异常码: 0x01/0x02/0x03 */
    AckLen = 3;
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}

/*******************************************************************************
* 0x06 写单个寄存器(仅配置区 0x0080/0x0081)。
* 成功: 应答 = 请求原样回显(Modbus 0x06 标准)。
* 失败: 返回 0(非法数据值→0x03) 或 0xFFFF(非法地址→0x02)。
* 波特率写后立即重配 UART3(主站需切新波特率重连)。
*******************************************************************************/
static UINT16 modbus_cmd06_ack( const UINT8 *pRec, UINT8 *pAck )
{
    UINT16 RegAddr, Value;
    UINT8  st;

    RegAddr = ( UINT16 )( ( pRec[ 2 ] << 8 ) | pRec[ 3 ] );
    Value   = ( UINT16 )( ( pRec[ 4 ] << 8 ) | pRec[ 5 ] );

    if( RegAddr == MODBUS_CFG_ADDR_REG )
    {
        if( Value < 1 || Value > 247 ) return 0;                /* 非法数据值→0x03 */
        st = modbus_cfg_set_addr( ( UINT8 )Value );
        if( st != 0 ) return 0;
        if( modbus_cfg_save() != 0 )
            up_puts( "cfg save fail\r\n" );                     /* 保存失败仅警告 */
        g_addr = ( UINT8 )Value;
    }
    else if( RegAddr == MODBUS_CFG_BAUD_REG )
    {
        if( Value >= MODBUS_BAUD_NUM ) return 0;                /* 非法数据值→0x03 */
        st = modbus_cfg_set_baud( ( UINT8 )Value );
        if( st != 0 ) return 0;
        if( modbus_cfg_save() != 0 )
            up_puts( "cfg save fail\r\n" );                     /* 保存失败仅警告 */
        g_baud = ( UINT8 )Value;
        UART3_BaudRateCfg( modbus_baud_table[ g_baud ] );       /* 立即生效 */
        g_idle_thresh = idle_thresh_tab[ g_baud ];              /* 帧边界阈值联动 */
    }
    else
    {
        return 0xFFFF;                                          /* 非法地址→0x02 */
    }

    /* 成功: 原样回显请求帧(含 CRC) */
    {
        UINT8 i;
        for( i = 0; i < 8; i ++ ) pAck[ i ] = pRec[ i ];
    }
    return 8;
}

/*******************************************************************************
* 整帧处理入口: 地址匹配 + CRC 校验 + 功能码分发。
* 返回应答长度; 返回 0 = 不应答(地址不符 / 帧太短 / CRC 错, 主站按超时处理)。
* 校验顺序: 先地址后 CRC(地址不符的不必算 CRC, 节省时间)。
*******************************************************************************/
static UINT16 modbus_frame_process( const UINT8 *pRec, UINT16 len, UINT8 *pAck )
{
    UINT16 CrcTmp;

    if( len < 8 ) return 0;                                     /* 帧太短(最小 8B) */
    if( pRec[ 0 ] != g_addr ) return 0;                         /* 地址不匹配(含广播0: 从机不响应广播) */

    /* 取帧尾的 CRC(低字节在前: len-2 存低位, len-1 存高位)并比对 */
    CrcTmp = ( UINT16 )( ( pRec[ len - 1 ] << 8 ) | pRec[ len - 2 ] );
    if( CrcTmp != CRC16( pRec, len - 2 ) ) return 0;            /* CRC 失败, 静默丢弃 */

    if( pRec[ 1 ] == 0x03 )                                     /* 功能码 0x03 读保持寄存器 */
    {
        UINT16 n = modbus_cmd03_ack( pRec, pAck );
        if( n == 0 ) return modbus_exception( 0x03, 0x02, pAck );  /* 参数非法→异常 0x02 */
        return n;
    }
    if( pRec[ 1 ] == 0x06 )                                     /* 功能码 0x06 写单个寄存器(配置) */
    {
        UINT16 n = modbus_cmd06_ack( pRec, pAck );
        if( n == 0 )      return modbus_exception( 0x06, 0x03, pAck );  /* 非法数据值→0x03 */
        if( n == 0xFFFF ) return modbus_exception( 0x06, 0x02, pAck );  /* 非法地址→0x02 */
        return n;
    }
    return modbus_exception( pRec[ 1 ], 0x01, pAck );           /* 其他功能码→异常 0x01 */
}

/*******************************************************************************
* UART3 发送(RS485 半双工方向控制):
*   DE 置高 → 等收发器稳定(2ms) → 逐字节写 FIFO → 等最后一字节移位输出完
*   → DE 拉低回接收态。
* 注意: 必须等 TX_ALL_EMP(发送移位寄存器空)才能拉低 DE, 否则帧尾被截断。
* 阻塞发送: 最大应答 253B @9600 ≈ 262ms, 期间不轮询 USB(可接受短阻塞)。
*******************************************************************************/
static void uart3_send( const UINT8 *buf, UINT16 len )
{
    UINT16 i;
    GPIOA_SetBits( GPIO_Pin_6 );                                /* DE/RE 使能发送(接 485 的 DI) */
    mDelaymS( 2 );                                              /* 给收发器建立时间 */
    for( i = 0; i < len; i ++ )
    {
        while( R8_UART3_TFC == UART_FIFO_SIZE ) ;               /* 发送 FIFO 满则等待 */
        R8_UART3_THR = buf[ i ];                                /* 写入一字节 */
    }
    while( !( R8_UART3_LSR & RB_LSR_TX_ALL_EMP ) ) ;            /* 等最后字节移位完(TXC 空) */
    GPIOA_ResetBits( GPIO_Pin_6 );                              /* 拉低 DE 回接收方向 */
}

/*******************************************************************************
* 接口实现
*******************************************************************************/
void modbus_rtu_init( void )
{
    modbus_cfg_init();                                          /* 上电加载配置(无效则默认) */
    g_addr = modbus_cfg_get_addr();
    g_baud = modbus_cfg_get_baud();

    GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU );                /* RXD3 上拉输入 */
    GPIOA_ModeCfg( GPIO_Pin_5, GPIO_ModeOut_PP_5mA );           /* TXD3 推挽输出 */
    GPIOA_ModeCfg( GPIO_Pin_6, GPIO_ModeOut_PP_5mA );           /* RE/DE 推挽输出 */
    GPIOA_ResetBits( GPIO_Pin_6 );                              /* 初始接收方向(DE=0) */
    UART3_DefInit();                                            /* UART3 默认 8 数据位, 无校验, 1 停止位 */
    UART3_BaudRateCfg( modbus_baud_table[ g_baud ] );           /* 运行时波特率 */
    UART3_ByteTrigCfg( UART_1BYTE_TRIG );                       /* 单字节即触发接收中断(FIFO 8 深, 防积压) */

    g_idle_thresh = idle_thresh_tab[ g_baud ];                  /* 3.5T 阈值按当前波特率 */
    rx_cnt = 0; rx_frame_done = 0; rx_frame_len = 0; rx_idle_cnt = 0;

    /* TIM1: 100μs tick 空闲计时(帧边界判定)。系统时钟 32MHz → CNT_END=3200 */
    TMR1_TimerInit( 3200u );
    TMR1_ITCfg( ENABLE, RB_TMR_IE_CYC_END );
    NVIC_EnableIRQ( TMR1_IRQn );

    /* UART3 接收中断(RECV_RDY), 每字节及时搬入 rbuf, 高波特率不丢 */
    UART3_INTCfg( ENABLE, RB_IER_RECV_RDY );
    NVIC_EnableIRQ( UART3_IRQn );

    __enable_irq();                                             /* 全局中断(USB host 轮询不依赖中断) */
}

/*******************************************************************************
* 主循环调用: 检查帧完成标志, 置位则解析并应答。
* 帧完成时 TMR1 ISR 已翻转半区, 新帧写入另一半区 —— 处理当前帧无需
*   关中断, 数据与长度天然一致。应答发送为阻塞(最大 253B@9600≈264ms)。
*******************************************************************************/
void modbus_rtu_poll( void )
{
    UINT16 n;

    if( rx_frame_done )
    {
        rx_frame_done = 0;                                      /* 先清标志, 允许 ISR 收新帧 */
        n = modbus_frame_process( rbuf[ rx_buf_sel ^ 1 ], rx_frame_len, tbuf ); /* 处理翻转前的半区 */
        if( n > 0 ) uart3_send( tbuf, n );                      /* 仅地址+CRC 匹配才应答 */
    }
}

/*******************************************************************************
* UART3 接收中断: 读空 FIFO 逐字节入 rbuf, 并清零空闲计数。
*   rbuf 溢出保护: 只保留前 RX_BUF_SIZE 字节(超长垃圾帧丢弃尾部)。
*   与主循环的同步: 帧完成由 TIM1 ISR 快照 rx_cnt 并清零, poll 只读快照。
*******************************************************************************/
void UART3_IRQHandler( void )
{
    while( R8_UART3_LSR & RB_LSR_DATA_RDY )
    {
        UINT8 b = R8_UART3_RBR;
        if( rx_cnt < RX_BUF_SIZE ) rbuf[ rx_buf_sel ][ rx_cnt ++ ] = b;   /* 防溢出 */
        rx_idle_cnt = 0;                                        /* 有数据即重置空闲计时 */
    }
}

/*******************************************************************************
* TIM1 100μs tick 中断: 空闲计数; 连续无字节达 3.5T 即判定一帧结束。
*   帧结束时快照 rx_cnt 并翻转半区: 新帧字节写入另一半区,
*   与 poll 正在处理的帧天然隔离, 无需拷贝也无需关中断。
*******************************************************************************/
void TMR1_IRQHandler( void )
{
    TMR1_ClearITFlag( RB_TMR_IF_CYC_END );                      /* 清周期中断标志 */

    rx_idle_cnt ++;
    if( rx_cnt > 0 && rx_idle_cnt >= g_idle_thresh )            /* 有数据且空闲达 3.5T */
    {
        rx_frame_len = rx_cnt;                                  /* 快照本帧长度 */
        rx_cnt       = 0;                                       /* 当前帧清零 */
        rx_buf_sel  ^= 1;                                       /* 翻转接收半区 */
        rx_idle_cnt  = 0;
        rx_frame_done = 1;                                      /* 通知主循环处理 */
    }
}
