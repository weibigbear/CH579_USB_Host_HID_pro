/*******************************************************************************
* modbus_rtu.c — Modbus RTU 从机协议层实现(UART3)
*
* 协议能力: 仅支持功能码 0x03 读保持寄存器(40001~40128), 与 ascii_frame 缓冲对接。
*   不支持写操作/其他功能码(按要求返回异常 0x01 非法功能)。
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
*   0x01 非法功能(非 0x03) / 0x02 非法数据地址或数量(数量=0、>125、越界)。
*
* 帧边界判定(参考 step2 注释): 9600 下 3.5 字符空闲 ≈ 3.65ms,
*   主循环每 2ms 心跳, 连续 2 次心跳(RB_LSR_DATA_RDY 无新数据)判为帧结束。
*
* 硬件依赖: UART3 (TXD3=PA5 / RXD3=PA4), PA6 作 RS485 收发方向控制。
*******************************************************************************/
#include "CH57x_common.h"
#include "modbus_rtu.h"
#include "ascii_frame.h"

/* 接收/发送缓冲大小: 最大合法 0x03 请求 8B, 最大正常应答 128*2+5=261B。
   Modbus 单次最多读 125 寄存器 → 应答 253B; 缓冲 256 留够余量。 */
#define RX_BUF_SIZE   256
#define TX_BUF_SIZE   256

static UINT8  rbuf[ RX_BUF_SIZE ];      /* 接收帧缓冲(暂存一整帧) */
static UINT16 rx_cnt = 0;               /* 已收字节数(当前帧长度) */
static UINT8  rx_idle = 0;              /* 空闲心跳计数(无新字节的次数) */
static UINT8  tbuf[ TX_BUF_SIZE ];      /* 应答帧缓冲(含 CRC) */

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
    if( ( UINT32 )RegAddr + Cnt > ASCII_FRAME_SIZE ) return 0;  /* 越界(寄存器组只有 128) */

    /* 组装应答头: 地址 + 功能码 + 数据字节数(=寄存器数*2) */
    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = 0x03;
    pAck[ 2 ] = ( UINT8 )( Cnt * 2 );                           /* 数据字节数 */
    AckLen = 3;
    /* 逐寄存器: 高字节恒 0, 低字节取对应 ascii_frame 值 */
    for( i = 0; i < Cnt; i ++ )
    {
        pAck[ AckLen ++ ] = 0x00;                               /* 高字节 */
        pAck[ AckLen ++ ] = ascii_frame_get( ( UINT8 )( RegAddr + i ) );  /* 低字节 ASCII */
    }
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
    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = ( UINT8 )( func | 0x80 );       /* 异常标志: 最高位置 1 */
    pAck[ 2 ] = code;                           /* 异常码: 0x01/0x02 */
    AckLen = 3;
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
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
    if( pRec[ 0 ] != MODBUS_ADDR ) return 0;                    /* 地址不匹配(含广播0: 从机不响应广播) */

    /* 取帧尾的 CRC(低字节在前: len-2 存低位, len-1 存高位)并比对 */
    CrcTmp = ( UINT16 )( ( pRec[ len - 1 ] << 8 ) | pRec[ len - 2 ] );
    if( CrcTmp != CRC16( pRec, len - 2 ) ) return 0;            /* CRC 失败, 静默丢弃 */

    if( pRec[ 1 ] == 0x03 )                                     /* 功能码 0x03 读保持寄存器 */
    {
        UINT16 n = modbus_cmd03_ack( pRec, pAck );
        if( n == 0 ) return modbus_exception( 0x03, 0x02, pAck );  /* 参数非法→异常 0x02 */
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
    GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU );                /* RXD3 上拉输入 */
    GPIOA_ModeCfg( GPIO_Pin_5, GPIO_ModeOut_PP_5mA );           /* TXD3 推挽输出 */
    GPIOA_ModeCfg( GPIO_Pin_6, GPIO_ModeOut_PP_5mA );           /* RE/DE 推挽输出 */
    GPIOA_ResetBits( GPIO_Pin_6 );                              /* 初始接收方向(DE=0) */
    UART3_DefInit();                                            /* UART3 默认 8 数据位, 无校验, 1 停止位 */
    UART3_BaudRateCfg( MODBUS_BAUD );                           /* 设波特率 9600 */
    rx_cnt  = 0;                                                /* 清接收状态 */
    rx_idle = 0;
}

/*******************************************************************************
* 主循环每 2ms 调用一次: 状态机分两段
*   A) 有数据可读: 读空 UART3 FIFO 存入 rbuf, 重置空闲计数。
*   B) 无数据但已收部分字节: 空闲计数++, 连续 2 次(≈4ms)无新字节
*      即认为收到一完整帧 → 交 modbus_frame_process 处理并应答, 清空缓冲。
*******************************************************************************/
void modbus_rtu_poll( void )
{
    UINT8  b;
    UINT16 n;

    if( R8_UART3_LSR & RB_LSR_DATA_RDY )
    {
        /* A) 读空 FIFO(一字节一读), 存帧缓冲; 防溢出: 只保留前 RX_BUF_SIZE 字节 */
        while( R8_UART3_LSR & RB_LSR_DATA_RDY )                 /* 读空 FIFO */
        {
            b = R8_UART3_RBR;
            if( rx_cnt < RX_BUF_SIZE ) rbuf[ rx_cnt ++ ] = b;
            rx_idle = 0;                                        /* 有数据即清空闲计数 */
        }
    }
    else if( rx_cnt > 0 )
    {
        /* B) 空闲计时: 帧边界判定(3.5 字符空闲 ≈ 2 次心跳 ≈ 4ms) */
        if( ++rx_idle >= 2 )                                    /* 3.5 字符空闲 ≈ 2 次心跳 */
        {
            n = modbus_frame_process( rbuf, rx_cnt, tbuf );     /* 解析并生成应答 */
            if( n > 0 ) uart3_send( tbuf, n );                  /* 仅地址+CRC 匹配才应答 */
            rx_cnt  = 0;                                        /* 清当前帧, 接收下一帧 */
            rx_idle = 0;
        }
    }
}
