/*******************************************************************************
* modbus_rtu.c — Modbus RTU 从机(地址 1, 9600 8N1, RS485)
* 仅支持功能码 0x03 读保持寄存器(40001~40128)
* 硬件: TXD3=PA5 推挽 / RXD3=PA4 上拉输入 / PA6=RE/DE 控制(1k 电阻, 5V 供电)
* 帧边界: 3.5 字符空闲(9600 ≈ 3.65ms, 主循环 2ms 心跳判 2 次无字节)
*******************************************************************************/
#include "CH57x_common.h"
#include "modbus_rtu.h"
#include "ascii_frame.h"

#define RX_BUF_SIZE   256
#define TX_BUF_SIZE   256

static UINT8  rbuf[ RX_BUF_SIZE ];      /* 接收帧缓冲 */
static UINT16 rx_cnt = 0;               /* 已收字节数 */
static UINT8  rx_idle = 0;              /* 空闲心跳计数 */
static UINT8  tbuf[ TX_BUF_SIZE ];      /* 应答缓冲 */

/*******************************************************************************
* CRC16 Modbus (Poly 0xA001, 位运算)
*******************************************************************************/
static UINT16 CRC16( const UINT8 *dataIn, UINT16 length )
{
    UINT16 crc = 0xFFFF;
    UINT16 i;
    UINT8  j;
    for( i = 0; i < length; i ++ )
    {
        crc ^= dataIn[ i ];
        for( j = 0; j < 8; j ++ )
            crc = ( crc & 1 ) != 0 ? ( ( crc >> 1 ) ^ 0xA001 ) : ( crc >> 1 );
    }
    return crc;
}

/*******************************************************************************
* 0x03 读保持寄存器应答. 返回应答长度; 失败返回 0
* 寄存器映射: Modbus 地址 0~127 ↔ 40001~40128 ↔ ascii_frame_get(index)
*******************************************************************************/
static UINT16 modbus_cmd03_ack( const UINT8 *pRec, UINT8 *pAck )
{
    UINT16 i, RegAddr, Cnt;
    UINT16 AckLen;
    UINT16 CrcTmp;

    RegAddr = ( UINT16 )( ( pRec[ 2 ] << 8 ) | pRec[ 3 ] );
    Cnt     = ( UINT16 )( ( pRec[ 4 ] << 8 ) | pRec[ 5 ] );

    if( Cnt == 0 || Cnt > 125 ) return 0;                       /* 数量非法 */
    if( ( UINT32 )RegAddr + Cnt > ASCII_FRAME_SIZE ) return 0;  /* 越界 */

    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = 0x03;
    pAck[ 2 ] = ( UINT8 )( Cnt * 2 );                           /* 数据字节数 */
    AckLen = 3;
    for( i = 0; i < Cnt; i ++ )
    {
        pAck[ AckLen ++ ] = 0x00;                               /* 高字节 */
        pAck[ AckLen ++ ] = ascii_frame_get( ( UINT8 )( RegAddr + i ) );  /* 低字节 ASCII */
    }
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}

/*******************************************************************************
* 异常应答 (功能码|0x80). 返回应答长度
*******************************************************************************/
static UINT16 modbus_exception( UINT8 func, UINT8 code, UINT8 *pAck )
{
    UINT16 CrcTmp, AckLen;
    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = ( UINT8 )( func | 0x80 );
    pAck[ 2 ] = code;
    AckLen = 3;
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}

/*******************************************************************************
* 帧处理: 地址匹配 + CRC 校验 + 功能码分发. 返回应答长度; 0=不应答
*******************************************************************************/
static UINT16 modbus_frame_process( const UINT8 *pRec, UINT16 len, UINT8 *pAck )
{
    UINT16 CrcTmp;

    if( len < 8 ) return 0;                                     /* 帧太短 */
    if( pRec[ 0 ] != MODBUS_ADDR ) return 0;                    /* 地址不匹配(含广播0不响应) */

    CrcTmp = ( UINT16 )( ( pRec[ len - 1 ] << 8 ) | pRec[ len - 2 ] );
    if( CrcTmp != CRC16( pRec, len - 2 ) ) return 0;            /* CRC 失败 */

    if( pRec[ 1 ] == 0x03 )
    {
        UINT16 n = modbus_cmd03_ack( pRec, pAck );
        if( n == 0 ) return modbus_exception( 0x03, 0x02, pAck );
        return n;
    }
    return modbus_exception( pRec[ 1 ], 0x01, pAck );           /* 功能码非法 */
}

/*******************************************************************************
* UART3 发送(RS485): DE 拉高 → 稳定 → 逐字节 → 等移位完 → DE 拉低
*******************************************************************************/
static void uart3_send( const UINT8 *buf, UINT16 len )
{
    UINT16 i;
    GPIOA_SetBits( GPIO_Pin_6 );                                /* DE/RE 使能发送 */
    mDelaymS( 2 );                                              /* 收发器稳定 */
    for( i = 0; i < len; i ++ )
    {
        while( R8_UART3_TFC == UART_FIFO_SIZE ) ;               /* FIFO 满等待 */
        R8_UART3_THR = buf[ i ];
    }
    while( !( R8_UART3_LSR & RB_LSR_TX_ALL_EMP ) ) ;            /* 最后字节移位完 */
    GPIOA_ClearBits( GPIO_Pin_6 );                              /* 回接收 */
}

/*******************************************************************************
* 接口实现
*******************************************************************************/
void modbus_rtu_init( void )
{
    GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU );                /* RXD3 上拉输入 */
    GPIOA_ModeCfg( GPIO_Pin_5, GPIO_ModeOut_PP_5mA );           /* TXD3 推挽 */
    GPIOA_ModeCfg( GPIO_Pin_6, GPIO_ModeOut_PP_5mA );           /* RE/DE 推挽 */
    GPIOA_ClearBits( GPIO_Pin_6 );                              /* 初始接收方向 */
    UART3_DefInit();
    UART3_BaudRateCfg( MODBUS_BAUD );
    rx_cnt = 0;
    rx_idle = 0;
}

void modbus_rtu_poll( void )
{
    UINT8  b;
    UINT16 n;

    if( R8_UART3_LSR & RB_LSR_DATA_RDY )
    {
        while( R8_UART3_LSR & RB_LSR_DATA_RDY )                 /* 读空 FIFO */
        {
            b = R8_UART3_RBR;
            if( rx_cnt < RX_BUF_SIZE ) rbuf[ rx_cnt ++ ] = b;
            rx_idle = 0;
        }
    }
    else if( rx_cnt > 0 )
    {
        if( ++rx_idle >= 2 )                                    /* 3.5 字符空闲 ≈ 2 次心跳 */
        {
            n = modbus_frame_process( rbuf, rx_cnt, tbuf );
            if( n > 0 ) uart3_send( tbuf, n );                  /* 仅地址+CRC 匹配才应答 */
            rx_cnt = 0;
            rx_idle = 0;
        }
    }
}
