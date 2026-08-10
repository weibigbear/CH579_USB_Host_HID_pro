/*******************************************************************************
* ascii_frame.c — ASCII 帧缓冲(Modbus 40001~40128 寄存器组映射)
* 填充即写入 reg_ascii[index]; 提交 = 余段清零 + index 复位
*******************************************************************************/
#include "CH57x_common.h"
#include "ascii_frame.h"

static UINT8  reg_ascii[ ASCII_FRAME_SIZE ];
static UINT8  frame_idx = 0;    /* 当前帧写入位置 */
static UINT16 idle_cnt  = 0;    /* 空闲心跳计数(主循环 2ms 一次) */

void ascii_frame_init( void )
{
    UINT8 i;
    for( i = 0; i < ASCII_FRAME_SIZE; i ++ ) reg_ascii[ i ] = 0;
    frame_idx = 0;
    idle_cnt  = 0;
}

void ascii_frame_putch( char c )
{
    if( frame_idx >= ASCII_FRAME_SIZE ) return;      /* 满帧忽略 */
    reg_ascii[ frame_idx ++ ] = ( UINT8 )c;
    idle_cnt = 0;
}

void ascii_frame_backspace( void )
{
    if( frame_idx > 0 )
    {
        frame_idx --;
        reg_ascii[ frame_idx ] = 0;
        idle_cnt = 0;
    }
}

void ascii_frame_commit( void )
{
    UINT8 i;
    for( i = frame_idx; i < ASCII_FRAME_SIZE; i ++ ) reg_ascii[ i ] = 0;
    frame_idx = 0;
    idle_cnt  = 0;
}

void ascii_frame_poll( void )
{
    if( frame_idx == 0 ) { idle_cnt = 0; return; }   /* 空帧不提交 */
    if( ++idle_cnt >= ( ASCII_IDLE_MS / 2 ) )        /* 500ms / 2ms 心跳 */
        ascii_frame_commit();
}

UINT8 ascii_frame_get( UINT8 index )
{
    if( index >= ASCII_FRAME_SIZE ) return 0;
    return reg_ascii[ index ];
}
