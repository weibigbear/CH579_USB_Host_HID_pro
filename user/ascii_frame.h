#ifndef __ASCII_FRAME_H
#define __ASCII_FRAME_H

#include "CH57x_common.h"

#define ASCII_FRAME_SIZE   128          /* 寄存器组大小: 40001~40128 */
#define ASCII_IDLE_MS      500          /* 空闲超时提交时间, 可调 */

void ascii_frame_init( void );          /* 清零寄存器组与状态 */
void ascii_frame_putch( char c );       /* 追加一个字符(满 128 忽略), 重置空闲计数 */
void ascii_frame_backspace( void );     /* 删除上一个字符(回退并清零) */
void ascii_frame_commit( void );        /* 立即提交: 余段清零 + 复位 index */
void ascii_frame_poll( void );          /* 主循环调用(约 2ms 一次): 空闲超时自动提交 */
UINT8 ascii_frame_get( UINT8 index );   /* 读寄存器低字节(index 0~127) */

#endif /* __ASCII_FRAME_H */
