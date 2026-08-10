#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "CH57x_common.h"

/*******************************************************************************
* UART1 调试打印 (THR 轮询, 不依赖标准 printf/fputc 链接)
*******************************************************************************/
void up_puts( const char *s );
void up_puthex( UINT32 v );
void up_putdec( UINT32 v );
void up_puthex4( UINT16 v );
void up_printf( const char *fmt, ... );   /* 支持 %s %d/%u %x, 其余原样 */

#endif /* __UART_DEBUG_H */
