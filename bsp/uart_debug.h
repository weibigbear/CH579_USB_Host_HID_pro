#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "CH57x_common.h"

/*******************************************************************************
* UART1 调试打印 (中断驱动非阻塞发送)
*   putc1 快路径: FIFO 有空位直接写 THR(立即上线, 不依赖中断);
*   FIFO 满时入环形缓冲, 由 UART1 TX 中断续填 —— 应用层永不等待。
* 依赖: main 需在 UART1_DefInit() 后调用 uart_debug_init() 并
*       NVIC_EnableIRQ( UART1_IRQn ); 主循环每轮调用 uart_debug_poll()
*       兜底重武装中断(防御中断丢失)。
*******************************************************************************/
void uart_debug_init( void );               /* 复位发送缓冲(清残余) */
void uart_debug_poll( void );               /* 主循环兜底: 缓冲非空且中断未开则重开 */
void up_puts( const char *s );
void up_puthex( UINT32 v );
void up_putdec( UINT32 v );
void up_puthex4( UINT16 v );
void up_printf( const char *fmt, ... );   /* 支持 %s %d/%u %x, 其余原样 */

/*******************************************************************************
* 业务日志裁剪开关:
*   APP_DEBUG_LOG = 1  -> 正常打印全部业务日志(开发调试用)
*   APP_DEBUG_LOG = 0  -> 裁剪业务日志, 仅保留错误级打印(WDOG reset / 配置保存失败等)
* 量产出货时置 0, 降低 UART1 干扰并减少主循环打印耗时。
*******************************************************************************/
#ifndef APP_DEBUG_LOG
#define APP_DEBUG_LOG   1
#endif

#if APP_DEBUG_LOG
#define dbg_puts( s )       up_puts( s )
#define dbg_printf( ... )   up_printf( __VA_ARGS__ )
#define dbg_puthex( v )     up_puthex( v )
#define dbg_puthex4( v )    up_puthex4( v )
#define dbg_putdec( v )     up_putdec( v )
#else
#define dbg_puts( s )
#define dbg_printf( ... )
#define dbg_puthex( v )
#define dbg_puthex4( v )
#define dbg_putdec( v )
#endif

#endif /* __UART_DEBUG_H */
