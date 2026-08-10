/*******************************************************************************
* uart_debug.c — UART1 调试打印工具
* 独立模块: USB 层与应用层共同依赖, 消除底层对 main.c 的反向依赖
*******************************************************************************/
#include <stdarg.h>
#include "CH57x_common.h"
#include "uart_debug.h"

/*******************************************************************************
* Local UART1 printf (via THR polling) -- independent of library fputc linkage
*******************************************************************************/
static void putc1( char c )
{
    while( R8_UART1_TFC == UART_FIFO_SIZE )
        ;
    R8_UART1_THR = c;
}

void up_puts( const char *s )
{
    while( *s ) putc1( *s++ );
}

void up_puthex( UINT32 v )
{
    static const char hx[] = "0123456789ABCDEF";
    char tmp[9]; int i = 8; tmp[8] = 0;
    do { tmp[--i] = hx[v & 0xF]; v >>= 4; } while( i > 0 );
    up_puts( &tmp[i] );
}

void up_puthex4( UINT16 v )
{
    static const char hx[] = "0123456789ABCDEF";
    char tmp[5]; int i = 4; tmp[4] = 0;
    do { tmp[--i] = hx[v & 0xF]; v >>= 4; } while( i > 0 );
    up_puts( &tmp[i] );
}

void up_putdec( UINT32 v )
{
    char tmp[12]; int i = 11; tmp[11] = 0;
    do { tmp[--i] = (char)( '0' + ( v % 10 ) ); v /= 10; } while( v );
    up_puts( &tmp[i] );
}

/*******************************************************************************
* 简易串口 printf -> UART1 THR，不依赖标准 printf/fputc 链接
* 支持格式：%s 字符串、%d/%u 十进制、%x 十六进制、其余原样输出
*******************************************************************************/
void up_printf( const char *fmt, ... )
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
