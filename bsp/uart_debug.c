/*******************************************************************************
* uart_debug.c — UART1 调试打印工具(中断驱动非阻塞发送)
*
* 架构: 应用层调用 up_puts/up_printf 等, 字符先写入环形发送缓冲,
*       由 UART1 TX 中断(THR_EMPTY)在后台续填硬件 FIFO, 应用不阻塞。
* 为什么非阻塞: 主循环是超级循环架构, 若打印用 THR 轮询忙等(每行日志
*       @115200 约 4ms), 快速敲键时一轮循环被日志拖到 50ms+, USB 键盘
*       轮询间隔超过键盘上报周期(8~10ms)会丢报告 → 按键/回车偶发丢失。
*       (历史缺陷, 2026-08-11 修复: 与 modbus_rtu 的 TX 中断同一模式)
*
* 发送流程(三层保障, 任何 THR_EMPTY 触发语义下都不滞留):
*   putc1(c):        FIFO 有空位 → 直接写 THR(立即上线, 不依赖中断);
*                    FIFO 满 → 入环形缓冲 + 开 THR_EMPTY 中断。
*   UART1_IRQHandler: 从缓冲续填 FIFO, 缓冲空则关中断。
*   uart_debug_poll(): 主循环每轮兜底 —— 缓冲非空且中断未开时直接搬 FIFO,
*                    不依赖中断触发(历史缺陷根因: 中断边沿语义), 见下文。
*   唯一写 THR 的是 ISR/poll/putc1 快路径 —— putc1 快路径与 poll 同在主循环
*   串行, poll 只在中断未开时搬 FIFO 与 ISR 互斥, 无并发写竞争。
*
* 缓冲溢出策略: 环形缓冲满时丢弃新字符(日志可丢, 按键数据不可丢 ——
*   按键数据走 evq + ascii_frame, 不经本模块)。溢出时丢字符可能导致
*   日志行残缺, 属可接受行为。
*
* 注意: UART1_IRQHandler 为强符号, 覆盖 startup_ARMCM0.s 中的 [WEAK] 死循环
*   默认实现; 本模块只处理 TX(THR_EMPTY), RX 仍由 main 轮询读取。
*******************************************************************************/
#include <stdarg.h>
#include "CH57x_common.h"
#include "uart_debug.h"

#define U1_TX_BUF_SIZE   512             /* 环形发送缓冲(约 5 行 KEY 日志) */

static volatile UINT16 u1_head = 0;      /* 写指针(应用写入) */
static volatile UINT16 u1_tail = 0;      /* 读指针(ISR 取走) */
static volatile UINT8  u1_buf[ U1_TX_BUF_SIZE ];

/*******************************************************************************
* 初始化: 复位环形缓冲。UART1_DefInit() 之后调用一次(清残余)。
*******************************************************************************/
void uart_debug_init( void )
{
    u1_head = u1_tail = 0;
    UART1_INTCfg( DISABLE, RB_IER_THR_EMPTY );
}

/*******************************************************************************
* putc1: 非阻塞发送单字符。
*   快路径: 发送 FIFO 有空位 → 直接写 THR(首字符立即上线, 不依赖中断 ——
*     CH579 THR_EMPTY 中断依赖"写 THR→变空"的边沿, 若只入缓冲等中断,
*     开中断时 THR 已空则无变空边沿 → 中断永不触发 → 字符永久滞留);
*   慢路径: FIFO 满 → 入环形缓冲 + ENABLE THR_EMPTY 中断, ISR 续填。
*   与 UART3(modbus_rtu) 的 send_start 同一模式, 已验证工作。
*   putc1 唯一写者 head/THR(快路径), ISR 唯一写者 tail/THR(慢路径续填),
*   快路径与 ISR 并发写 THR 安全: THR 为 8 位写寄存器, FIFO 8 深兜底,
*   极端并发最多丢 1 字符(日志可丢, 按键数据不经本模块)。
*******************************************************************************/
static void putc1( char c )
{
    UINT16 next;

    if( R8_UART1_TFC < UART_FIFO_SIZE )          /* 快路径: FIFO 有空位 */
    {
        R8_UART1_THR = c;                        /* 直接上线, 不等中断 */
        return;
    }

    next = ( UINT16 )( ( u1_head + 1 ) % U1_TX_BUF_SIZE );
    if( next == u1_tail ) return;                /* 缓冲满: 丢弃(日志可丢) */
    u1_buf[ u1_head ] = ( UINT8 )c;
    u1_head = next;

    UART1_INTCfg( ENABLE, RB_IER_THR_EMPTY );    /* FIFO 满: 入缓冲等 ISR 续填 */
}

/*******************************************************************************
* 主循环兜底(每轮调用): 保证发送在任何中断语义下都不滞留。
*   缓冲非空 → 临界区内无条件搬缓冲进发送 FIFO(不判断 IER —— IER 置位
*   不代表中断会触发, CH579 THR_EMPTY 依赖"写 THR→变空"边沿, 历史缺陷
*   2026-08-13: 用 IER 判断中断在跑导致中断实际失效时兜底失效, 日志滞留,
*   按下其他键才偶然补发)。
*   搬不完(超 FIFO 深度)再开中断交给 ISR 续填。
* 并发安全: __disable_irq 临界区与 ISR 互斥(防 tail 竞争); putc1 与
*   本函数同在主循环串行。临界区 ~µs 级, UART3/TMR1 中断仅延迟若干 µs
*   (UART3 FIFO 8 深兜底, 不丢字节)。
*******************************************************************************/
void uart_debug_poll( void )
{
    if( u1_head == u1_tail ) return;

    __disable_irq();
    while( u1_head != u1_tail && R8_UART1_TFC < UART_FIFO_SIZE )
    {
        R8_UART1_THR = u1_buf[ u1_tail ];
        u1_tail = ( UINT16 )( ( u1_tail + 1 ) % U1_TX_BUF_SIZE );
    }
    if( u1_head != u1_tail ) UART1_INTCfg( ENABLE, RB_IER_THR_EMPTY ); /* 搬不完开中断 */
    __enable_irq();
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
* 简易串口 printf -> UART1(非阻塞), 不依赖标准 printf/fputc 链接
* 支持格式: %s 字符串、%d/%u 十进制、%x 十六进制、其余原样输出
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

/*******************************************************************************
* UART1 TX 中断: 从环形缓冲续填发送 FIFO, 直到填满或缓冲空。
* 中断源: 本模块只使能 THR_EMPTY, main 未开 UART1 RX 中断(RX 为轮询),
*   故进入本 ISR 必为 THR_EMPTY —— 不读 IIR(读 IIR 有清除挂起的副作用,
*   且其编码在 CH579 无明确文档), 直接填 FIFO, 与 UART3 ISR 同一模式。
* 收尾: 缓冲真空时关中断; putc1 快路径直接写 THR 或 uart_debug_poll
*   兜底重武装, 保证后续字符不滞留。
*******************************************************************************/
void UART1_IRQHandler( void )
{
    while( u1_head != u1_tail && R8_UART1_TFC < UART_FIFO_SIZE )
    {
        R8_UART1_THR = u1_buf[ u1_tail ];
        u1_tail = ( UINT16 )( ( u1_tail + 1 ) % U1_TX_BUF_SIZE );
    }

    if( u1_head == u1_tail )                           /* 缓冲已空: 收尾 */
    {
        UART1_INTCfg( DISABLE, RB_IER_THR_EMPTY );
    }
}
