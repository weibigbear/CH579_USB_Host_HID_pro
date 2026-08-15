
#include "CH57x_common.h"
#include "CH579UFI.H"
#include "uart_debug.h"
#include "usb_host_hid.h"
#include "keymap.h"
#include "ascii_frame.h"
#include "modbus_rtu.h"
#include "modbus_cfg.h"

/*******************************************************************************
* 按键日志批输出: 将累积的一批 KEY 事件合并为一条汇总行输出(而非逐事件打印)。
* 为什么合并: 快速打字时一轮可弹 12 个事件, 逐行打印每行 ~29B @115200
*   ≈2.5ms, 一次突发 ~350B 灌入 UART1 环形缓冲(512B) → 缓冲溢出日志丢行;
*   合并后一轮最多 ~35B, 任何打字速度不溢出(USB 轮询/按键功能不受影响)。
* 单事件时仍输出原详细格式(含 mods), 保留调试可读性。
* 调用点: process_key_events 内遇到 MEDIA 事件时与循环结束时。
*******************************************************************************/
static void flush_key_log( UINT8 *dn, UINT8 *up, char *chr, UINT8 *n,
                           UINT8 f_type, UINT8 f_mods, UINT16 f_usage )
{
    if( *dn == 0 && *up == 0 ) return;

    if( *dn + *up == 1 )                       /* 单事件: 原详细格式 */
    {
        dbg_puts( "KEY:  [0] " );
        dbg_puts( f_type == KEV_PRESS ? "DN " : "UP " );
        dbg_puts( "\"" );
        dbg_puts( key_display( f_usage, ( ( f_mods >> 1 ) | ( f_mods >> 5 ) ) & 1 ) );
        dbg_puts( "\" (mods=" );
        dbg_puthex( f_mods );
        dbg_puts( ")\r\n" );
    }
    else                                       /* 多事件: 合并汇总行 */
    {
        dbg_puts( "KEY:  [0] " );
        dbg_putdec( *dn );
        dbg_puts( "dn " );
        dbg_putdec( *up );
        dbg_puts( "up" );
        if( *n ) { dbg_puts( " \"" ); dbg_puts( chr ); dbg_puts( "\"" ); }
        dbg_puts( "\r\n" );
    }
    *dn = *up = *n = 0;
}

/*******************************************************************************
* 消费: 弹空队列 → 按键逻辑(功能, 逐事件) + 日志合并输出
*******************************************************************************/
static void process_key_events( void )
{
    key_event_t ev;
    UINT8  kdn = 0, kup = 0;            /* 本批 KEY 按下/释放计数 */
    UINT8  kchr_used = 0;               /* 已收集字符数 */
    char   kchr[ 9 ] = { 0 };           /* 收集的字符(≤8) */
    UINT8  f_type = 0, f_mods = 0;      /* 首个 KEY 事件(单事件时保留详情) */
    UINT16 f_usage = 0;

    while( usb_hid_ev_pop( &ev ) )
    {
        if( ev.ifidx == 1 )
        {
            /* 先刷出已累积的 KEY 批, 保持 MEDIA 行顺序正确 */
            flush_key_log( &kdn, &kup, kchr, &kchr_used, f_type, f_mods, f_usage );

            const char *n = consumer_usage_name( ev.usage );
            dbg_puts( "MEDIA: [1] " );
            dbg_puts( ev.type == KEV_PRESS ? "DN " : "UP " );
            if( n ) dbg_puts( n );
            else { dbg_puts( "0x" ); dbg_puthex4( ev.usage ); }
            dbg_puts( "\r\n" );
        }
        else
        {
            if( ev.type == KEV_PRESS && ev.usage == 0x39 )
                keymap_caps_toggle();                       /* CapsLock 按下切换 */

            /* --- ASCII 帧写入 (Modbus 40001~40128) ---
             * 仅在"按下"(KEV_PRESS)时处理一次, 抬键不再写,
             * 避免一次敲击重复写入两遍。
             * 修饰键左手 control(bit1)/左手 GUI(bit5) 合并计作 Shift ——
             * 对 ASCII 字符映射而言, 只要任一 Shift 态即可(与左右无关)。 */
            if( ev.type == KEV_PRESS )
            {
                UINT8  sh = ( ( ev.mods >> 1 ) | ( ev.mods >> 5 ) ) & 1;
                if( ev.usage == 0x28 || ev.usage == 0x58 )  /* Enter/小键盘Enter: 立即提交 */
                    ascii_frame_commit();
                else if( ev.usage == 0x2A )                 /* Backspace: 删字 */
                    ascii_frame_backspace();
                else
                {
                    const char *s = key_display( ev.usage, sh );
                    if( ev.usage == 0x2C )                  /* Space: 写空格(避免 key_display 返回空) */
                        ascii_frame_putch( ' ' );
                    else if( s[ 0 ] >= 0x20 && s[ 0 ] <= 0x7E && s[ 1 ] == 0 )
                        ascii_frame_putch( s[ 0 ] );        /* 单字符可打印才写入 */
                    /* 多字符名称(F1/Esc 等)与非 ASCII 键忽略 */
                }
            }

            /* 日志批累积 */
            if( kdn == 0 && kup == 0 ) { f_type = ev.type; f_mods = ev.mods; f_usage = ev.usage; }
            if( ev.type == KEV_PRESS )
            {
                kdn ++;
                if( kchr_used < 8 )                         /* 收集按下的字符(≤8) */
                {
                    char c = 0;
                    if( ev.usage == 0x2C ) c = ' ';         /* Space 显示为空格 */
                    else
                    {
                        const char *s = key_display( ev.usage,
                            ( ( ev.mods >> 1 ) | ( ev.mods >> 5 ) ) & 1 );
                        if( s[ 0 ] >= 0x20 ) c = s[ 0 ];    /* 多字符名(F1等)取首字符 */
                    }
                    if( c ) { kchr[ kchr_used ++ ] = c; kchr[ kchr_used ] = 0; }
                }
            }
            else kup ++;
        }
    }
    /* 收尾: 刷出剩余批 */
    flush_key_log( &kdn, &kup, kchr, &kchr_used, f_type, f_mods, f_usage );
}

int main()
{
    SetSysClock( CLK_SOURCE_HSE_32MHz );                 /* 外部32M晶振 */
    PWR_UnitModCfg( ENABLE, UNIT_SYS_PLL );              /* 开PLL */
    mDelaymS( 5 );

    GPIOA_SetBits( GPIO_Pin_9 );                                 /* TXD idle high */
    GPIOA_ModeCfg( GPIO_Pin_8,  GPIO_ModeIN_PU );                /* RXD */
    GPIOA_ModeCfg( GPIO_Pin_9,  GPIO_ModeOut_PP_5mA );           /* TXD */
    UART1_DefInit();                                             /* 115200 */
    uart_debug_init();                                           /* 发送缓冲复位(非阻塞打印) */
    NVIC_EnableIRQ( UART1_IRQn );                                /* UART1 TX 中断(THR_EMPTY) */

    ascii_frame_init();     /* 清零 Modbus 寄存器组缓冲(40001~40128) */
    ascii_frame_set_idle_ms( modbus_cfg_get_idle() );   /* 空闲超时加载配置(0x0086, 0=禁用) */
    modbus_rtu_init();      /* 初始化 UART3 + PA4/PA5/PA6(RS485) */

/* S3: 复位原因诊断——记录(上次复位原因通过 Modbus 0x0082 可读) */
    modbus_diag_set_reset_cause( SYS_GetLastResetSta() );

/* S1: 使能看门狗, 溢出即复位(初值 12 → 32MHz 下约 1s 超时) */
    WWDG_SetCounter( 12 );              /* 先重载计数再使能, 防计数恰为 0 的瞬时误复位 */
    WWDG_ResetCfg( ENABLE );

    usb_hid_init();

    while(1)
    {
        WWDG_SetCounter( 12 );      /* 每 2ms 心跳喂狗, 防饿狗(含 Modbus 阻塞后) */

        usb_hid_poll();

        if( usb_hid_device_ready() )
        {
            usb_hid_poll_endpoints();
            process_key_events();
        }

        ascii_frame_poll();     /* 空闲超时自动提交帧(500ms 可调) */
        modbus_rtu_poll();      /* 轮询 UART3: 收帧/解析/应答 Modbus 主站 */
        uart_debug_poll();      /* UART1 发送兜底: 缓冲非空且中断未开则重开 */
        modbus_diag_set_key_drop( ( UINT16 )usb_hid_ev_drop() );  /* 按键丢弃计数 → 0x0085 */

        mDelaymS( 2 );
    }
}
