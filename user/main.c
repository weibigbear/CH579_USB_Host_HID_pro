
#include "CH57x_common.h"
#include "CH579UFI.H"
#include "uart_debug.h"
#include "usb_host_hid.h"
#include "keymap.h"
#include "ascii_frame.h"
#include "modbus_rtu.h"
#include "modbus_cfg.h"

/*******************************************************************************
* 消费: 弹空队列 → UART 打印
*******************************************************************************/
static void process_key_events( void )
{
    key_event_t ev;
    while( usb_hid_ev_pop( &ev ) )
    {
        if( ev.ifidx == 1 )
        {
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

            dbg_puts( "KEY:  [0] " );
            dbg_puts( ev.type == KEV_PRESS ? "DN " : "UP " );
            dbg_puts( "\"" );
            dbg_puts( key_display( ev.usage, ( ( ev.mods >> 1 ) | ( ev.mods >> 5 ) ) & 1 ) );
            dbg_puts( "\" (mods=" );
            dbg_puthex( ev.mods );
            dbg_puts( ")\r\n" );
        }
    }
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

    ascii_frame_init();     /* 清零 Modbus 寄存器组缓冲(40001~40128) */
    modbus_rtu_init();      /* 初始化 UART3 + PA4/PA5/PA6(RS485) */

/* S3: 复位原因诊断——记录并打印(上次复位原因同时通过 Modbus 0x0082 可读) */
    modbus_diag_set_reset_cause( SYS_GetLastResetSta() );
    if( SYS_GetLastResetSta() == RST_FLAG_WTR )
        up_puts( "WDOG reset\r\n" );
    else
        up_puts( "reset: normal\r\n" );

/* S1: 使能看门狗, 溢出即复位(初值 12 → 32MHz 下约 1s 超时) */
    WWDG_SetCounter( 12 );              /* 先重载计数再使能, 防计数恰为 0 的瞬时误复位 */
    WWDG_ResetCfg( ENABLE );

    dbg_puts( "\r\nMK5 USB-HID Host start\r\n" );

/* 现场诊断: 打印当前 Modbus 地址/波特率(恒打印, 接线与配置排查第一手信息) */
    up_puts( "mb: addr=" );
    up_putdec( modbus_cfg_get_addr() );
    up_puts( " baud=" );
    up_putdec( modbus_baud_table[ modbus_cfg_get_baud() ] );
    up_puts( "\r\n" );

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
        modbus_diag_set_key_drop( ( UINT16 )usb_hid_ev_drop() );  /* 按键丢弃计数 → 0x0085 */

/* 串口命令: 'p' 打印状态 'd' 打印丢弃计数 'e' 清空队列(其余字符忽略) */
        if( R8_UART1_LSR & RB_LSR_DATA_RDY )
        {
            UINT8 c = R8_UART1_RBR;
            if( c == 'p' )
            {
                up_puts( "st=" );
                up_printf( "%x", usb_hid_dev_status() );
                up_puts( " type=" );
                up_printf( "%x", usb_hid_dev_type() );
                up_puts( " ep0=" );
                up_printf( "%x", usb_hid_ep0() );
                up_puts( " ep1=" );
                up_printf( "%x", usb_hid_ep1() );
                up_puts( " attach=" );
                up_printf( "%x", usb_hid_attach() );
                up_puts( "\r\n" );
            }
            else if( c == 'd' )
            {
                up_puts( "drop=" );
                up_putdec( usb_hid_ev_drop() );
                up_puts( "\r\n" );
            }
            else if( c == 'e' )
            {
                usb_hid_ev_clear();
                up_puts( "q clr\r\n" );
            }
        }

/* 心跳: 每秒打印一次轮询统计(业务日志, 量产出货可裁剪) */
        {
            static UINT8 sec = 0;
            if( ++sec >= 250 )
            {
                sec = 0;
                dbg_puts( "H: poll=" );
                dbg_putdec( usb_hid_diag_poll() );
                dbg_puts( " ok=" );
                dbg_putdec( usb_hid_diag_ok() );
                dbg_puts( " nak=" );
                dbg_putdec( usb_hid_diag_nak() );
                dbg_puts( " err=" );
                dbg_putdec( usb_hid_diag_err() );
                dbg_puts( "\r\n" );
                usb_hid_diag_reset();
            }
        }

        mDelaymS( 2 );
    }
}
