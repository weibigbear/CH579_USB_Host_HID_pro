
#include "CH57x_common.h"
#include "CH579UFI.H"
#include "uart_debug.h"
#include "usb_host_hid.h"
#include "keymap.h"

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
            up_puts( "MEDIA: [1] " );
            up_puts( ev.type == KEV_PRESS ? "DN " : "UP " );
            if( n ) up_puts( n );
            else { up_puts( "0x" ); up_puthex4( ev.usage ); }
            up_puts( "\r\n" );
        }
        else
        {
            if( ev.type == KEV_PRESS && ev.usage == 0x39 )
                keymap_caps_toggle();                       /* CapsLock 按下切换 */
            up_puts( "KEY:  [0] " );
            up_puts( ev.type == KEV_PRESS ? "DN " : "UP " );
            up_puts( "\"" );
            up_puts( key_display( ev.usage, ( ( ev.mods >> 1 ) | ( ev.mods >> 5 ) ) & 1 ) );
            up_puts( "\" (mods=" );
            up_puthex( ev.mods );
            up_puts( ")\r\n" );
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

    up_puts( "\r\nMK5 USB-HID Host start\r\n" );

    usb_hid_init();

    while(1)
    {
        usb_hid_poll();

        if( usb_hid_device_ready() )
        {
            usb_hid_poll_endpoints();
            process_key_events();
        }

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

/* 心跳: 每秒打印一次轮询统计 */
        {
            static UINT8 sec = 0;
            if( ++sec >= 250 )
            {
                sec = 0;
                up_puts( "H: poll=" );
                up_putdec( usb_hid_diag_poll() );
                up_puts( " ok=" );
                up_putdec( usb_hid_diag_ok() );
                up_puts( " nak=" );
                up_putdec( usb_hid_diag_nak() );
                up_puts( " err=" );
                up_putdec( usb_hid_diag_err() );
                up_puts( "\r\n" );
                usb_hid_diag_reset();
            }
        }

        mDelaymS( 2 );
    }
}
