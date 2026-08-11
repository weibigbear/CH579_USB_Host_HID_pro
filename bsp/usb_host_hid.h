#ifndef __USB_HOST_HID_H
#define __USB_HOST_HID_H

#include "CH57x_common.h"
#include "uart_debug.h"

/*******************************************************************************
* 按键事件抽象层: 事件环形队列 + 解析/消费解耦
*******************************************************************************/
#define KEV_PRESS     0
#define KEV_RELEASE   1
/* 事件队列深度: 最坏突发(6 键和弦 press+release + 修饰键变化)≈28 事件;
   32 余量覆盖 Flash 保存等 ~20ms 主循环暂停窗口, 防止快速输入丢键
   (丢键 = Modbus 寄存器数据错字, 本设备核心数据路径, 宁大勿小)。 */
#define KEY_EV_QUEUE_SIZE  32

typedef struct
{
    UINT8   type;      /* KEV_PRESS / KEV_RELEASE */
    UINT8   mods;      /* 修饰键位: bit0 LCtrl bit1 LShift bit2 LAlt bit3 LGui bit4 RCtrl bit5 RShift bit6 RAlt bit7 RGui */
    UINT8   ifidx;     /* 0=键盘 1=多媒体 */
    UINT16  usage;     /* HID usage 码 */
} key_event_t;

/*******************************************************************************
* USB Host HID 接口
*******************************************************************************/
void    usb_hid_init( void );              /* 使能 USB 引脚/物理层 + USB_HostInit, 必须配 DMA 缓冲后调用 */
void    usb_hid_poll( void );              /* 轮询: 连接检测/枚举/HID 接口配置, 每次主循环调用 */
UINT8   usb_hid_device_ready( void );      /* 设备枚举成功且为键盘/可轮询类型 */
void    usb_hid_poll_endpoints( void );    /* 轮询 IN 端点并解析报告, 事件入队 */

UINT8   usb_hid_ev_pop( key_event_t *ev ); /* 弹出一个按键事件, 返回 1=有 0=空 */
UINT32  usb_hid_ev_drop( void );           /* 事件队列溢出丢弃计数 */
void    usb_hid_ev_clear( void );          /* 清空事件队列 */

UINT8   usb_hid_dev_status( void );        /* ThisUsbDev.DeviceStatus */
UINT8   usb_hid_dev_type( void );          /* ThisUsbDev.DeviceType */
UINT8   usb_hid_ep0( void );               /* GpVar[0] 键盘 IN 端点 */
UINT8   usb_hid_ep1( void );               /* GpVar[1] 多媒体 IN 端点 */
UINT8   usb_hid_attach( void );            /* R8_USB_MIS_ST.RB_UMS_DEV_ATTACH */
UINT16  usb_hid_vid( void );               /* 设备 VID */
UINT16  usb_hid_pid( void );               /* 设备 PID */
UINT8   usb_hid_speed( void );             /* 0=低速 1=全速 */

UINT32  usb_hid_diag_poll( void );         /* 发起的 IN 次数 */
UINT32  usb_hid_diag_ok( void );           /* 成功收到数据 */
UINT32  usb_hid_diag_nak( void );          /* 设备空闲 NAK */
UINT32  usb_hid_diag_err( void );          /* 其它错误 */
void    usb_hid_diag_reset( void );        /* 清零统计 */

#endif /* __USB_HOST_HID_H */
