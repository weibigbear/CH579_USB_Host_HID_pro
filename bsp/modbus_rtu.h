/*******************************************************************************
* modbus_rtu.h — Modbus RTU 从机模块接口
*
* 功能定位: 在 UART3 上实现 Modbus RTU 从机(地址 1, 9600 8N1),
*   应答主站的 0x03 读保持寄存器请求。寄存器内容来自 ascii_frame 模块
*   (键盘 ASCII 缓冲), 本模块只负责收发帧 + CRC + 协议解析。
*
* 硬件接线(RS485):
*   TXD3 = PA5(推挽输出)  ->  485 收发器的 DI
*   RXD3 = PA4(上拉输入)  ->  485 收发器的 RO
*   PA6  = RE/DE 方向控制(推挽, 经 1k 电阻接收发器, 5V 供电)
*     发送时 PA6 置高点亮 DE(驱动总线), 发送完毕拉低回接收态;
*     TTL 直连测试(无收发器)时 PA6 悬空不影响协议工作。
*
* 帧边界: Modbus RTU 要求帧间静默 >= 3.5 字符时间;
*   9600 下约 3.65ms, 主循环每 2ms 心跳判 2 次无新字节即认为帧结束。
*
* 上层调用约定:
*   modbus_rtu_init()  — main 开机调一次, 配置引脚与 UART3。
*   modbus_rtu_poll()  — 主循环每 2ms 调一次: 收字节/判帧/解析/应答。
*******************************************************************************/
#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "CH57x_common.h"

#define MODBUS_ADDR         1           /* 从机地址, 可调(0x00=广播不响应) */
#define MODBUS_BAUD         9600        /* 波特率, 可调(需与主站一致) */

void modbus_rtu_init( void );           /* UART3 + RS485 引脚初始化 */
void modbus_rtu_poll( void );           /* 主循环调用: 收帧/解析/应答 */

#endif /* __MODBUS_RTU_H */