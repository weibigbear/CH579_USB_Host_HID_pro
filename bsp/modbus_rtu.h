/*******************************************************************************
* modbus_rtu.h — Modbus RTU 从机模块接口
*
* 功能定位: 在 UART3 上实现 Modbus RTU 从机, 应答 0x03 读保持寄存器
*   (键盘 ASCII 数据区 + 配置区)与 0x06 写配置寄存器(地址/波特率)。
*   地址与波特率运行时取自 modbus_cfg 模块(DataFlash 掉电保存)。
*   本模块只负责收发帧 + CRC + 协议解析。
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

#define MODBUS_CFG_ADDR_REG 0x0080      /* 0x06 写: 从机地址配置寄存器(1~247) */
#define MODBUS_CFG_BAUD_REG 0x0081      /* 0x06 写: 波特率索引配置寄存器(0~4) */

void modbus_rtu_init( void );           /* UART3 + RS485 引脚初始化 */
void modbus_rtu_poll( void );           /* 主循环调用: 收帧/解析/应答 */

#endif /* __MODBUS_RTU_H */
