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
 *   TIM1 100μs tick 按当前波特率精确判定(9600→4.1ms, 115200→0.4ms),
 *   UART3 接收中断逐字节收帧。需占用 TMR1 + UART3 两个 NVIC 中断。
 *
 * 应答实时性: 应答为中断驱动非阻塞发送(TX 空中断续填 FIFO),
 *   最大 255B(0x03 读 125 寄存器)@9600 线路上约需 266ms, 但主循环不被阻塞:
 *   USB 轮询/按键事件/心跳在应答期间照常运行。主站轮询周期只需
 *   > 应答时长 + 余量(255B@9600≈266ms), 无额外限制。
 *
 * 寄存器映射:
 *   数据区  0x0000~0x007F  ↔ 40001~40128(键盘 ASCII 帧, 高字节恒 0)
 *   配置区  0x0080 = 从机地址(1~247)   0x0081 = 波特率索引(0~4)   [0x06 可写]
 *   状态区  0x0082 = 上次复位原因       0x0083 = 已处理帧数
 *           0x0084 = CRC 错误帧数(含过短帧)  0x0085 = 按键事件丢弃计数
 *           0x0083~0x0085 为 16 位计数(高字节在前)              [只读, 0x03 读]
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

#define MODBUS_STAT_RESET_REG  0x0082   /* 0x03 读: 上次复位原因(SYS_GetLastResetSta) */
#define MODBUS_STAT_FRAMES_REG 0x0083   /* 0x03 读: 已处理帧数(16 位, 高字节在前) */
#define MODBUS_STAT_CRCERR_REG 0x0084   /* 0x03 读: CRC 错误帧数(16 位, 高字节在前, 含过短帧) */
#define MODBUS_STAT_KEYDROP_REG 0x0085  /* 0x03 读: 按键事件丢弃计数(16 位, 高字节在前) */

void modbus_rtu_init( void );           /* UART3 + RS485 引脚初始化 */
void modbus_rtu_poll( void );           /* 主循环调用: 收帧/解析/应答 */
void modbus_diag_set_reset_cause( UINT8 cause );  /* main 上电时记录复位原因, 供 0x0082 读 */
void modbus_diag_set_key_drop( UINT16 cnt );      /* main 每循环推送按键丢弃计数, 供 0x0085 读 */

#endif /* __MODBUS_RTU_H */
