#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "CH57x_common.h"

#define MODBUS_ADDR         1           /* 从机地址, 可调 */
#define MODBUS_BAUD         9600

void modbus_rtu_init( void );           /* UART3 + RS485 引脚初始化 */
void modbus_rtu_poll( void );           /* 主循环调用: 收帧/解析/应答 */

#endif /* __MODBUS_RTU_H */
