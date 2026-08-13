/*******************************************************************************
* modbus_cfg.h — Modbus 参数配置模块(DataFlash 掉电保存)
*
* 功能定位: 将 Modbus 从机地址与波特率索引持久化到 CH579M 内置 DataFlash
*   (DATA_FLASH_ADDR=0x3E800, 512B/扇区), 上电自动加载, 修改后可立即保存。
*
* 寄存器映射:
*   0x0080 = 从机地址 (1~247)
*   0x0081 = 波特率索引 (0~4, 查 modbus_baud_table)
*
* 依赖: 仅 CH57x_common.h(含 CH57x_flash.h); 不依赖 uart_debug 等其它模块。
*******************************************************************************/
#ifndef __MODBUS_CFG_H
#define __MODBUS_CFG_H

#include "CH57x_common.h"

#define MODBUS_CFG_MAGIC    0xA5A5A5A5      /* DataFlash 有效性标记 */
#define MODBUS_CFG_ADDR     0x3E800         /* DataFlash 数据区起始(512B/扇区) */
#define MODBUS_BAUD_NUM     5

#define MODBUS_DEF_ADDR     1               /* 默认从机地址 */
#define MODBUS_DEF_BAUD     0               /* 默认波特率索引(0=9600) */
#define MODBUS_DEF_IDLE_MS  1000            /* 默认空闲超时提交(ms), 0=禁用自动提交 */

extern const UINT32 modbus_baud_table[ MODBUS_BAUD_NUM ];

void  modbus_cfg_init( void );              /* 上电: 读 DataFlash, 无效则默认+回写 */
UINT8 modbus_cfg_save( void );              /* 擦扇区+写, 返回 0=成功 */
UINT8 modbus_cfg_get_addr( void );          /* 1~247 */
UINT8 modbus_cfg_get_baud( void );          /* 0~4 */
UINT16 modbus_cfg_get_idle( void );         /* 空闲超时 ms, 0=禁用自动提交 */
UINT8 modbus_cfg_set_addr( UINT8 a );       /* 成功 0; 非法返回 1 */
UINT8 modbus_cfg_set_baud( UINT8 b );       /* 成功 0; 非法返回 1 */
UINT8 modbus_cfg_set_idle( UINT16 ms );     /* 成功 0; 非法返回 1 */

#endif /* __MODBUS_CFG_H */
