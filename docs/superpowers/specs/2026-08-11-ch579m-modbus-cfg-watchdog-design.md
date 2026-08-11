# CH579M Modbus 从机产品化加固：看门狗 + 参数可配置掉电保存 — 设计文档

- 日期：2026-08-11
- 工程：CH579_USB_Host_HID_pro（CH579M，ARM Cortex-M0，Keil，UART1 115200 调试）
- 状态：已与用户逐段确认，待实施

## 背景与目标

现有 Modbus RTU 从机功能（TTL 直连验证全部通过）为编译期宏配置：地址/波特率固定于
`bsp/modbus_rtu.h` 的 `MODBUS_ADDR` / `MODBUS_BAUD`，改参数需重新编译烧录，且无死机自恢复机制。

本设计实现两项产品化加固：

1. **S1+S3 看门狗**：系统死机自动复位 + 重启后打印复位原因（是否被狗复位）
2. **S2+S4 参数可配置掉电保存**：Modbus 地址/波特率可通过 **0x06 写单个寄存器**运行时修改，
   存入 CH579M 内置 DataFlash，掉电不丢失

## 已确认的设计决策

| 决策点 | 选择 |
|---|---|
| S4 配置入口 | Modbus 0x06 写寄存器（UART1 仅保留信息打印，不承担配置命令） |
| 波特率档位 | 9600 / 19200 / 38400 / 57600 / 115200（索引 0~4） |
| 波特率生效时机 | **立即生效**（写入后马上重配 UART3，主站切新波特率重连） |
| 写配置密码保护 | 无（保持简单） |
| 配置可读回 | 0x03 扩展可读配置区（0x0080/0x0081） |
| 配置存储模块组织 | 独立新模块 `bsp/modbus_cfg.c/.h` |

## 寄存器地图（最终）

| Modbus 地址 | 用途 | 读(0x03) | 写(0x06) |
|---|---|---|---|
| 0x0000~0x007F | 键盘 ASCII 数据区 40001~40128 | ✅ | ❌ 异常 0x02 |
| 0x0080 | 从机地址配置（1~247） | ✅（数量必须=1） | ✅ |
| 0x0081 | 波特率索引配置（0~4） | ✅（数量必须=1） | ✅ |
| 其他 | — | 异常 0x02 | 异常 0x02 |

## 架构

```
┌─────────────────────────────────────────────────────────┐
│  bsp/modbus_cfg.c/.h  【新增】                            │
│  - modbus_cfg_t 结构(DataFlash 持久化)                    │
│  - modbus_baud_table[5] = {9600,19200,38400,57600,115200}│
│  - 上电 load / 修改后 save(擦扇区+写)                     │
│  - get/set API + 值校验                                   │
└─────────────────────────────────────────────────────────┘
                        ▲ 依赖(调用接口)
┌─────────────────────────────────────────────────────────┐
│  bsp/modbus_rtu.c  【修改】                               │
│  - MODBUS_ADDR 宏 → g_addr(运行时)                        │
│  - MODBUS_BAUD 宏 → baud_table[g_baud](运行时)            │
│  - 新增 0x06 写单个寄存器(配置区)                          │
│  - 0x03 扩展可读配置区(0x0080/0x0081)                     │
│  - init() 先 modbus_cfg_init() 再配 UART3                 │
└─────────────────────────────────────────────────────────┘
                        ▲
┌─────────────────────────────────────────────────────────┐
│  user/main.c  【修改】                                    │
│  - S1 看门狗: WWDG_ResetCfg + SetCounter + 主循环喂狗     │
│  - S3 复位诊断: SYS_GetLastResetSta 打印                  │
└─────────────────────────────────────────────────────────┘
```

依赖单向：`main → modbus_rtu → modbus_cfg`，与现有 `ascii_frame` 关系一致，无循环依赖。

## 模块设计

### 1. `bsp/modbus_cfg.h`（新）

```c
#ifndef __MODBUS_CFG_H
#define __MODBUS_CFG_H

#include "CH57x_common.h"

#define MODBUS_CFG_MAGIC    0xA5A5A5A5      /* DataFlash 有效性标记 */
#define MODBUS_CFG_ADDR     0x3E800         /* DataFlash 数据区起始(512B/扇区) */
#define MODBUS_BAUD_NUM     5

#define MODBUS_DEF_ADDR     1               /* 默认从机地址 */
#define MODBUS_DEF_BAUD     0               /* 默认波特率索引(0=9600) */

extern const UINT32 modbus_baud_table[ MODBUS_BAUD_NUM ];

void  modbus_cfg_init( void );              /* 上电: 读 DataFlash, 无效则默认+回写 */
UINT8 modbus_cfg_save( void );              /* 擦扇区+写, 返回 0=成功 */
UINT8 modbus_cfg_get_addr( void );          /* 1~247 */
UINT8 modbus_cfg_get_baud( void );          /* 0~4 */
UINT8 modbus_cfg_set_addr( UINT8 a );       /* 成功 0; 非法返回 1 */
UINT8 modbus_cfg_set_baud( UINT8 b );       /* 成功 0; 非法返回 1 */

#endif /* __MODBUS_CFG_H */
```

### 2. `bsp/modbus_cfg.c`（新）

```c
typedef struct {
    UINT32 magic;          /* MODBUS_CFG_MAGIC */
    UINT8  addr;           /* 从机地址 1~247 */
    UINT8  baud;           /* 波特率索引 0~4 */
    UINT8  rsv[2];         /* 预留 */
    UINT16 crc;            /* 结构校验(CRC16) */
} modbus_cfg_t;            /* 共 8 字节, 4 字节对齐 */
```

要点：
- 波特率表：`const UINT32 modbus_baud_table[5] = {9600, 19200, 38400, 57600, 115200};`
- `modbus_cfg_init()`：`(modbus_cfg_t*)MODBUS_CFG_ADDR` 直接指针读；
  校验 magic == `MODBUS_CFG_MAGIC` 且 CRC16 通过 → 采用；否则填默认值（addr=1, baud=0）并 `save()` 回写
- `modbus_cfg_save()`：`FlashBlockErase(MODBUS_CFG_ADDR)` →
  `FlashWriteBuf(MODBUS_CFG_ADDR, (PUINT32)&cfg, sizeof(cfg))`，返回库返回值（0=成功）
- set 函数：先校验范围（addr 1~247、baud 0~4），合法才写入内存 `cfg` 字段并更新 CRC，
  非法返回 1；**注意 set 只改内存，持久化由调用方决定**（本设计内 0x06 写时 set 后立即 save）
- CRC16 算法：与 `modbus_rtu.c` 中一致（Poly 0xA001 位运算），可在 cfg 模块内实现一份
  （或提供共享入口，见"风险与备注"）
- 库 Flash 擦写内部自带低压检测与操作码防呆（`CH57x_flash.c`），返回值非 0 视为保存失败

### 3. `bsp/modbus_rtu.h`（修改）

```c
#define MODBUS_CFG_ADDR_REG 0x0080  /* 0x06 写: 从机地址配置寄存器 */
#define MODBUS_CFG_BAUD_REG 0x0081  /* 0x06 写: 波特率索引配置寄存器 */
```

- 删除 `MODBUS_ADDR` / `MODBUS_BAUD` 两个宏（被运行时变量取代）

### 4. `bsp/modbus_rtu.c`（修改）

- 新增静态变量：`static UINT8 g_addr; static UINT8 g_baud;`
- `modbus_rtu_init()`：
  1. `modbus_cfg_init();` → `g_addr = modbus_cfg_get_addr(); g_baud = modbus_cfg_get_baud();`
  2. GPIO/UART3 引脚初始化（不变）
  3. `UART3_BaudRateCfg( modbus_baud_table[ g_baud ] );`
- 宏引用替换：应答头 `MODBUS_ADDR`（2 处）、地址匹配 `MODBUS_ADDR`（1 处）→ `g_addr`；
  `MODBUS_BAUD`（1 处）→ 查表
- **0x03 扩展**（`modbus_cmd03_ack`）：
  - 数据区（RegAddr < 0x0080）：原逻辑不变（Cnt 1~125、越界检查、取 ascii_frame）
  - 配置区（RegAddr >= 0x0080）：仅接受 `Cnt == 1`；
    - 0x0080 → 高字节 0，低字节 `g_addr`
    - 0x0081 → 高字节 0，低字节 `g_baud`
    - 其他 → 返回 0（上层转异常 0x02）
  - 0x03 应答结构复用现有组装方式（地址 + 功能码 + 数据字节数 + 数据 + CRC）
- **新增 0x06 写单个寄存器**（`modbus_cmd06_ack`）：
  - 解析 `RegAddr`（pRec[2..3]）与 `Value`（pRec[4..5]，高字节应为 0，否则按值整体校验）
  - `RegAddr == 0x0080`：`Value < 1 || Value > 247` → 异常 0x03（非法数据值）；
    否则 `modbus_cfg_set_addr()` + `modbus_cfg_save()`（保存失败 → 异常 0x04 或直接不应答，见"风险与备注"）
  - `RegAddr == 0x0081`：`Value > 4` → 异常 0x03；
    否则 `modbus_cfg_set_baud()` + `UART3_BaudRateCfg(modbus_baud_table[Value])`（**立即生效**）+ `save()`
  - 其他地址 → 异常 0x02（非法数据地址）
  - 成功应答 = 请求原样回显（0x06 标准：地址 + 0x06 + RegAddr + Value + CRC）
- **功能码分发**（`modbus_frame_process`）：`else if( pRec[1] == 0x06 ) return modbus_cmd06_ack(...);`
- 异常码扩展：新增 0x03（非法数据值）复用 `modbus_exception()`

### 5. `user/main.c`（修改）

初始化段（`modbus_rtu_init()` 之后）：

```c
/* S3: 复位原因诊断 */
if( SYS_GetLastResetSta() & RB_RESET_FLAG )
    up_puts( "WDOG reset\r\n" );
else
    up_puts( "reset: normal\r\n" );

/* S1: 使能看门狗, 溢出即复位 */
WWDG_ResetCfg( ENABLE );
WWDG_SetCounter( 250 );        /* 初值 250 → 32MHz 下约 1s 超时 */
```

主循环喂狗（`while(1)` 开头、`usb_hid_poll()` 之前）：

```c
WWDG_SetCounter( 250 );        /* 每 2ms 心跳喂狗 */
```

- 喂狗放主循环**开头**：Modbus 应答阻塞（最长 ~262ms @9600）结束后必然喂狗，不会饿死
- 1s 超时 ≫ 2ms 心跳 + 262ms 阻塞，余量充裕

### 6. `project/CH579_USB_Host_HID.uvprojx`（修改）

- 将 `..\bsp\modbus_cfg.c` 注册到 `bsp` 组（`uart_debug.c` / `modbus_rtu.c` 之后）

## 验证计划

1. **构建**：0 Error 0 Warning
2. **回归**：0x03 读数据区（键盘字符）行为不变
3. **0x03 读配置区**：`01 03 00 80 00 01 <CRC>` → 返回默认 `addr=1`
4. **0x06 写地址**：`01 06 00 80 00 02 <CRC>` → 应答回显；重启后 0x03 读回 `addr=2`（掉电保存生效）
5. **0x06 写波特率**：`01 06 00 81 00 01 <CRC>` → 主站切 19200 后仍可通信（立即生效）
6. **非法值**：写 addr=0/250、baud=5 → 异常码 0x03；写数据区 0x0001 → 异常码 0x02；
   非 0x03/0x06 功能码 → 异常码 0x01（回归）
7. **看门狗**：正常运行心跳不复位，上电打印 `reset: normal`，无 `WDOG reset`
8. **DataFlash 持久化**：多次断电上电后配置保持

## 改动文件清单

| 文件 | 动作 |
|---|---|
| `bsp/modbus_cfg.c` | 新增 |
| `bsp/modbus_cfg.h` | 新增 |
| `bsp/modbus_rtu.h` | 修改 |
| `bsp/modbus_rtu.c` | 修改 |
| `user/main.c` | 修改 |
| `project/CH579_USB_Host_HID.uvprojx` | 修改 |

编码：新文件 UTF-8 无 BOM（与现有 user/bsp 文件一致）；不动 GBK 库文件。

## 风险与备注

- **CRC16 复用**：`modbus_rtu.c` 现有静态 `CRC16`；`modbus_cfg.c` 需要校验函数。
  选项：a) cfg 模块内复制一份（简单，重复 10 行）；b) 将 CRC16 提为公共函数。
  默认取 a（保持模块独立，YAGNI），若实施中觉得重复可改为 b。
- **保存失败处理**：Flash 擦写可能因低压失败（库返回非 0）。默认策略：
  写入值仍然生效于内存（本次会话可用），但应答按正常回显；在 UART1 打印警告。
  备选：保存失败回异常码 0x04（设备故障）。实施时取"打印警告 + 正常应答"，
  避免主站因 Flash 故障被误判。
- **写波特率立即生效的时序**：`UART3_BaudRateCfg` 重配瞬间当前会话掉线，
  主站需切新波特率重连。此行为已与用户确认。
- **DataFlash 寿命**：参数修改频率低（现场偶发），2KB 数据区 512B 扇区，
  擦写寿命足以覆盖产品生命周期；当前不做磨损均衡（YAGNI）。
- **0x06 写数据区拒绝**：数据区 40001~40128 只读（0x06 写 0x0000~0x007F 回异常 0x02），
  保持键盘输入语义不被主站覆盖。
