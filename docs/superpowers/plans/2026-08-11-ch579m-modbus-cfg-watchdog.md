# CH579M Modbus 从机产品化加固（看门狗 + 参数可配置掉电保存）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 CH579M Modbus RTU 从机添加看门狗自恢复（S1+S3）与地址/波特率运行时可配置、DataFlash 掉电保存（S2+S4，0x06 写寄存器入口）。

**Architecture:** 新增独立模块 `bsp/modbus_cfg.c/.h` 封装配置结构 + DataFlash 读写 + 波特率表；`bsp/modbus_rtu.c` 将编译期宏 `MODBUS_ADDR`/`MODBUS_BAUD` 改为运行时变量，新增 0x06 写单个寄存器功能码并扩展 0x03 可读配置区（0x0080/0x0081）；`user/main.c` 加看门狗使能与主循环喂狗。依赖单向：`main → modbus_rtu → modbus_cfg`。

**Tech Stack:** C99 (ARMCC V5.06), CH579M 外设库 (CH57x_common.h / CH57x_flash.h), Keil MDK UV4 命令行构建, Modbus RTU 协议

**规格:** `docs/superpowers/specs/2026-08-11-ch579m-modbus-cfg-watchdog-design.md`

---

### Task 1: 创建 `bsp/modbus_cfg.h` — 配置模块接口

**Files:**
- Create: `bsp/modbus_cfg.h`

- [ ] **Step 1: 创建头文件**

```c
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

extern const UINT32 modbus_baud_table[ MODBUS_BAUD_NUM ];

void  modbus_cfg_init( void );              /* 上电: 读 DataFlash, 无效则默认+回写 */
UINT8 modbus_cfg_save( void );              /* 擦扇区+写, 返回 0=成功 */
UINT8 modbus_cfg_get_addr( void );          /* 1~247 */
UINT8 modbus_cfg_get_baud( void );          /* 0~4 */
UINT8 modbus_cfg_set_addr( UINT8 a );       /* 成功 0; 非法返回 1 */
UINT8 modbus_cfg_set_baud( UINT8 b );       /* 成功 0; 非法返回 1 */

#endif /* __MODBUS_CFG_H */
```

- [ ] **Step 2: 提交**

```bash
git add bsp/modbus_cfg.h
git commit -m "feat: Modbus 配置模块接口(DataFlash 掉电保存, 地址/波特率)"
```

---

### Task 2: 创建 `bsp/modbus_cfg.c` — 配置模块实现

**Files:**
- Create: `bsp/modbus_cfg.c`

- [ ] **Step 1: 创建源文件**

```c
/*******************************************************************************
* modbus_cfg.c — Modbus 参数配置模块实现(DataFlash 掉电保存)
*
* 存储布局: 结构体 modbus_cfg_t 共 8 字节(4 字节对齐), 存于
*   DATA_FLASH_ADDR(0x3E800)。写入前先整扇区擦除(512B), 再按双字写。
*
* 校验: magic + CRC16(Poly 0xA001) 双保险, 任一不符即视为无效配置,
*   回退默认值(addr=1, baud=0)并写回 Flash。
*
* 注意: 库的 Flash 擦写函数内部自带低压检测与操作码防呆(CH57x_flash.c),
*   返回非 0 表示擦写失败(如电源电压偏低), 调用方需据此处理。
*******************************************************************************/
#include "CH57x_common.h"
#include "modbus_cfg.h"

const UINT32 modbus_baud_table[ MODBUS_BAUD_NUM ] = { 9600, 19200, 38400, 57600, 115200 };

typedef struct
{
    UINT32 magic;          /* MODBUS_CFG_MAGIC */
    UINT8  addr;           /* 从机地址 1~247 */
    UINT8  baud;           /* 波特率索引 0~4 */
    UINT8  rsv[ 2 ];       /* 预留 */
    UINT16 crc;            /* 结构校验(CRC16) */
} modbus_cfg_t;            /* 共 8 字节, 4 字节对齐 */

static modbus_cfg_t g_cfg = { MODBUS_CFG_MAGIC, MODBUS_DEF_ADDR, MODBUS_DEF_BAUD, {0,0}, 0 };

/*******************************************************************************
* CRC16 Modbus (Poly 0xA001, 逐位运算) — 与 modbus_rtu.c 内实现一致
*******************************************************************************/
static UINT16 cfg_crc16( const UINT8 *dataIn, UINT16 length )
{
    UINT16 crc = 0xFFFF;
    UINT16 i;
    UINT8  j;
    for( i = 0; i < length; i ++ )
    {
        crc ^= dataIn[ i ];
        for( j = 0; j < 8; j ++ )
            crc = ( crc & 1 ) != 0 ? ( ( crc >> 1 ) ^ 0xA001 ) : ( crc >> 1 );
    }
    return crc;
}

/*******************************************************************************
* 刷新结构 CRC 字段(前 6 字节参与计算)
*******************************************************************************/
static void cfg_update_crc( void )
{
    g_cfg.crc = cfg_crc16( ( const UINT8 * )&g_cfg, 6 );
}

/*******************************************************************************
* 上电加载: 直接从 DataFlash 地址指针读取, 校验 magic + CRC;
* 无效则用默认值并回写 Flash(首次上电完成初始化)。
*******************************************************************************/
void modbus_cfg_init( void )
{
    const modbus_cfg_t *p = ( const modbus_cfg_t * )MODBUS_CFG_ADDR;

    if( p->magic == MODBUS_CFG_MAGIC && p->crc == cfg_crc16( ( const UINT8 * )p, 6 ) )
    {
        g_cfg.magic = p->magic;
        g_cfg.addr  = p->addr;
        g_cfg.baud  = p->baud;
        g_cfg.rsv[ 0 ] = p->rsv[ 0 ];
        g_cfg.rsv[ 1 ] = p->rsv[ 1 ];
        g_cfg.crc   = p->crc;
    }
    else
    {
        cfg_update_crc();
        modbus_cfg_save();
    }
}

/*******************************************************************************
* 保存: 整扇区擦除后按双字写入结构体(8 字节 = 2 个双字)。
* 返回 0=成功; 非 0=擦写失败(库返回值)。
*******************************************************************************/
UINT8 modbus_cfg_save( void )
{
    UINT8 s;
    s = FlashBlockErase( MODBUS_CFG_ADDR );
    if( s != 0 ) return s;
    return FlashWriteBuf( MODBUS_CFG_ADDR, ( PUINT32 )&g_cfg, sizeof( modbus_cfg_t ) );
}

UINT8 modbus_cfg_get_addr( void ) { return g_cfg.addr; }
UINT8 modbus_cfg_get_baud( void ) { return g_cfg.baud; }

UINT8 modbus_cfg_set_addr( UINT8 a )
{
    if( a < 1 || a > 247 ) return 1;
    g_cfg.addr = a;
    cfg_update_crc();
    return 0;
}

UINT8 modbus_cfg_set_baud( UINT8 b )
{
    if( b >= MODBUS_BAUD_NUM ) return 1;
    g_cfg.baud = b;
    cfg_update_crc();
    return 0;
}
```

- [ ] **Step 2: 提交**

```bash
git add bsp/modbus_cfg.c
git commit -m "feat: Modbus 配置模块实现(magic+CRC 校验, 扇区擦写, 波特率表)"
```

---

### Task 3: 修改 `bsp/modbus_rtu.h` — 移除宏、定义配置寄存器地址

**Files:**
- Modify: `bsp/modbus_rtu.h`

- [ ] **Step 1: 替换第 27-28 行的两个宏**

将：

```c
#define MODBUS_ADDR         1           /* 从机地址, 可调(0x00=广播不响应) */
#define MODBUS_BAUD         9600        /* 波特率, 可调(需与主站一致) */
```

替换为：

```c
#define MODBUS_CFG_ADDR_REG 0x0080      /* 0x06 写: 从机地址配置寄存器(1~247) */
#define MODBUS_CFG_BAUD_REG 0x0081      /* 0x06 写: 波特率索引配置寄存器(0~4) */
```

- [ ] **Step 2: 更新头文件顶部注释**（第 2-6 行附近，将"仅支持功能码 0x03"改为同时支持 0x06 写配置）

将第 4-6 行：

```c
* 功能定位: 在 UART3 上实现 Modbus RTU 从机(地址 1, 9600 8N1),
*   应答主站的 0x03 读保持寄存器请求。寄存器内容来自 ascii_frame 模块
*   (键盘 ASCII 缓冲), 本模块只负责收发帧 + CRC + 协议解析。
```

替换为：

```c
* 功能定位: 在 UART3 上实现 Modbus RTU 从机, 应答 0x03 读保持寄存器
*   (键盘 ASCII 数据区 + 配置区)与 0x06 写配置寄存器(地址/波特率)。
*   地址与波特率运行时取自 modbus_cfg 模块(DataFlash 掉电保存)。
*   本模块只负责收发帧 + CRC + 协议解析。
```

- [ ] **Step 3: 提交**

```bash
git add bsp/modbus_rtu.h
git commit -m "feat: modbus_rtu.h 移除编译期宏, 定义 0x0080/0x0081 配置寄存器"
```

---

### Task 4: 修改 `bsp/modbus_rtu.c` — 运行时配置 + 0x03 扩展 + 0x06 新增

**Files:**
- Modify: `bsp/modbus_rtu.c`

- [ ] **Step 1: 头文件包含** — 在 `#include "ascii_frame.h"` 后新增一行

将：

```c
#include "CH57x_common.h"
#include "modbus_rtu.h"
#include "ascii_frame.h"
```

替换为：

```c
#include "CH57x_common.h"
#include "modbus_rtu.h"
#include "modbus_cfg.h"
#include "ascii_frame.h"
```

- [ ] **Step 2: 新增运行时变量** — 在静态变量区（`static UINT8  tbuf[...]` 行后）追加

将：

```c
static UINT8  tbuf[ TX_BUF_SIZE ];      /* 应答帧缓冲(含 CRC) */
```

替换为：

```c
static UINT8  tbuf[ TX_BUF_SIZE ];      /* 应答帧缓冲(含 CRC) */
static UINT8  g_addr = MODBUS_DEF_ADDR; /* 运行时从机地址(0x06 可改) */
static UINT8  g_baud = MODBUS_DEF_BAUD; /* 运行时波特率索引(0x06 可改) */
```

- [ ] **Step 3: 修改 `modbus_cmd03_ack` 支持配置区读取**

将整个函数（第 63-92 行）：

```c
static UINT16 modbus_cmd03_ack( const UINT8 *pRec, UINT8 *pAck )
{
    UINT16 i, RegAddr, Cnt;             /* 寄存器起始地址、寄存器数量 */
    UINT16 AckLen;                      /* 应答长度(不含 CRC 前) */
    UINT16 CrcTmp;

    /* 请求字段: 字节2~3=起始地址(高前低后), 字节4~5=数量 */
    RegAddr = ( UINT16 )( ( pRec[ 2 ] << 8 ) | pRec[ 3 ] );
    Cnt     = ( UINT16 )( ( pRec[ 4 ] << 8 ) | pRec[ 5 ] );

    if( Cnt == 0 || Cnt > 125 ) return 0;                       /* 数量非法(限 125) */
    if( ( UINT32 )RegAddr + Cnt > ASCII_FRAME_SIZE ) return 0;  /* 越界(寄存器组只有 128) */

    /* 组装应答头: 地址 + 功能码 + 数据字节数(=寄存器数*2) */
    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = 0x03;
    pAck[ 2 ] = ( UINT8 )( Cnt * 2 );                           /* 数据字节数 */
    AckLen = 3;
    /* 逐寄存器: 高字节恒 0, 低字节取对应 ascii_frame 值 */
    for( i = 0; i < Cnt; i ++ )
    {
        pAck[ AckLen ++ ] = 0x00;                               /* 高字节 */
        pAck[ AckLen ++ ] = ascii_frame_get( ( UINT8 )( RegAddr + i ) );  /* 低字节 ASCII */
    }
    /* 追加 CRC(低字节在前) */
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}
```

替换为：

```c
static UINT16 modbus_cmd03_ack( const UINT8 *pRec, UINT8 *pAck )
{
    UINT16 i, RegAddr, Cnt;             /* 寄存器起始地址、寄存器数量 */
    UINT16 AckLen;                      /* 应答长度(不含 CRC 前) */
    UINT16 CrcTmp;

    /* 请求字段: 字节2~3=起始地址(高前低后), 字节4~5=数量 */
    RegAddr = ( UINT16 )( ( pRec[ 2 ] << 8 ) | pRec[ 3 ] );
    Cnt     = ( UINT16 )( ( pRec[ 4 ] << 8 ) | pRec[ 5 ] );

    if( Cnt == 0 || Cnt > 125 ) return 0;                       /* 数量非法(限 125) */

    /* 组装应答头: 地址 + 功能码 + 数据字节数(=寄存器数*2) */
    pAck[ 0 ] = g_addr;
    pAck[ 1 ] = 0x03;
    pAck[ 2 ] = ( UINT8 )( Cnt * 2 );                           /* 数据字节数 */
    AckLen = 3;

    /* 配置区(0x0080/0x0081): 仅支持单寄存器读取 */
    if( RegAddr >= MODBUS_CFG_ADDR_REG )
    {
        UINT8 val = 0;
        if( Cnt != 1 ) return 0;                                /* 配置区只允许数量=1 */
        if( RegAddr == MODBUS_CFG_ADDR_REG ) val = g_addr;
        else if( RegAddr == MODBUS_CFG_BAUD_REG ) val = g_baud;
        else return 0;                                          /* 未知配置地址→异常 0x02 */
        pAck[ AckLen ++ ] = 0x00;
        pAck[ AckLen ++ ] = val;
        goto ack_done;
    }

    /* 数据区 0x0000~0x007F: 越界检查 */
    if( ( UINT32 )RegAddr + Cnt > ASCII_FRAME_SIZE ) return 0;  /* 越界(寄存器组只有 128) */

    /* 逐寄存器: 高字节恒 0, 低字节取对应 ascii_frame 值 */
    for( i = 0; i < Cnt; i ++ )
    {
        pAck[ AckLen ++ ] = 0x00;                               /* 高字节 */
        pAck[ AckLen ++ ] = ascii_frame_get( ( UINT8 )( RegAddr + i ) );  /* 低字节 ASCII */
    }

ack_done:
    /* 追加 CRC(低字节在前) */
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}
```

- [ ] **Step 4: 修改 `modbus_exception` 使用运行时地址**

将第 98-109 行中：

```c
    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = ( UINT8 )( func | 0x80 );       /* 异常标志: 最高位置 1 */
```

替换为：

```c
    pAck[ 0 ] = g_addr;
    pAck[ 1 ] = ( UINT8 )( func | 0x80 );       /* 异常标志: 最高位置 1 */
```

- [ ] **Step 5: 新增 0x06 写单个寄存器函数** — 在 `modbus_frame_process` 函数前插入

在 `modbus_exception` 函数结束（`return AckLen;` 与 `}` 之后）插入新函数：

```c
/*******************************************************************************
* 0x06 写单个寄存器(仅配置区 0x0080/0x0081)。
* 成功: 应答 = 请求原样回显(Modbus 0x06 标准)。
* 失败: 返回 0(上层转异常): 0x02 非法地址 / 0x03 非法数据值。
* 波特率写后立即重配 UART3(主站需切新波特率重连)。
*******************************************************************************/
static UINT16 modbus_cmd06_ack( const UINT8 *pRec, UINT8 *pAck )
{
    UINT16 RegAddr, Value;
    UINT8  st;

    RegAddr = ( UINT16 )( ( pRec[ 2 ] << 8 ) | pRec[ 3 ] );
    Value   = ( UINT16 )( ( pRec[ 4 ] << 8 ) | pRec[ 5 ] );

    if( RegAddr == MODBUS_CFG_ADDR_REG )
    {
        if( Value < 1 || Value > 247 ) return 0;                /* 非法数据值→0x03 */
        st = modbus_cfg_set_addr( ( UINT8 )Value );
        if( st != 0 ) return 0;
        if( modbus_cfg_save() != 0 )
            up_puts( "cfg save fail\r\n" );                     /* 保存失败仅警告 */
        g_addr = ( UINT8 )Value;
    }
    else if( RegAddr == MODBUS_CFG_BAUD_REG )
    {
        if( Value >= MODBUS_BAUD_NUM ) return 0;                /* 非法数据值→0x03 */
        st = modbus_cfg_set_baud( ( UINT8 )Value );
        if( st != 0 ) return 0;
        if( modbus_cfg_save() != 0 )
            up_puts( "cfg save fail\r\n" );                     /* 保存失败仅警告 */
        g_baud = ( UINT8 )Value;
        UART3_BaudRateCfg( modbus_baud_table[ g_baud ] );       /* 立即生效 */
    }
    else
    {
        return 0xFFFF;                                          /* 非法地址→0x02 */
    }

    /* 成功: 原样回显请求帧(含 CRC) */
    {
        UINT8 i;
        for( i = 0; i < 8; i ++ ) pAck[ i ] = pRec[ i ];
    }
    return 8;
}
```

- [ ] **Step 6: 修改 `modbus_frame_process` — 运行时地址 + 0x06 分发**

将第 116-134 行整个函数替换为：

```c
static UINT16 modbus_frame_process( const UINT8 *pRec, UINT16 len, UINT8 *pAck )
{
    UINT16 CrcTmp;

    if( len < 8 ) return 0;                                     /* 帧太短(最小 8B) */
    if( pRec[ 0 ] != g_addr ) return 0;                         /* 地址不匹配(含广播0: 从机不响应广播) */

    /* 取帧尾的 CRC(低字节在前: len-2 存低位, len-1 存高位)并比对 */
    CrcTmp = ( UINT16 )( ( pRec[ len - 1 ] << 8 ) | pRec[ len - 2 ] );
    if( CrcTmp != CRC16( pRec, len - 2 ) ) return 0;            /* CRC 失败, 静默丢弃 */

    if( pRec[ 1 ] == 0x03 )                                     /* 功能码 0x03 读保持寄存器 */
    {
        UINT16 n = modbus_cmd03_ack( pRec, pAck );
        if( n == 0 ) return modbus_exception( 0x03, 0x02, pAck );  /* 参数非法→异常 0x02 */
        return n;
    }
    if( pRec[ 1 ] == 0x06 )                                     /* 功能码 0x06 写单个寄存器(配置) */
    {
        UINT16 n = modbus_cmd06_ack( pRec, pAck );
        if( n == 0 ) return modbus_exception( 0x06, 0x02, pAck );  /* 暂按 0x02, 见下方修正 */
        return n;
    }
    return modbus_exception( pRec[ 1 ], 0x01, pAck );           /* 其他功能码→异常 0x01 */
}
```

- [ ] **Step 7: 修正 0x06 异常码区分**（0x02 非法地址 vs 0x03 非法数据值）

`modbus_cmd06_ack` 返回 0 有"非法地址"与"非法数据值"两种含义，需要区分。Step 5 的函数中：
- 地址 == 0x0080 且值非法：`return 0;`（保持，→0x03）
- 地址 == 0x0081 且值非法：`return 0;`（保持，→0x03）
- 其它地址：`return 0xFFFF;`（→0x02）——Step 5 代码中已按此写法

Step 6 的分发改为：

```c
    if( pRec[ 1 ] == 0x06 )                                     /* 功能码 0x06 写单个寄存器(配置) */
    {
        UINT16 n = modbus_cmd06_ack( pRec, pAck );
        if( n == 0 )      return modbus_exception( 0x06, 0x03, pAck );  /* 非法数据值→0x03 */
        if( n == 0xFFFF ) return modbus_exception( 0x06, 0x02, pAck );  /* 非法地址→0x02 */
        return n;
    }
```

同时修改 Step 5 函数中三处 `return 0;`：

- 地址 == 0x0080 且值非法：`return 0;`（保持，→0x03）
- 地址 == 0x0081 且值非法：`return 0;`（保持，→0x03）
- 其它地址：`return 0xFFFF;`（→0x02）

最终 `modbus_cmd06_ack` 中 `else { return 0; }` 一处改为 `return 0xFFFF;`。

- [ ] **Step 8: 修改 `modbus_rtu_init` — 加载配置 + 运行时波特率**

将第 160-170 行整个函数替换为：

```c
void modbus_rtu_init( void )
{
    modbus_cfg_init();                                          /* 上电加载配置(无效则默认) */
    g_addr = modbus_cfg_get_addr();
    g_baud = modbus_cfg_get_baud();

    GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU );                /* RXD3 上拉输入 */
    GPIOA_ModeCfg( GPIO_Pin_5, GPIO_ModeOut_PP_5mA );           /* TXD3 推挽输出 */
    GPIOA_ModeCfg( GPIO_Pin_6, GPIO_ModeOut_PP_5mA );           /* RE/DE 推挽输出 */
    GPIOA_ResetBits( GPIO_Pin_6 );                              /* 初始接收方向(DE=0) */
    UART3_DefInit();                                            /* UART3 默认 8 数据位, 无校验, 1 停止位 */
    UART3_BaudRateCfg( modbus_baud_table[ g_baud ] );           /* 运行时波特率 */
    rx_cnt  = 0;                                                /* 清接收状态 */
    rx_idle = 0;
}
```

- [ ] **Step 9: 更新文件头注释**（第 2-17 行，反映 0x06 支持）

将第 4-5 行：

```c
* 协议能力: 仅支持功能码 0x03 读保持寄存器(40001~40128), 与 ascii_frame 缓冲对接。
*   不支持写操作/其他功能码(按要求返回异常 0x01 非法功能)。
```

替换为：

```c
* 协议能力: 功能码 0x03 读保持寄存器(数据区 40001~40128 + 配置区 0x0080/0x0081)
*   与 0x06 写单个寄存器(仅配置区, DataFlash 掉电保存)。
*   地址/波特率为运行时配置, 来自 modbus_cfg 模块。
```

并将第 17 行：

```c
*   0x01 非法功能(非 0x03) / 0x02 非法数据地址或数量(数量=0、>125、越界)。
```

替换为：

```c
*   0x01 非法功能(非 0x03/0x06) / 0x02 非法数据地址或数量 / 0x03 非法数据值。
```

- [ ] **Step 10: 构建验证**

```bash
Remove-Item "project\Objects\CH579_USB_Host_HID_Target 1.dep" -ErrorAction SilentlyContinue
Remove-Item project\Objects\modbus_cfg*, project\Objects\modbus_rtu* -ErrorAction SilentlyContinue
& "D:\Keil_v5\UV4\UV4.exe" -b project\CH579_USB_Host_HID.uvprojx -j0 -o project\build_log.txt
Get-Content project\build_log.txt -Tail 6
```

Expected: `".\Objects\CH579_USB_Host_HID.axf" - 0 Error(s), 0 Warning(s)`

- [ ] **Step 11: 提交**

```bash
git add bsp/modbus_rtu.c
git commit -m "feat: modbus_rtu 运行时地址/波特率 + 0x03 读配置区 + 0x06 写配置"
```

---

### Task 5: 修改 `user/main.c` — 看门狗 + 复位诊断

**Files:**
- Modify: `user/main.c`

- [ ] **Step 1: 初始化段加复位诊断与看门狗** — 在 `modbus_rtu_init();` 之后

将：

```c
    ascii_frame_init();     /* 清零 Modbus 寄存器组缓冲(40001~40128) */
    modbus_rtu_init();      /* 初始化 UART3 + PA4/PA5/PA6(RS485) */

    up_puts( "\r\nMK5 USB-HID Host start\r\n" );
```

替换为：

```c
    ascii_frame_init();     /* 清零 Modbus 寄存器组缓冲(40001~40128) */
    modbus_rtu_init();      /* 初始化 UART3 + PA4/PA5/PA6(RS485) */

/* S3: 复位原因诊断——上次是否为看门狗复位 */
    if( SYS_GetLastResetSta() & RB_RESET_FLAG )
        up_puts( "WDOG reset\r\n" );
    else
        up_puts( "reset: normal\r\n" );

/* S1: 使能看门狗, 溢出即复位(初值 250 → 32MHz 下约 1s 超时) */
    WWDG_ResetCfg( ENABLE );
    WWDG_SetCounter( 250 );

    up_puts( "\r\nMK5 USB-HID Host start\r\n" );
```

- [ ] **Step 2: 主循环喂狗** — 在 `while(1)` 开头第一句

将：

```c
    while(1)
    {
        usb_hid_poll();
```

替换为：

```c
    while(1)
    {
        WWDG_SetCounter( 250 );     /* 每 2ms 心跳喂狗, 防饿狗(含 Modbus 阻塞后) */

        usb_hid_poll();
```

- [ ] **Step 3: 构建验证**

```bash
Remove-Item project\Objects\CH579_USB_Host_HID.axf -ErrorAction SilentlyContinue
& "D:\Keil_v5\UV4\UV4.exe" -b project\CH579_USB_Host_HID.uvprojx -j0 -o project\build_log.txt
Get-Content project\build_log.txt -Tail 6
```

Expected: `0 Error(s), 0 Warning(s)`

- [ ] **Step 4: 提交**

```bash
git add user/main.c
git commit -m "feat: 看门狗使能+主循环喂狗(1s 超时) + 复位原因诊断打印"
```

---

### Task 6: Keil 工程注册 modbus_cfg.c + 全量构建

**Files:**
- Modify: `project/CH579_USB_Host_HID.uvprojx`

- [ ] **Step 1: 将 modbus_cfg.c 加入 bsp 组**

在 `<GroupName>bsp</GroupName>` 组内 `modbus_rtu.c` 文件条目（`<FilePath>..\bsp\modbus_rtu.c</FilePath>`）之后追加：

```xml
            <File>
              <FileName>modbus_cfg.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\bsp\modbus_cfg.c</FilePath>
            </File>
```

- [ ] **Step 2: 清理缓存并全量构建**

```bash
Remove-Item "project\Objects\CH579_USB_Host_HID_Target 1.dep" -ErrorAction SilentlyContinue
Remove-Item project\Objects\modbus_cfg*, project\Objects\modbus_rtu*, project\Objects\ascii_frame*, project\Objects\main*, project\Objects\keymap* -ErrorAction SilentlyContinue
& "D:\Keil_v5\UV4\UV4.exe" -b project\CH579_USB_Host_HID.uvprojx -j0 -o project\build_log.txt
Get-Content project\build_log.txt -Tail 6
```

Expected: `0 Error(s), 0 Warning(s)`；`project\Objects\modbus_cfg.o` 存在（无 `_1` 后缀）

- [ ] **Step 3: 提交**

```bash
git add project/CH579_USB_Host_HID.uvprojx
git commit -m "build: Keil 工程注册 modbus_cfg 模块"
```

---

### Task 7: TTL 实测验证（USB-TTL 直连 PA4/PA5，PA6 悬空）

**Files:** 无（用户执行硬件测试）

前置：USB-TTL 接 PA5↔RX、PA4↔TX、GND 共地；串口助手 9600 8N1 HEX 模式。

- [ ] **Step 1: 回归 — 数据区读取不受影响**

发：`01 03 00 00 00 05 85 C9`
预期：`01 03 0A 00 41 00 42 00 43 ...`（键盘敲 ABC 后，40001~3 为 'A''B''C'）

- [ ] **Step 2: 读配置区 — 默认地址**

发：`01 03 00 80 00 01 85 E2`
预期：`01 03 02 00 01 xx xx`（addr=1 默认值）

- [ ] **Step 3: 读配置区 — 默认波特率索引**

发：`01 03 00 81 00 01 D4 22`
预期：`01 03 02 00 00 xx xx`（baud=0 → 9600）

- [ ] **Step 4: 0x06 写地址 = 2**

发：`01 06 00 80 00 02 09 E3`
预期：应答原样回显 `01 06 00 80 00 02 09 E3`；UART1 打印 `KEY:` 正常
随后：发 `02 03 00 80 00 01 85 D1`（新地址 02 读配置区），预期返回 `02 03 02 00 02 xx xx`
再发旧地址帧 `01 03 00 00 00 05 85 C9` → 无应答（地址已改为 2）

- [ ] **Step 5: 断电重启验证掉电保存**

断电重新上电后，直接发：`02 03 00 80 00 01 85 D1`
预期：`02 03 02 00 02 xx xx`（地址仍为 2，DataFlash 生效）

- [ ] **Step 6: 0x06 写波特率 = 1 (19200)**

发（仍在地址 2 下）：`02 06 00 81 00 01 18 11`
预期：应答回显 `02 06 00 81 00 01 18 11`；随后主站切 19200 重连，发 `02 03 00 81 00 01 D4 11` → `02 03 02 00 01 xx xx`（立即生效）

- [ ] **Step 7: 非法值注入**

| 帧 | 预期 |
|---|---|
| 地址=0：`01 06 00 80 00 00 88 22` | 异常 `01 86 03 xx xx`（0x03 非法数据值） |
| 地址=250：`01 06 00 80 00 FA 08 61` | 异常 `01 86 03 xx xx` |
| 波特率=5：`01 06 00 81 00 05 19 E1` | 异常 `01 86 03 xx xx` |
| 写数据区 0x0001：`01 06 00 01 00 41 18 3A` | 异常 `01 86 02 xx xx`（0x02 非法地址） |
| 读配置区数量=2：`01 03 00 80 00 02 C5 E3` | 异常 `01 83 02 xx xx` |

- [ ] **Step 8: 看门狗验证**

- 正常运行 30 秒：无复位，UART1 心跳 `H:` 持续输出，上电打印 `reset: normal`
- 人为饿狗（可临时注释喂狗行烧录验证）→ 上电打印 `WDOG reset`（S3 生效）
- 恢复喂狗行后回归正常

- [ ] **Step 9: 恢复默认配置**（验证完成后，将地址/波特率改回默认以便后续 485 联调）

1. 当前地址 2 下把地址改回 1：发 `02 06 00 80 00 01 49 D1`，预期回显
2. 地址回 1 后把波特率改回 9600：发 `01 06 00 81 00 00 D9 E2`，预期回显；主站切回 9600
3. 回归确认：发 `01 03 00 00 00 05 85 C9` → 正常应答数据区

- [x] **Task 7 实施结果（2026-08-11 实测，全部通过）**

**看门狗实测发现并修复（f8934bd）**：插入键盘触发 USB 枚举，`InitRootDevice()` 阻塞超 1s（> 8 位计数上限 1.02s）→ 饿狗复位 → 重启再枚举死循环，串口停止打印。修复：`usb_host_hid.c` 枚举分支前后 `WWDG_ResetCfg(DISABLE)` / `SetCounter(250)+ResetCfg(ENABLE)`，枚举窗口有库超时兜底。修复后插入键盘正常枚举打印。

**配置功能实测记录**：
1. 读配置区默认值：`01 03 00 80 00 01 85 E2` → `01 03 02 00 01`（addr=1）
2. 0x06 写地址=2：`01 06 00 80 00 02 09 E3` → 回显；断电重插后 `02 03 00 80 00 01 85 D1` → `00 02`（**DataFlash 掉电保存生效**）
3. 0x06 写波特率=1(19200)：`02 06 00 81 00 01 18 11` → 回显；切 19200 后 `02 03 00 81 00 01 D4 11` → `00 01`（**立即生效**）
4. 非法值（当前地址 2 下发，地址 1 的帧被静默丢弃=地址不匹配正确行为）：地址=0/250、波特率=5 → 异常 `0x86 03`；写数据区 0x0001 → 异常 `0x86 02`；读配置区数量=2 → 异常 `0x83 02`
5. 恢复默认：`02 06 00 80 00 01 49 D1`（地址→1）+ `01 06 00 81 00 00 D9 E2`（波特率→9600）；切回 9600 验证 `01 03 00 80 00 01 85 E2` → `00 01`、`01 03 00 81 00 01 D4 22` → `00 00`、数据区回归正常
6. 上电复位诊断：断电重启打印 `reset: normal`（RST_FLAG_WTR 精确比较修正 fb30e88 生效）
7. 最终审查防御加固（05b0008）：配置加载范围校验（防损坏值越界查波特率表）、看门狗先重载计数再使能（防计数恰为 0 瞬时误复位）

**结论**：看门狗 + 参数可配置掉电保存全部验收通过，配置已恢复默认（addr=1, 9600），代码 0 Error 0 Warning。

---

## 注意事项

- **CRC 计算**：所有测试帧 CRC 需按实际字节计算（Poly 0xA001 位运算，低字节在前）；本计划中已给默认地址帧的 CRC，改地址后的帧用 `python -c` 或在线工具重新计算
- **0x06 异常码区分**：`modbus_cmd06_ack` 返回 0=非法数据值(0x03)、返回 0xFFFF=非法地址(0x02)，勿混淆
- **保存失败仅警告**：`modbus_cfg_save()` 失败时内存配置仍生效（本次会话可用），仅 UART1 打印 `cfg save fail`，应答正常回显（规格文档已确认）
- **写波特率立即生效**：重配瞬间当前会话掉线，主站需切新波特率重连
- **编码**：新文件 UTF-8 无 BOM；不动 GBK 库文件
- **对象重名警告**：若构建出现 "_1.o" 重命名，删除 `project\Objects\CH579_USB_Host_HID_Target 1.dep` 缓存重建
- **DataFlash 地址**：`0x3E800` 为数据区起始，仅存本配置结构（8 字节），512B 扇区擦除，不与其他用途冲突
