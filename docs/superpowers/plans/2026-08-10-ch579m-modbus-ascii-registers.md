# CH579M USB 键盘 → Modbus RTU 从机实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 USB 键盘输入的字符按顺序存入 Modbus 保持寄存器 40001~40128（低字节 ASCII、高字节 0），CH579M 作为 Modbus RTU 从机（地址 1、9600 8N1、RS485）响应 0x03 读保持寄存器。

**Architecture:** 新增两个独立模块：`user/ascii_frame.c`（ASCII 帧缓冲=寄存器组映射，Enter 立即提交 / 500ms 空闲超时提交 / Backspace 删字）和 `bsp/modbus_rtu.c`（UART3 + PA6 RS485 方向控制 + 帧解析/CRC16/0x03 应答）。`user/main.c` 在 `process_key_events` 中把可打印字符写入帧缓冲，主循环每轮驱动超时判定与 Modbus 轮询。依赖方向单向：main → ascii_frame / modbus_rtu。

**Tech Stack:** C99 (ARMCC V5.06), CH579M 外设库 (CH57x_common.h/CH57x_usbhost.h), Keil MDK UV4 命令行构建, Modbus RTU 协议

**硬件:** TXD3=PA5（推挽）、RXD3=PA4（上拉输入）、PA6=RE/DE 控制（推挽，经 1k 电阻，5V 供电收发器）

**参考:** `E:\MeWork\CH579M\modbus.c`（协议层结构：CRC16 位算法 / 帧处理函数 / 寄存器映射集中）

---

### Task 1: ASCII 帧缓冲模块 ascii_frame.c/.h

**Files:**
- Create: `user/ascii_frame.h`
- Create: `user/ascii_frame.c`

- [ ] **Step 1: 创建 `user/ascii_frame.h`**

```c
#ifndef __ASCII_FRAME_H
#define __ASCII_FRAME_H

#include "CH57x_common.h"

#define ASCII_FRAME_SIZE   128          /* 寄存器组大小: 40001~40128 */
#define ASCII_IDLE_MS      500          /* 空闲超时提交时间, 可调 */

void ascii_frame_init( void );          /* 清零寄存器组与状态 */
void ascii_frame_putch( char c );       /* 追加一个字符(满 128 忽略), 重置空闲计数 */
void ascii_frame_backspace( void );     /* 删除上一个字符(回退并清零) */
void ascii_frame_commit( void );        /* 立即提交: 余段清零 + 复位 index */
void ascii_frame_poll( void );          /* 主循环调用(约 2ms 一次): 空闲超时自动提交 */
UINT8 ascii_frame_get( UINT8 index );   /* 读寄存器低字节(index 0~127) */

#endif /* __ASCII_FRAME_H */
```

- [ ] **Step 2: 创建 `user/ascii_frame.c`**

```c
/*******************************************************************************
* ascii_frame.c — ASCII 帧缓冲(Modbus 40001~40128 寄存器组映射)
* 填充即写入 reg_ascii[index]; 提交 = 余段清零 + index 复位
*******************************************************************************/
#include "CH57x_common.h"
#include "ascii_frame.h"

static UINT8  reg_ascii[ ASCII_FRAME_SIZE ];
static UINT8  frame_idx = 0;    /* 当前帧写入位置 */
static UINT16 idle_cnt  = 0;    /* 空闲心跳计数(主循环 2ms 一次) */

void ascii_frame_init( void )
{
    UINT8 i;
    for( i = 0; i < ASCII_FRAME_SIZE; i ++ ) reg_ascii[ i ] = 0;
    frame_idx = 0;
    idle_cnt  = 0;
}

void ascii_frame_putch( char c )
{
    if( frame_idx >= ASCII_FRAME_SIZE ) return;      /* 满帧忽略 */
    reg_ascii[ frame_idx ++ ] = ( UINT8 )c;
    idle_cnt = 0;
}

void ascii_frame_backspace( void )
{
    if( frame_idx > 0 )
    {
        frame_idx --;
        reg_ascii[ frame_idx ] = 0;
    }
}

void ascii_frame_commit( void )
{
    UINT8 i;
    for( i = frame_idx; i < ASCII_FRAME_SIZE; i ++ ) reg_ascii[ i ] = 0;
    frame_idx = 0;
    idle_cnt  = 0;
}

void ascii_frame_poll( void )
{
    if( ++idle_cnt >= ( ASCII_IDLE_MS / 2 ) )        /* 500ms / 2ms 心跳 */
        ascii_frame_commit();
}

UINT8 ascii_frame_get( UINT8 index )
{
    if( index >= ASCII_FRAME_SIZE ) return 0;
    return reg_ascii[ index ];
}
```

- [ ] **Step 3: 提交**

```bash
git add user/ascii_frame.c user/ascii_frame.h
git commit -m "feat: ASCII 帧缓冲模块(40001~40128 寄存器组映射, 空闲超时提交)"
```

---

### Task 2: Modbus RTU 从机协议层 modbus_rtu.c/.h

**Files:**
- Create: `bsp/modbus_rtu.h`
- Create: `bsp/modbus_rtu.c`

- [ ] **Step 1: 创建 `bsp/modbus_rtu.h`**

```c
#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "CH57x_common.h"

#define MODBUS_ADDR         1           /* 从机地址, 可调 */
#define MODBUS_BAUD         9600

void modbus_rtu_init( void );           /* UART3 + RS485 引脚初始化 */
void modbus_rtu_poll( void );           /* 主循环调用: 收帧/解析/应答 */

#endif /* __MODBUS_RTU_H */
```

- [ ] **Step 2: 创建 `bsp/modbus_rtu.c`**

```c
/*******************************************************************************
* modbus_rtu.c — Modbus RTU 从机(地址 1, 9600 8N1, RS485)
* 仅支持功能码 0x03 读保持寄存器(40001~40128)
* 硬件: TXD3=PA5 推挽 / RXD3=PA4 上拉输入 / PA6=RE/DE 控制(1k 电阻, 5V 供电)
* 帧边界: 3.5 字符空闲(9600 ≈ 3.65ms, 主循环 2ms 心跳判 2 次无字节)
*******************************************************************************/
#include "CH57x_common.h"
#include "usb_host_hid.h"       /* 仅用 uart_debug.h 实际无需; 见 Step 3 修正 */
#include "uart_debug.h"
#include "modbus_rtu.h"
#include "ascii_frame.h"

#define RX_BUF_SIZE   256
#define TX_BUF_SIZE   256

static UINT8  rbuf[ RX_BUF_SIZE ];      /* 接收帧缓冲 */
static UINT8  rx_cnt = 0;               /* 已收字节数 */
static UINT8  rx_idle = 0;              /* 空闲心跳计数 */
static UINT8  tbuf[ TX_BUF_SIZE ];      /* 应答缓冲 */

/*******************************************************************************
* CRC16 Modbus (Poly 0xA001, 位运算)
*******************************************************************************/
static UINT16 CRC16( const UINT8 *dataIn, UINT16 length )
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
* 0x03 读保持寄存器应答. 返回应答长度; 失败返回 0
* 寄存器映射: Modbus 地址 0~127 ↔ 40001~40128 ↔ ascii_frame_get(index)
*******************************************************************************/
static UINT16 modbus_cmd03_ack( const UINT8 *pRec, UINT8 *pAck )
{
    UINT16 i, RegAddr, Cnt;
    UINT16 AckLen;
    UINT16 CrcTmp;

    RegAddr = ( UINT16 )( ( pRec[ 2 ] << 8 ) | pRec[ 3 ] );
    Cnt     = ( UINT16 )( ( pRec[ 4 ] << 8 ) | pRec[ 5 ] );

    if( Cnt == 0 || Cnt > 125 ) return 0;                       /* 数量非法 */
    if( ( UINT32 )RegAddr + Cnt > ASCII_FRAME_SIZE ) return 0;  /* 越界 */

    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = 0x03;
    pAck[ 2 ] = ( UINT8 )( Cnt * 2 );                           /* 数据字节数 */
    AckLen = 3;
    for( i = 0; i < Cnt; i ++ )
    {
        pAck[ AckLen ++ ] = 0x00;                               /* 高字节 */
        pAck[ AckLen ++ ] = ascii_frame_get( ( UINT8 )( RegAddr + i ) );  /* 低字节 ASCII */
    }
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}

/*******************************************************************************
* 异常应答 (功能码|0x80). 返回应答长度
*******************************************************************************/
static UINT16 modbus_exception( UINT8 func, UINT8 code, UINT8 *pAck )
{
    UINT16 CrcTmp, AckLen;
    pAck[ 0 ] = MODBUS_ADDR;
    pAck[ 1 ] = ( UINT8 )( func | 0x80 );
    pAck[ 2 ] = code;
    AckLen = 3;
    CrcTmp = CRC16( pAck, AckLen );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp & 0xFF );
    pAck[ AckLen ++ ] = ( UINT8 )( CrcTmp >> 8 );
    return AckLen;
}

/*******************************************************************************
* 帧处理: 地址匹配 + CRC 校验 + 功能码分发. 返回应答长度; 0=不应答
*******************************************************************************/
static UINT16 modbus_frame_process( const UINT8 *pRec, UINT8 len, UINT8 *pAck )
{
    UINT16 CrcTmp;

    if( len < 8 ) return 0;                                     /* 帧太短 */
    if( pRec[ 0 ] != MODBUS_ADDR ) return 0;                    /* 地址不匹配(含广播0不响应) */

    CrcTmp = ( UINT16 )( ( pRec[ len - 1 ] << 8 ) | pRec[ len - 2 ] );
    if( CrcTmp != CRC16( pRec, len - 2 ) ) return 0;            /* CRC 失败 */

    if( pRec[ 1 ] == 0x03 )
    {
        UINT16 n = modbus_cmd03_ack( pRec, pAck );
        if( n == 0 ) return modbus_exception( 0x03, 0x02, pAck );
        return n;
    }
    return modbus_exception( pRec[ 1 ], 0x01, pAck );           /* 功能码非法 */
}

/*******************************************************************************
* UART3 发送(RS485): DE 拉高 → 稳定 → 逐字节 → 等移位完 → DE 拉低
*******************************************************************************/
static void uart3_send( const UINT8 *buf, UINT16 len )
{
    UINT16 i;
    GPIOA_SetBits( GPIO_Pin_6 );                                /* DE/RE 使能发送 */
    mDelaymS( 2 );                                              /* 收发器稳定 */
    for( i = 0; i < len; i ++ )
    {
        while( R8_UART3_TFC == UART_FIFO_SIZE ) ;               /* FIFO 满等待 */
        R8_UART3_THR = buf[ i ];
    }
    while( !( R8_UART3_LSR & RB_LSR_TX_ALL_EMP ) ) ;            /* 最后字节移位完 */
    GPIOA_ClearBits( GPIO_Pin_6 );                              /* 回接收 */
}

/*******************************************************************************
* 接口实现
*******************************************************************************/
void modbus_rtu_init( void )
{
    GPIOA_ModeCfg( GPIO_Pin_4, GPIO_ModeIN_PU );                /* RXD3 上拉输入 */
    GPIOA_ModeCfg( GPIO_Pin_5, GPIO_ModeOut_PP_5mA );           /* TXD3 推挽 */
    GPIOA_ModeCfg( GPIO_Pin_6, GPIO_ModeOut_PP_5mA );           /* RE/DE 推挽 */
    GPIOA_ClearBits( GPIO_Pin_6 );                              /* 初始接收方向 */
    UART3_DefInit();
    UART3_BaudRateCfg( MODBUS_BAUD );
    rx_cnt = 0;
    rx_idle = 0;
}

void modbus_rtu_poll( void )
{
    UINT8  b;
    UINT16 n;

    if( R8_UART3_LSR & RB_LSR_DATA_RDY )
    {
        while( R8_UART3_LSR & RB_LSR_DATA_RDY )                 /* 读空 FIFO */
        {
            b = R8_UART3_RBR;
            if( rx_cnt < RX_BUF_SIZE ) rbuf[ rx_cnt ++ ] = b;
            rx_idle = 0;
        }
    }
    else if( rx_cnt > 0 )
    {
        if( ++rx_idle >= 2 )                                    /* 3.5 字符空闲 ≈ 2 次心跳 */
        {
            n = modbus_frame_process( rbuf, rx_cnt, tbuf );
            if( n > 0 ) uart3_send( tbuf, n );                  /* 仅地址+CRC 匹配才应答 */
            rx_cnt = 0;
            rx_idle = 0;
        }
    }
}
```

- [ ] **Step 3: 修正头文件包含**（Step 2 中 `#include "usb_host_hid.h"` 行是多余的，modbus_rtu 只依赖 uart_debug.h 与 ascii_frame.h，删除该行后重新保存文件）

```c
#include "CH57x_common.h"
#include "uart_debug.h"
#include "modbus_rtu.h"
#include "ascii_frame.h"
```

- [ ] **Step 4: 提交**

```bash
git add bsp/modbus_rtu.c bsp/modbus_rtu.h
git commit -m "feat: Modbus RTU 从机协议层(UART3/RS485/CRC16/0x03应答/异常码)"
```

---

### Task 3: main.c 粘合 — 键盘事件写帧 + 主循环驱动

**Files:**
- Modify: `user/main.c`（`process_key_events` 函数体、`main()` 初始化段与主循环）

- [ ] **Step 1: 添加头文件包含**

在 `user/main.c` 顶部 include 块中，`#include "keymap.h"` 之后新增两行：

```c
#include "ascii_frame.h"
#include "modbus_rtu.h"
```

- [ ] **Step 2: 修改 `process_key_events` — 键盘按下事件写 ASCII 帧**

在 `process_key_events` 的 `else` 分支（键盘 ifidx==0）中，`caps_lock` 切换之后、`up_puts( "KEY: ..." )` 打印之前，插入帧写入逻辑。替换原函数为：

```c
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

            /* --- ASCII 帧写入 (Modbus 40001~40128) --- */
            if( ev.type == KEV_PRESS )
            {
                UINT8  sh = ( ( ev.mods >> 1 ) | ( ev.mods >> 5 ) ) & 1;
                if( ev.usage == 0x28 )                      /* Enter: 立即提交 */
                    ascii_frame_commit();
                else if( ev.usage == 0x2A )                 /* Backspace: 删字 */
                    ascii_frame_backspace();
                else
                {
                    const char *s = key_display( ev.usage, sh );
                    if( ev.usage == 0x2C )                  /* Space: 写空格 */
                        ascii_frame_putch( ' ' );
                    else if( s[ 0 ] >= 0x20 && s[ 0 ] <= 0x7E && s[ 1 ] == 0 )
                        ascii_frame_putch( s[ 0 ] );        /* 单字符可打印才写入 */
                    /* 多字符名称(F1/Esc 等)与非 ASCII 键忽略 */
                }
            }

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
```

- [ ] **Step 3: 修改 `main()` — 初始化与主循环**

在 `UART1_DefInit();` 与 `up_puts( "\r\nMK5 USB-HID Host start\r\n" );` 之间插入初始化：

```c
    ascii_frame_init();
    modbus_rtu_init();
```

在主循环 `usb_hid_poll();` 之后、`if( usb_hid_device_ready() )` 块之后（`mDelaymS( 2 );` 之前）插入驱动调用：

```c
        ascii_frame_poll();
        modbus_rtu_poll();
```

- [ ] **Step 4: Keil CLI 构建验证**

```bash
Remove-Item project\Objects\CH579_USB_Host_HID.axf -ErrorAction SilentlyContinue
Start-Process "D:\Keil_v5\UV4\UV4.exe" -ArgumentList "-b","project\CH579_USB_Host_HID.uvprojx","-j0" -Wait -NoNewWindow
```

Expected: `".\Objects\CH579_USB_Host_HID.axf" - 0 Error(s)`（其余为库自身 warning 可接受）

- [ ] **Step 5: 提交**

```bash
git add user/main.c
git commit -m "feat: 键盘 ASCII 写入 Modbus 帧缓冲(Enter 提交/Backspace 删字/可打印过滤)"
```

---

### Task 4: Keil 工程添加新文件并验证

**Files:**
- Modify: `project/CH579_USB_Host_HID.uvprojx`

- [ ] **Step 1: 将 ascii_frame.c 加入 user 组**

在 `<GroupName>user</GroupName>` 组的 `keymap.c` 文件条目后追加：

```xml
            <File>
              <FileName>ascii_frame.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\user\ascii_frame.c</FilePath>
            </File>
```

- [ ] **Step 2: 将 modbus_rtu.c 加入 bsp 组**

在 `<GroupName>bsp</GroupName>` 组的 `uart_debug.c` 文件条目后追加：

```xml
            <File>
              <FileName>modbus_rtu.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\bsp\modbus_rtu.c</FilePath>
            </File>
```

- [ ] **Step 3: 清理旧缓存并全量构建**

```bash
Remove-Item "project\Objects\CH579_USB_Host_HID_Target 1.dep" -ErrorAction SilentlyContinue
Remove-Item project\Objects\modbus_rtu*, project\Objects\ascii_frame* -ErrorAction SilentlyContinue
Start-Process "D:\Keil_v5\UV4\UV4.exe" -ArgumentList "-b","project\CH579_USB_Host_HID.uvprojx","-j0" -Wait -NoNewWindow
```

Expected: `0 Error(s)`；`project\Objects\modbus_rtu.o` 与 `project\Objects\ascii_frame.o` 存在（无 `_1` 后缀）

- [ ] **Step 4: 提交**

```bash
git add project/CH579_USB_Host_HID.uvprojx
git commit -m "build: Keil 工程加入 modbus_rtu 与 ascii_frame 模块"
```

---

### Task 5: 硬件联调验证清单

**Files:** 无（用户执行硬件测试）

- [ ] **Step 1: TTL 级协议验证（可选）**

UART3 的 PA4/PA5 用 USB-TTL 直连 PC（跳过 485 收发器），Modbus Poll 或串口助手发包验证协议正确性。注意此时无 PA6 方向切换问题（全双工 TTL）。

- [ ] **Step 2: RS485 联调**

RS485 转 USB 接 PC，Modbus Poll 读 40001 起，逐项验证：
- 键盘输入字符 → 低字节 ASCII 正确、高字节 0
- 无字符寄存器 = 0
- Enter → 立即提交；停止输入 500ms → 超时提交
- Backspace → 前一位清 0
- 分段读：40001~40125 与 40126~40128 两次读取（单次 ≤125）
- 错误注入：错误 CRC、错误地址 → 无响应；越界地址/数量=0/数量>125 → 异常码 0x02；非 0x03 功能码 → 异常码 0x01

- [ ] **Step 3: 方向切换时序验证**

示波器观察 PA6 与 TXD3：应答发送期间 PA6 为高，最后一位完整发出后才拉低；收帧期间 PA6 保持低。

- [ ] **Step 4: 回归**

UART1 调试打印、USB 枚举打印、KEY/MEDIA 事件、MK5 全速键盘均不受影响；`p/d/e` 串口命令正常。

- [ ] **Step 5: 推送**

```bash
git push origin master
```

---

## 注意事项

- **主循环阻塞**：应答发送是阻塞的（≤252 字节 @9600 ≈ 262ms）。期间不轮询 USB，属可接受的短时阻塞；若需改善可在后续版本改为非阻塞发送
- **3.5 字符空闲判定**：9600 下 ≈ 3.65ms，主循环 2ms 心跳，连续 2 次轮询无新字节即判帧结束（≈4ms），满足标准
- **编码**：新文件用 UTF-8 无 BOM（与 user/main.c 一致）；不要改动 GBK 编码的库文件
- **对象重名警告**：若构建日志出现 "object file renamed ... _1.o"，删除 `project\Objects\CH579_USB_Host_HID_Target 1.dep` 缓存后重建
- **CRC 与地址**：`modbus_frame_process` 先匹配地址再验 CRC（与参考 modbus.c 顺序一致）；广播地址 0 不响应
