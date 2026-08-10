# CH579M 适配 78 键 USB HID 键盘 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 78 键低速键盘在 CH579M 主机上枚举后全部传输无应答（0x20）的问题，并加按键事件抽象层、多媒体键解码与完整键位映射表。

**Architecture:** 根因是库 `InitRootDevice()` 成功路径末尾强制 `SetUsbSpeed(1)` 把物理层翻回全速，而端口仍标记低速，导致低速键盘完全收不到令牌。先用 main.c 一行验证假设，再修库根因；随后在 main.c 内实现事件环形队列 + 键盘/Consumer 报告解析器 + 完整 usage 映射表，输出仅 UART1 串口。

**Tech Stack:** C（Keil MDK / ARMCC，ARM Cortex-M0），WCH CH579 官方 USB Host 库（CH57x_usbhostBase/Class），UART1 115200。

**Spec:** `docs/superpowers/specs/2026-08-10-ch579m-78key-keyboard-design.md`

**验证方式说明：** 裸机工程无单元测试框架，验证 = Keil CLI 编译通过 + 烧录后串口日志证据（COM4/115200）。每步给出预期串口输出。

**Git 说明：** 当前目录不是 git 仓库。Task 1 第 1 步为可选的 `git init`；若用户不初始化仓库，后续 commit 步骤可跳过。

---

### Task 1: 诊断打印 + 验证假设（临时改动，不改库）

**Files:**
- Modify: `user/main.c`（InitRootDevice 成功分支内、HID_SetIdle 调用之前插入）

- [ ] **Step 1: （可选）初始化 git 仓库**

Run:
```bash
git init
git add -A
git commit -m "chore: 快照当前可用状态（MK5 正常）"
```
（若用户不需要版本管理，跳过本步及后续所有 commit 步骤。）

- [ ] **Step 2: 插入诊断打印与验证行**

打开 `user/main.c`，在以下代码块之后（第 315 行 `up_printf( "%x\r\n", ThisUsbDev.GpVar[ 1 ] );` 之后）插入：

```c
                /* 诊断：枚举后的速度状态（0=低速 1=全速） */
                up_puts( "spd=" );
                up_printf( "%x", ThisUsbDev.DeviceSpeed );
                up_puts( " UC_LS=" );
                up_printf( "%x", ( R8_USB_CTRL & RB_UC_LOW_SPEED ) ? 1 : 0 );
                up_puts( " UH_LS=" );
                up_printf( "%x\r\n", ( R8_UHOST_CTRL & RB_UH_LOW_SPEED ) ? 1 : 0 );

                /* 验证假设（临时）：恢复设备实际速度，库修复后移除 */
                SetUsbSpeed( ThisUsbDev.DeviceSpeed );
```

- [ ] **Step 3: CLI 编译**

Run:
```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -j0 -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt"; Get-Content "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" | Select-Object -Last 5
```
Expected: 日志末尾 `0 Error(s), 0 Warning(s)`（若有警告需检查无实质问题）。

- [ ] **Step 4: 烧录 + 串口取证**

用 Keil（WCH-LinkE）烧录，插 78 键键盘，串口 COM4/115200 观察（可用 /serial skill 或任意串口终端）。

**假设成立**的日志证据：
```
spd=0 UC_LS=0 UH_LS=1        ← 速度检测=低速，但 USB_CTRL 已被翻回全速、UHOST_CTRL 仍低速
if0: 00000000 00000000 00000000   ← 从 20 20 20 变全 0
if1: 00000000 00000000 00000000
H: poll=500 ok=0 nak=500 err=0    ← 心跳从 err=500 变 nak=500（空闲 NAK 正常）
```
按按键应出现 `KEY: ...` 输出。

**假设不成立**：`if0: 20 20 20` 依旧、轮询仍全 err → **停止本计划**，向用户汇报，后备分支：试 `R8_UH_SETUP |= RB_UH_PRE_PID_EN`、强制全速、换 USB 线、查 VBUS 供电。

- [ ] **Step 5: 汇报结果**

假设成立 → 向用户展示日志，进入 Task 2（修库根因）。

---

### Task 2: 修库根因 + 移除临时验证行 + 双键盘回归

**Files:**
- Modify: `library/StdPeriphDriver/CH57x_usbhostClass.c`（6 处成功路径）
- Modify: `user/main.c`（移除 Task 1 的临时验证行，保留诊断打印）

- [ ] **Step 1: 修库 6 处**

在 `CH57x_usbhostClass.c` 中，将以下 6 处成功路径的 `SetUsbSpeed( 1 );` 全部替换为 `SetUsbSpeed( ThisUsbDev.DeviceSpeed );`：

| 行号 | 所在分支 |
|---|---|
| 95 | U 盘成功 |
| 106 | 打印机成功 |
| 123 | HID 键盘成功 |
| 130 | HID 鼠标成功 |
| 145 | HUB 成功 |
| 156 | 未知设备成功 |

替换后示例（键盘分支）：
```c
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        if ( if_cls == 1 ) {
                        ThisUsbDev.DeviceType = DEV_TYPE_KEYBOARD;
//	��һ����ʼ��,�����豸����ָʾ��LED��
                        PRINT( "USB-Keyboard Ready\n" );
                        SetUsbSpeed( ThisUsbDev.DeviceSpeed );
                        return( ERR_SUCCESS );
                        }
```

注意：失败清理路径（第 171 行 `SetUsbSpeed( 1 );`）**保持不动**。仅改这 6 处，用 grep 确认无遗漏：

```powershell
Get-ChildItem -Recurse -Include *.c -Path library | Select-String -Pattern "SetUsbSpeed" | ForEach-Object { "$($_.Path):$($_.LineNumber): $($_.Line.Trim())" }
```
Expected: 只剩 usbhostClass.c 中 7 处（6 处已改 + 第 171 行保留）及 usbhostBase.c 中的定义。

- [ ] **Step 2: 移除 main.c 临时验证行**

删除 Task 1 插入的注释行 `/* 验证假设（临时）：恢复设备实际速度，库修复后移除 */` 与其下 `SetUsbSpeed( ThisUsbDev.DeviceSpeed );` 一行。诊断打印（spd/UC_LS/UH_LS）**保留**。

- [ ] **Step 3: 编译**

Run:
```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -j0 -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt"; Get-Content "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" | Select-Object -Last 5
```
Expected: `0 Error(s)`。

- [ ] **Step 4: 双键盘回归**

烧录后依次插：
1. **78 键键盘**（低速）：预期 `spd=0`，`if0/if1` 全 0，心跳 nak 正常，按键有输出
2. **MK5 小键盘**（全速）：预期 `spd=1`，功能与改动前一致（SetUsbSpeed(1) 等效原行为）

Expected: 两者都正常工作即回归通过。

- [ ] **Step 5: Commit**

```bash
git add user/main.c library/StdPeriphDriver/CH57x_usbhostClass.c
git commit -m "fix(usbhost): 枚举成功后按检测速度设置 USB 物理层，修复低速键盘无应答"
```

---

### Task 3: 按键事件抽象层 + 轮询重写 + 报告描述符取证

**Files:**
- Modify: `user/main.c`

- [ ] **Step 1: 添加事件结构与环形队列**

在 `main.c` 顶部 `#include "CH579UFI.H"` 之后、`RxBuffer` 定义附近添加：

```c
/*******************************************************************************
* 按键事件抽象层: 事件环形队列 + 解析/消费解耦
*******************************************************************************/
#define KEV_PRESS     0
#define KEV_RELEASE   1
#define KEY_EV_QUEUE_SIZE  16

typedef struct
{
    UINT8   type;      /* KEV_PRESS / KEV_RELEASE */
    UINT8   mods;      /* bit0 Shift bit1 Ctrl bit2 Alt bit3 GUI */
    UINT8   ifidx;     /* 0=键盘 1=多媒体 */
    UINT16  usage;     /* HID usage 码 */
} key_event_t;

static key_event_t  evq[ KEY_EV_QUEUE_SIZE ];
static UINT8  evq_head = 0, evq_tail = 0, evq_cnt = 0;
static UINT32 ev_drop = 0;

static UINT8 kbd_ev_push( UINT8 type, UINT8 mods, UINT8 ifidx, UINT16 usage )
{
    if( evq_cnt >= KEY_EV_QUEUE_SIZE ) { ev_drop ++; return 0; }
    evq[ evq_head ].type  = type;
    evq[ evq_head ].mods  = mods;
    evq[ evq_head ].ifidx = ifidx;
    evq[ evq_head ].usage = usage;
    evq_head = ( UINT8 )( ( evq_head + 1 ) % KEY_EV_QUEUE_SIZE );
    evq_cnt ++;
    return 1;
}

static UINT8 kbd_ev_pop( key_event_t *ev )
{
    if( evq_cnt == 0 ) return 0;
    *ev = evq[ evq_tail ];
    evq_tail = ( UINT8 )( ( evq_tail + 1 ) % KEY_EV_QUEUE_SIZE );
    evq_cnt --;
    return 1;
}
```

- [ ] **Step 2: 添加解析器（键盘 + Consumer）与 4 位 hex 打印助手**

在 `PollKeyboard` 函数定义之前添加：

```c
/*******************************************************************************
* 键盘报告解析: 与上次按下集合对比生成 按下/释放 事件
* 报告格式: [0]=修饰键 [1]=保留 [2..7]=按键码
*******************************************************************************/
static void parse_kbd_report( UINT8 *buf, UINT8 len )
{
    static UINT8 last_keys[ 6 ];
    static UINT8 last_mods = 0;
    UINT8  i, j, mods, found;

    if( len < 3 ) return;
    mods = buf[ 0 ] & 0x0F;

    /* 释放: 上次有而本次无 */
    for( i = 0; i < 6; i ++ )
    {
        if( last_keys[ i ] == 0 ) continue;
        found = 0;
        for( j = 2; j < len; j ++ )
            if( buf[ j ] == last_keys[ i ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_RELEASE, last_mods, 0, last_keys[ i ] );
    }
    /* 按下: 本次有而上次无 */
    for( j = 2; j < len; j ++ )
    {
        if( buf[ j ] == 0 ) continue;
        found = 0;
        for( i = 0; i < 6; i ++ )
            if( last_keys[ i ] == buf[ j ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_PRESS, mods, 0, buf[ j ] );
    }
    /* 更新快照 */
    for( i = 0; i < 6; i ++ ) last_keys[ i ] = 0;
    for( i = 0, j = 2; j < len && i < 6; j ++ ) last_keys[ i ++ ] = buf[ j ];
    last_mods = mods;
}

/*******************************************************************************
* Consumer 页 usage 名字表 (Task 5 实测后校正)
*******************************************************************************/
static const char *consumer_usage_name( UINT16 u )
{
    switch( u )
    {
        case 0x00E2: return "Mute";
        case 0x00E9: return "Vol+";
        case 0x00EA: return "Vol-";
        case 0x00B5: return "Next";
        case 0x00B6: return "Prev";
        case 0x00B7: return "Stop";
        case 0x00CD: return "Play/Pause";
        case 0x0223: return "Home";
        case 0x0224: return "Back";
        default:     return 0;
    }
}

/*******************************************************************************
* Consumer 报告解析: 按 16 位 usage 数组解码 (与上次对比生成 按下/释放 事件)
* 若 Task 3 Step 6 抓到的报告描述符不是 16 位 usage 数组结构, 在此按实测调整
*******************************************************************************/
static void parse_consumer_report( UINT8 *buf, UINT8 len )
{
    static UINT16 last_cu[ 4 ];
    static UINT8  last_n = 0;
    UINT16 cur[ 4 ];
    UINT8  n = 0, i, j, found;

    if( len < 2 ) return;
    for( j = 0; j + 1 < len && n < 4; j += 2 )
    {
        UINT16 u = ( UINT16 )( buf[ j ] | ( buf[ j + 1 ] << 8 ) );
        if( u ) cur[ n ++ ] = u;
    }
    /* 释放 */
    for( i = 0; i < last_n; i ++ )
    {
        found = 0;
        for( j = 0; j < n; j ++ )
            if( cur[ j ] == last_cu[ i ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_RELEASE, 0, 1, last_cu[ i ] );
    }
    /* 按下 */
    for( j = 0; j < n; j ++ )
    {
        found = 0;
        for( i = 0; i < last_n; i ++ )
            if( last_cu[ i ] == cur[ j ] ) { found = 1; break; }
        if( !found )
            kbd_ev_push( KEV_PRESS, 0, 1, cur[ j ] );
    }
    for( i = 0; i < n; i ++ ) last_cu[ i ] = cur[ i ];
    last_n = n;
}

static void up_puthex4( UINT16 v )
{
    static const char hx[] = "0123456789ABCDEF";
    char tmp[5]; int i = 4; tmp[4] = 0;
    do { tmp[--i] = hx[v & 0xF]; v >>= 4; } while( i > 0 );
    up_puts( &tmp[i] );
}

/*******************************************************************************
* 消费: 弹空队列 → UART 打印
*******************************************************************************/
static void process_key_events( void )
{
    key_event_t ev;
    while( kbd_ev_pop( &ev ) )
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
            up_puts( "KEY:  [0] " );
            up_puts( ev.type == KEV_PRESS ? "DN " : "UP " );
            up_puts( "\"" );
            up_puts( scancode_to_str( ( UINT8 )ev.usage ) );
            up_puts( "\" (mods=" );
            up_puthex( ev.mods );
            up_puts( ")\r\n" );
        }
    }
}

static void dump_bytes( const UINT8 *p, UINT8 n )
{
    UINT8 i;
    for( i = 0; i < n; i ++ ) { up_puthex( p[ i ] ); up_puts( " " ); }
    up_puts( "\r\n" );
}
```

- [ ] **Step 3: 重写轮询函数（替换原 `PollKeyboard`）**

删除原 `PollKeyboard` 整个函数（含其 `last`/`last_len`/`last_ep` 去重逻辑），替换为：

```c
/* 轮询两个 IN 端点; 按来源接口分发到对应解析器 */
static void PollHIDEndpoints( void )
{
    UINT8  i, ep, s, len;

    for( i = 0; i < 2; i ++ )
    {
        ep = ( i == 0 ) ? ThisUsbDev.GpVar[ 0 ] : ThisUsbDev.GpVar[ 1 ];
        if( ep == 0 ) continue;

        diag_poll ++;
        s = USBHostTransact( ( USB_PID_IN << 4 ) | ep, RB_UH_R_AUTO_TOG, 50 );
        if( s != ERR_SUCCESS )
        {
            if( s == ( USB_PID_NAK | ERR_USB_TRANSFER ) ) { diag_nak ++; continue; }
            diag_err ++;
            if( s == ERR_USB_DISCON ) up_puts( "\r\ndev out\r\n" );
            continue;
        }

        diag_ok ++;
        len = R8_USB_RX_LEN;
        if( len > 8 ) len = 8;
        if( i == 0 ) parse_kbd_report( RxBuffer, len );
        else         parse_consumer_report( RxBuffer, len );
    }
}
```

- [ ] **Step 4: 改造 HID_GetReportDescr 以输出长度，并在初始化处 dump 报告描述符**

将现有 `HID_GetReportDescr`（main.c 第 195-204 行）改为：

```c
static UINT8 HID_GetReportDescr( UINT8 infc, UINT8 *retlen )
{
    UINT8  s;
    /* bmRequestType=0x81 bRequest=0x06 wValue=0x2200 wLength=64 */
    const UINT8 getrep[] = { 0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0x40, 0x00 };
    CopySetupReqPkg( (PCHAR)getrep );
    pSetupReq -> wIndex = infc;
    s = HostCtrlTransfer( Com_Buffer, retlen );
    return( s );
}
```

将 main() 中的初始化块（第 317-333 行）替换为：

```c
                kbd_ifnum = ( ThisUsbDev.GpVar[ 2 ] == 0xFF ) ? 0 : ThisUsbDev.GpVar[ 2 ];

                up_puts( "if0: " );
                up_printf( "%x %x ",
                    ( UINT8 )HID_SetIdle( kbd_ifnum, 0x0A ),
                    ( UINT8 )HID_SetProtocol( kbd_ifnum, 0 ) );
                s = HID_GetReportDescr( kbd_ifnum, &rlen );
                up_printf( "%x\r\n", s );
                if( s == ERR_SUCCESS ) { up_puts( "rep0: " ); dump_bytes( Com_Buffer, rlen ); }

                if( ThisUsbDev.GpVar[ 1 ] != 0 )
                {
                    up_puts( "if1: " );
                    up_printf( "%x %x ",
                        ( UINT8 )HID_SetIdle( 1, 0x0A ),
                        ( UINT8 )HID_SetProtocol( 1, 0 ) );
                    s = HID_GetReportDescr( 1, &rlen );
                    up_printf( "%x\r\n", s );
                    if( s == ERR_SUCCESS ) { up_puts( "rep1: " ); dump_bytes( Com_Buffer, rlen ); }
                }
```

并在 main() 的局部变量声明区（`UINT8 s;` 之后）添加：`UINT8 rlen;`

- [ ] **Step 5: 主循环接线（轮询 → 事件 → 打印；扩展串口命令）**

将主循环中的 `PollKeyboard();`（第 343 行）替换为：

```c
                PollHIDEndpoints();
                process_key_events();
```

将串口命令块（第 348-364 行）替换为：

```c
        if( R8_UART1_LSR & RB_LSR_DATA_RDY )
        {
            UINT8 c = R8_UART1_RBR;
            if( c == 'p' )
            {
                up_puts( "st=" );
                up_printf( "%x", ThisUsbDev.DeviceStatus );
                up_puts( " type=" );
                up_printf( "%x", ThisUsbDev.DeviceType );
                up_puts( " ep0=" );
                up_printf( "%x", ThisUsbDev.GpVar[ 0 ] );
                up_puts( " ep1=" );
                up_printf( "%x", ThisUsbDev.GpVar[ 1 ] );
                up_puts( " attach=" );
                up_printf( "%x", ( R8_USB_MIS_ST & RB_UMS_DEV_ATTACH ) ? 1 : 0 );
                up_puts( "\r\n" );
            }
            else if( c == 'd' )
            {
                up_puts( "drop=" );
                up_putdec( ev_drop );
                up_puts( "\r\n" );
            }
            else if( c == 'e' )
            {
                evq_head = evq_tail = evq_cnt = 0;
                ev_drop = 0;
                up_puts( "q clr\r\n" );
            }
        }
```

- [ ] **Step 6: 编译 + 烧录 + 串口验证**

Run:
```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -j0 -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt"; Get-Content "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" | Select-Object -Last 5
```
Expected: `0 Error(s)`（如有未使用变量警告，检查无逻辑问题即可）。

烧录后预期日志：
```
if0: 00000000 00000000 00000000
rep0: 05 01 09 06 A1 01 ...            ← 键盘接口报告描述符
if1: 00000000 00000000 00000000
rep1: 05 0C 09 01 A1 01 ...            ← 若开头 05 0C → 确认 Consumer 页
```
按按键：
```
KEY:  [0] DN "A" (mods=00)
KEY:  [0] UP "A" (mods=00)
```
连按多键应出现连续 DN/UP；队满丢事件时串口命令 `d` 显示 drop 计数，`e` 清零。

**取证点：** 记录 `rep1` 的完整字节，供 Task 5 校正 Consumer 解码（若 `rep1` 开头不是 `05 0C`，标记 `parse_consumer_report` 需按实测结构调整）。

- [ ] **Step 7: Commit**

```bash
git add user/main.c
git commit -m "feat(host): 按键事件环形队列 + 键盘/Consumer 报告解析 + 报告描述符取证"
```

---

### Task 4: 完整键位映射表

**Files:**
- Modify: `user/main.c`

- [ ] **Step 1: 用完整 usage 表替换 `scancode_to_str`**

删除现有 `scancode_to_str` 函数（main.c 第 70-111 行），替换为：

```c
/*******************************************************************************
* HID usage 码 → 名称 (覆盖 0x04-0x65 全键盘区 + 修饰键)
* 78 键布局对照实物校正 (Task 4 Step 3)
*******************************************************************************/
static const char *usage_name( UINT16 code )
{
    switch( code )
    {
        case 0x04: return "A"; case 0x05: return "B"; case 0x06: return "C";
        case 0x07: return "D"; case 0x08: return "E"; case 0x09: return "F";
        case 0x0A: return "G"; case 0x0B: return "H"; case 0x0C: return "I";
        case 0x0D: return "J"; case 0x0E: return "K"; case 0x0F: return "L";
        case 0x10: return "M"; case 0x11: return "N"; case 0x12: return "O";
        case 0x13: return "P"; case 0x14: return "Q"; case 0x15: return "R";
        case 0x16: return "S"; case 0x17: return "T"; case 0x18: return "U";
        case 0x19: return "V"; case 0x1A: return "W"; case 0x1B: return "X";
        case 0x1C: return "Y"; case 0x1D: return "Z";
        case 0x1E: return "1"; case 0x1F: return "2"; case 0x20: return "3";
        case 0x21: return "4"; case 0x22: return "5"; case 0x23: return "6";
        case 0x24: return "7"; case 0x25: return "8"; case 0x26: return "9";
        case 0x27: return "0";
        case 0x28: return "Enter"; case 0x29: return "Esc"; case 0x2A: return "Backspace";
        case 0x2B: return "Tab";   case 0x2C: return "Space";
        case 0x2D: return "-";     case 0x2E: return "=";  case 0x2F: return "[";
        case 0x30: return "]";     case 0x31: return "\\";
        case 0x33: return ";";     case 0x34: return "'";  case 0x35: return "`";
        case 0x36: return ",";     case 0x37: return ".";  case 0x38: return "/";
        case 0x39: return "CapsLock";
        case 0x3A: return "F1";  case 0x3B: return "F2";  case 0x3C: return "F3";
        case 0x3D: return "F4";  case 0x3E: return "F5";  case 0x3F: return "F6";
        case 0x40: return "F7";  case 0x41: return "F8";  case 0x42: return "F9";
        case 0x43: return "F10"; case 0x44: return "F11"; case 0x45: return "F12";
        case 0x46: return "PrintScreen"; case 0x47: return "ScrollLock";
        case 0x48: return "Pause";       case 0x49: return "Insert";
        case 0x4A: return "Home";        case 0x4B: return "PageUp";
        case 0x4C: return "Delete";      case 0x4D: return "End";
        case 0x4E: return "PageDown";
        case 0x4F: return "Right"; case 0x50: return "Left";
        case 0x51: return "Down";  case 0x52: return "Up";
        case 0x53: return "NumLock";
        case 0x54: return "/"; case 0x55: return "*"; case 0x56: return "-";
        case 0x57: return "+"; case 0x58: return "Enter";
        case 0x59: return "1"; case 0x5A: return "2"; case 0x5B: return "3";
        case 0x5C: return "4"; case 0x5D: return "5"; case 0x5E: return "6";
        case 0x5F: return "7"; case 0x60: return "8"; case 0x61: return "9";
        case 0x62: return "0"; case 0x63: return ".";
        case 0x64: return "AppMenu"; case 0x65: return "Power";
        case 0xE0: return "LCtrl"; case 0xE1: return "LShift";
        case 0xE2: return "LAlt";  case 0xE3: return "LGui";
        case 0xE4: return "RCtrl"; case 0xE5: return "RShift";
        case 0xE6: return "RAlt";  case 0xE7: return "RGui";
        default:   return "?";
    }
}
```

- [ ] **Step 2: 更新调用点**

`process_key_events()` 中把 `scancode_to_str( ( UINT8 )ev.usage )` 改为 `usage_name( ev.usage )`。用 grep 确认无残留 `scancode_to_str`：

```powershell
Get-ChildItem -Include *.c,*.h -Recurse | Select-String -Pattern "scancode_to_str" | ForEach-Object { $_.Path }
```
Expected: 无输出。

- [ ] **Step 3: 编译 + 烧录 + 全键位测试**

Run:
```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -j0 -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt"; Get-Content "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" | Select-Object -Last 5
```
Expected: `0 Error(s)`。

烧录后请用户逐键按下并记录串口输出，对照 78 键实物布局，把映射不对的 case 校正（如 78 键键盘若有 `Fn` 被独立上报、或 `-`/`=` 位置差异）。校正结果直接改 `usage_name` 的 case 值。

- [ ] **Step 4: Commit**

```bash
git add user/main.c
git commit -m "feat(host): 完整 HID usage 键位映射表"
```

---

### Task 5: 接口 1 多媒体键实测校正

**Files:**
- Modify: `user/main.c`（按实测调整 `consumer_usage_name` / `parse_consumer_report`）

- [ ] **Step 1: 分析 Task 3 抓到的 `rep1` 报告描述符**

根据 `rep1:` 日志：
- 若以 `05 0C` 开头 → Consumer 页确认，走 Step 2
- 若为其他 usage page（如 `06 00 FF` vendor 页）→ 走 Step 3
- 若接口 1 静默无报告（Fn 组合由键盘内部处理，不经过 USB）→ 走 Step 3

- [ ] **Step 2: Consumer 键实测**

请用户按 Fn+F1~F12 及键盘上标注的多媒体符号键，观察串口 `MEDIA:` 行。将实际 usage 与名字对照：

| 期望 usage | 名称 |
|---|---|
| 0x00E9 | Vol+ |
| 0x00EA | Vol- |
| 0x00E2 | Mute |
| 0x00CD | Play/Pause |
| 0x00B5 | Next |
| 0x00B6 | Prev |
| 0x00B7 | Stop |

若某键打印 `0xXXXX`（未识别）且实际是多媒体键 → 在 `consumer_usage_name` 中按实测值补 case；若打出的名字与键帽功能不符 → 改 case 返回值。若 `MEDIA:` 完全无输出（Fn 组合不产生 USB 报告）→ 记录结论"接口 1 静默"，`consumer_usage_name` 无需再改。

- [ ] **Step 3: 非 Consumer / 静默处理**

- vendor 页报告 → 保留 raw 打印现状（`MEDIA: 0xXXXX`），在代码注释记录结论
- 接口 1 静默 → 无代码改动，在 Task 6 验收记录中注明

- [ ] **Step 4: 编译 + 烧录复测 + Commit**

```powershell
& "D:\Keil_v5\UV4\UV4.exe" -b "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -j0 -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt"; Get-Content "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" | Select-Object -Last 5
```
Expected: `0 Error(s)`；复测通过后：

```bash
git add user/main.c
git commit -m "feat(host): 多媒体键解码按实测校正"
```

---

### Task 6: 总验收

**Files:** 无代码改动

- [ ] **Step 1: 按验收清单逐项确认**

| # | 验收项 | 判定 |
|---|---|---|
| 1 | 78 键键盘插上，`if0/if1` 全 0，心跳 `nak` 正常、`err≈0` | 串口日志 |
| 2 | 按键 → `KEY: [0] DN/UP` 事件输出 | 串口日志 |
| 3 | Fn 组合多媒体键 → `MEDIA:`（若设备实际发送） | 串口日志 |
| 4 | MK5 小键盘回归正常 | 串口日志 |
| 5 | 修改仅限 `user/main.c` + `CH57x_usbhostClass.c`，无新增文件 | git status |

- [ ] **Step 2: 更新设计文档实施结果**

在 `docs/superpowers/specs/2026-08-10-ch579m-78key-keyboard-design.md` 末尾追加"实施结果"小节：验证日志摘要、`rep1` 描述符结论、多媒体键实测结论、剩余问题（如有）。

- [ ] **Step 3: 最终 Commit**

```bash
git add -A
git commit -m "docs: 78 键键盘适配实施结果"
```

---

## Self-Review

**Spec coverage:**
- Phase 1 修复（诊断/验证/修库/回归/掉线后备）→ Task 1 + Task 2 ✓
- Phase 2 事件抽象层（结构/队列/解析/消费/轮询/命令）→ Task 3 ✓
- Phase 3 多媒体解码（取证/解码/实测校正）→ Task 3 Step 6 + Task 5 ✓
- Phase 4 映射表 → Task 4 ✓
- 验收 → Task 6 ✓
- "不新建 .c/.h" → 全部改动仅在现有 2 文件 ✓

**Placeholder scan:** 无 TBD/TODO；所有代码步骤均给出完整代码；验证命令含预期输出。

**Type consistency:** `key_event_t.usage` 为 UINT16，键盘事件 push 时传 UINT8 提升无冲突；`HID_GetReportDescr` 签名改动同步了唯一调用点（main 初始化块）；`scancode_to_str` → `usage_name` 改名同步了唯一调用点（Task 4 Step 2）；`evq_head/tail` 为 UINT8 与队列大小 16 匹配。

**后备分支确认：** Task 1 Step 4 定义了假设失败的停止点与后备路径（PRE PID / 强制全速 / 换线 / 供电），与 spec 风险节一致。
