# 同键重复按下不输出 — 超时推断释放 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复同键连按第二次不输出：键盘不上报松开信号，以"距上条报告 >50ms 视为全部松开"推断释放，让同键重按按新键处理。

**Architecture:** 仅改 `bsp/usb_host_hid.c`。在 `PollHIDEndpoints` 维护 `usb_poll_tick`（每主循环 +1，约 2.4ms/轮）；`parse_kbd_report` 与 `parse_consumer_report` 解析报告时若 `usb_poll_tick - last_tick > KBD_RELEASE_TIMEOUT_TICKS(20)`，先补发快照键的 RELEASE 并清空快照，再走原有 diff → 同键重按产生 PRESS。

**Tech Stack:** Keil C51/Cortex-M0（CH579M），CH57x USB Host 库，无单元测试框架（验证=Keil 命令行构建 + 烧录实机测试）。

**参考规格:** `docs/superpowers/specs/2026-08-15-same-key-repress-design.md`

---

### Task 1: 新增 tick 计数与超时宏

**Files:**
- Modify: `bsp/usb_host_hid.c:55-58`（轮询统计区，插入 tick 变量与宏）
- Modify: `bsp/usb_host_hid.c:284`（`diag_poll++` 旁加 `usb_poll_tick++`）

- [ ] **Step 1: 在轮询统计区插入 tick 变量与宏**

在 line 58（`static UINT32 diag_err   = 0;   /* 其它错误 */`）之后新增：

```c
static UINT16 usb_poll_tick = 0;                  /* 主循环轮数, 约2.4ms/轮 */

/* 键盘不上报松开信号: 距上条报告超过该阈值即认为此前所有键已松开(≈50ms) */
#define KBD_RELEASE_TIMEOUT_TICKS   20
```

- [ ] **Step 2: 在 PollHIDEndpoints 递增 tick**

`PollHIDEndpoints` 内 `diag_poll ++;`（line 284）之后新增一行：

```c
        diag_poll ++;
        usb_poll_tick ++;
```

- [ ] **Step 3: 构建验证（未用变量暂以宏为"已用"，先构建确认无语法错）**

Run:
```powershell
& "D:\Keil_v5\UV4\UV4.exe" -r "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" -j0
```
Expected: `build_log.txt` 中 `0 Error(s)`（此时 tick 已被 PollHIDEndpoints 使用，宏在 Task 2 使用）。

---

### Task 2: parse_kbd_report 加超时推断释放

**Files:**
- Modify: `bsp/usb_host_hid.c:200-204`

- [ ] **Step 1: 加静态 last_tick 与超时块**

`static UINT8 last_mods = 0;`（line 201）后新增 `static UINT16 last_tick = 0;`；
`UINT8  i, j, mods, found;`（line 202）与 `if( len < 3 ) return;`（line 204）之间插入：

```c
    /* 键盘不上报松开信号: 距上条报告超阈值即认为此前所有键已松开,
       补发RELEASE并清空快照, 使同键重按被视为新按下 */
    if( ( UINT16 )( usb_poll_tick - last_tick ) > KBD_RELEASE_TIMEOUT_TICKS )
    {
        for( i = 0; i < 6; i ++ )
            if( last_keys[ i ] )
                kbd_ev_push( KEV_RELEASE, last_mods, 0, last_keys[ i ] );
        for( i = 0; i < 6; i ++ ) last_keys[ i ] = 0;
    }
    last_tick = usb_poll_tick;
```

- [ ] **Step 2: 构建验证**

Run:
```powershell
& "D:\Keil_v5\UV4\UV4.exe" -r "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" -j0
```
Expected: `0 Error(s)`。

- [ ] **Step 3: 提交**

```bash
git add bsp/usb_host_hid.c
git commit -m "fix: 同键重复按下不输出——主键盘超时推断释放(~50ms无报告视为松开)"
```

---

### Task 3: parse_consumer_report 加超时推断释放

**Files:**
- Modify: `bsp/usb_host_hid.c:240-245`

- [ ] **Step 1: 加静态 last_tick 与超时块**

`static UINT8  last_n = 0;`（line 241）后新增 `static UINT16 last_tick = 0;`；
`UINT8  n = 0, i, j, found;`（line 243）与 `if( len < 3 ) return;`（line 245）之间插入：

```c
    /* 同主键盘: 超时推断释放 */
    if( ( UINT16 )( usb_poll_tick - last_tick ) > KBD_RELEASE_TIMEOUT_TICKS )
    {
        for( i = 0; i < last_n; i ++ )
            kbd_ev_push( KEV_RELEASE, 0, 1, last_cu[ i ] );
        last_n = 0;
    }
    last_tick = usb_poll_tick;
```

- [ ] **Step 2: 构建验证**

Run:
```powershell
& "D:\Keil_v5\UV4\UV4.exe" -r "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\CH579_USB_Host_HID.uvprojx" -o "E:\MeWork\CH579M\CH579_USB_Host_HID_pro\project\build_log.txt" -j0
```
Expected: `0 Error(s)`，无未使用告警（`last_cu` 仍被 diff 逻辑使用）。

- [ ] **Step 3: 提交**

```bash
git add bsp/usb_host_hid.c
git commit -m "fix: 媒体键同键重复按下不输出——consumer超时推断释放"
```

---

### Task 4: 实机验证（需要烧录）

- [ ] **Step 1: 烧录** `project/Objects/CH579_USB_Host_HID.hex`
- [ ] **Step 2: 功能测试**（UART1 115200 观察 `KEY:` / `MEDIA:` 日志）
  - `s`、`s` 连按（间隔 >0.1s）→ 各输出 1 个，日志形如 `1dn 1up "s"` `1dn 1up "s"`
  - `s`、`d`、`s` → 各 1 个；快速连打 `asdf` → 各 1 个（含批量合并行 `4dn 4up "asdf"`）
  - 长按 `s`（>1s）→ 仅 1 个（连发已禁用）
  - 媒体键连按同键 → 均输出
  - 边界抽查：按住 `s` >50ms 再按 `d` → 多发 1 个 `s`（已知、可接受）
- [ ] **Step 3: 回归** Modbus 读 40001 仍返回键入内容；看门狗心跳正常

---

### Task 5: 收尾提交（可选，若 Task 2/3 已提交则跳过）

- [ ] **Step 1: 确认 git 工作区干净**（`.embeddedskills/config.json` 属本地配置，永不提交）

```bash
git status --short
git log --oneline -5
```

- [ ] **Step 2: 如需推送**

```bash
git push
```
