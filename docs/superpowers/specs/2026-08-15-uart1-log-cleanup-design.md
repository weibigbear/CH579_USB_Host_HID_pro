# 2026-08-15 UART1 日志裁剪为仅按键报文

## 目标

UART1（115200 调试口）输出精简为**仅按键报文**（`KEY: ...` / `MEDIA: ...` 行）。
其余所有日志删除：开机横幅、复位原因、Modbus 配置、`p`/`d`/`e` 串口命令应答、
每秒心跳 `H:` 行、USB 枚举过程日志、设备断开日志、报告转储。

**其余功能一律不动**：Modbus RTU 从机（UART3）、USB HID 键盘/多媒体解析、
repeat 连发、看门狗、DataFlash 配置加载全部保持现状。

## 方案

**方案 A（选定）：纯裁剪**。只删除打印语句，不改任何逻辑、不拆批量合并行。

## 保留

- `main.c` 按键报文输出：单事件 `KEY:  [0] DN "A" (mods=00)` / 批量合并行
  `KEY: 6dn 6up "abcdef"` / `MEDIA: [1] DN Vol+`
- 错误级告警（极罕见、出现时是关键诊断，不污染输出）：
  - `usb_host_hid.c` `enum fail, sys reset`（枚举失败软件复位告警）
  - `modbus_rtu.c` 三处 `cfg save fail`（配置保存失败告警）
- 全部功能调用（含被删日志旁的逻辑，如 `modbus_diag_set_reset_cause()`、
  `HID_SetIdle`、`HID_GetReportDescr` 等）

## 裁剪点

### `user/main.c`（4 处，约 55 行）

| 位置 | 内容 | 处理 |
|----|----|----|
| 复位原因打印（`WDOG reset` / `reset: normal`） | 删除打印，保留 `modbus_diag_set_reset_cause()` 调用（0x0082 仍写） |
| `\r\nMK5 USB-HID Host start\r\n` 横幅 | 删除 |
| `mb: addr=... baud=...` 配置打印 | 删除 |
| `p`/`d`/`e` 串口命令块 | 整块删除 |
| 心跳 `H: poll=... ok=... nak=... err=...` 块 | 整块删除 |

### `bsp/usb_host_hid.c`（3 处，约 45 行）

| 位置 | 内容 | 处理 |
|----|----|----|
| `dump_bytes()` 静态函数 | 删除函数及两处调用（`rep0:`/`rep1:` 转储） |
| `dev out` 断开日志 | 删除 |
| 枚举过程日志（`dev in, enum...`、`InitRootDevice=`、`VID/PID`、`ep0/ep1`、`spd/UC_LS/UH_LS`、`if0:`、`rep0/rep1:`） | 删除打印，保留全部调用语句 |

## 不做

- 不拆批量合并行（快速打字仍显示 `KEY: 6dn 6up "abcdef"`）
- 不改 Modbus 寄存器映射、功能码、波特率
- 不动 `.embeddedskills/config.json`（本地串口配置）
- 文档暂不改（README 概括性描述仍准确）

## 验证

1. Keil 命令行构建（`UV4.exe -r ... -o build_log.txt -j0`）→ `0 Error(s)`
2. 烧录后串口输出仅 `KEY:` / `MEDIA:` 行；无 `MK5 start` / `H:` / `mb:` / `reset:`
3. 功能回归：输入 `ABC` 按键报文正常；Modbus 读 40001 仍得 ABC；
   枚举失败告警、配置保存失败告警仍能打印
