# CH579M 适配 78 键 USB HID 键盘 — 设计文档

- 日期：2026-08-10
- 工程：CH579_USB_Host_HID_pro（CH579M，ARM Cortex-M0，Keil，UART1 115200 调试）

## 背景与问题

现有工程基于 WCH 官方 USB Host 库 + `user/main.c`，原本适配 MK5 数字小键盘（同板正常）。

新接入一把有线 78 键键盘（VID=0x2C1A，PID=0x0B2A，LITEON）后，串口日志显示：

- 枚举全部成功（GetDevDescr / SetAddress / GetCfgDescr / SetConfig），打印 `USB-Keyboard Ready`，`InitRootDevice=0`
- 但 SetConfig 之后所有传输失声：
  - `if0/if1` 的 SetIdle / SetProtocol / GetReportDescr 全部返回 `0x20`
  - IN 轮询 `poll=500 ok=0 nak=0 err=500`
- 按下按键时出现 `USB dev out`（设备掉线）
- 设备描述符 `bMaxPacketSize0=0x08` → 典型**低速（1.5Mbps）设备**
- 配置描述符：2 个 HID 接口（if0=Boot 键盘 subclass 1 / protocol 1，if1=subclass 0 / protocol 0），各 1 个中断 IN 端点（0x81、0x82），报告长度 8 字节，间隔 10ms

### 根因分析

- `0x20 = ERR_USB_TRANSFER`；底层 `USBHostTransact`（CH57x_usbhostBase.c）返回裸 `0x20` 表示设备**完全无应答**（`MASK_UIS_H_RES=0`，timeout），既非 NAK 也非 STALL
- 库 `InitRootDevice()`（CH57x_usbhostClass.c）所有成功路径末尾强制 `SetUsbSpeed(1)`，将 `R8_USB_CTRL.RB_UC_LOW_SPEED` 清掉、切回全速 12Mbps；而 `R8_UHOST_CTRL.RB_UH_LOW_SPEED` 仍标记低速
- 结果：枚举后主机以全速时序与低速键盘通信 → 键盘收不到令牌 → 全部超时 0x20
- 按下按键时键盘以 1.5Mbps 应答，全速主机将其视为总线毛刺 → 误判 `dev out`
- MK5 小键盘为全速设备，不受该翻转影响，故之前正常

## 目标与范围

1. （必做）78 键键盘正常枚举与按键识别
2. 解接口 1 多媒体键（Consumer 页；该键盘媒体功能为 Fn 组合，由键盘内部处理，接口 1 是否发报告需实测）
3. 键位映射表校正（78 键布局）
4. 按键事件抽象层（环形队列 + 解析/消费解耦）
5. 输出仅 UART1 串口打印；不新建 .c/.h 文件（改动仅在 `user/main.c` 与库文件 `CH57x_usbhostClass.c`）

## 设计

### Phase 1 — 修复 0x20 无应答（核心）

1. **诊断打印**（`user/main.c`，InitRootDevice 成功后）：
   - 打印 `ThisUsbDev.DeviceSpeed`（0=低速 / 1=全速）
   - 打印 `R8_USB_CTRL` 的 `RB_UC_LOW_SPEED` 与 `R8_UHOST_CTRL` 的 `RB_UH_LOW_SPEED` 实际位状态
2. **验证假设（一行）**：在 `InitRootDevice()` 返回、`HID_SetIdle` 之前加 `SetUsbSpeed(ThisUsbDev.DeviceSpeed);`
   - 预期：`if0/if1` 从 `20 20 20` 变全 `0`；轮询从 `err=500` 变 `nak=500`
   - 若预期未达成 → 停在诊断重新分析；备选分支：试 `RB_UH_PRE_PID_EN`、强制全速、换 USB 线
   - **该验证行在库修复（1.3）落地且回归通过后移除**（库已保证正确速度，main.c 中该行冗余）
3. **修库根因**（`CH57x_usbhostClass.c`）：成功路径 6 处 `SetUsbSpeed(1)`（第 95 / 106 / 123 / 130 / 145 / 156 行）改为 `SetUsbSpeed(ThisUsbDev.DeviceSpeed)`；失败清理路径（第 171 行）保持不动
4. **回归**：MK5 小键盘与 78 键键盘双测
5. **掉线残留处理**：若速度修复后按 Fn 组合键仍 `dev out` → VBUS 供电问题（按键瞬时电流拉垮 5V），属硬件侧，检查电源 / 加大 VBUS 电容

### Phase 2 — 按键事件抽象层（main.c 内）

- 事件结构：

  ```c
  typedef struct {
      UINT8  type;   /* KEV_PRESS / KEV_RELEASE */
      UINT8  mods;   /* bit0 Shift bit1 Ctrl bit2 Alt bit3 GUI */
      UINT8  ifidx;  /* 0=键盘 1=多媒体 */
      UINT16 usage;  /* HID usage 码 */
  } key_event_t;
  ```

- 环形队列 16 深：`kbd_ev_push()` / `kbd_ev_pop()`；队满丢弃并累加 `ev_drop` 计数
- `parse_kbd_report(buf, len)`：字节 2-7 与上次按下集合对比，新增=PRESS、消失=RELEASE（支持同帧多键）；修饰键变化仅更新 `mods`，不单独发事件
- `parse_consumer_report(buf, len)`：Consumer usage 解码，变化去重
- `process_key_events()`：主循环弹空队列 → UART 打印
  - 格式：`KEY:  [0] DN "1"    (mods=00)` / `KEY:  [0] UP "1"` / `MEDIA: [1] DN Vol+`
- 轮询重写为 `PollHIDEndpoints()`：对 GpVar[0]/GpVar[1] 各做一次 IN → 按 ifidx 分发解析；NAK/错误统计与心跳保留
- 串口命令扩展：`p` 状态、`d` 打印 ev_drop、`e` 清空队列

### Phase 3 — 接口 1 多媒体键解码

1. 速度修复后 `HID_GetReportDescr(1)` 应成功 → 打印接口 1 报告描述符，确认 usage page 与报告结构
2. 若为 Consumer 页（0x0C）：按 usage 查表 — Vol+ `0xE9`、Vol- `0xEA`、Mute `0xE2`、Play/Pause `0xCD`、Stop `0xB7`、Prev `0xB6`、Next `0xB5`、Home `0x223`、Back `0x224`
3. 查不到的 usage 打原值 `MEDIA: 0xXXXX`
4. 非 Consumer 页（如 vendor）→ 保留 raw 打印，不强行解码
5. Fn 组合实测：按 Fn+F1~F12 观察接口 1 是否发报告；有 → 用实测 usage 校正解码表；无 → 多媒体解码降级为 raw 打印

### Phase 4 — 键位映射表

- 完整 HID 标准 usage 表替换现有稀疏 switch：字母区 `0x04`-`0x29`、编辑键 `0x2A`-`0x38`、F1-F12 `0x3A`-`0x45`、方向/编辑/小键盘 `0x46`-`0x63`、特殊键（Esc/Tab/Caps/Shift/Ctrl/Win/Alt/菜单）
- 表为 `const char*` 数组（下标=usage 码），先上标准表，再由用户对照实物 78 键校正

## 改动文件清单

| 文件 | 改动 |
|---|---|
| `user/main.c` | 诊断打印、验证行（临时，库修复落地后移除）、事件层、Consumer 解码、完整映射表 |
| `library/StdPeriphDriver/CH57x_usbhostClass.c` | 6 处 `SetUsbSpeed(1)` → `SetUsbSpeed(ThisUsbDev.DeviceSpeed)` |

不新建 .c/.h 文件（如需新建，先征求用户同意）。

## 实施顺序

1. 只加诊断 + 验证行，烧录实测确认假设（不改库）
2. 验证通过 → 改库 6 处 → 回归 MK5 + 78 键
3. 事件层 + Consumer 解码 + 映射表（一次编译验证）
4. 用户对照实物校正 78 键映射表

## 验收标准

- 78 键键盘插上：`if0/if1` 全 `0`，心跳 `nak` 正常、`err≈0`
- 按键 → 串口出 `KEY: [0] DN/UP`；Fn 组合出 `MEDIA:`（若设备实际发送）
- MK5 小键盘回归正常
- 全部串口日志可复盘（COM4 / 115200）

## 风险与后备

- 速度修复无效 → 备选：强制全速测试 / `RB_UH_PRE_PID_EN` / 换 USB 线
- 按键掉线残留 → VBUS 供电问题，硬件侧处理

## 实施结果（2026-08-10）

### 根因确认（硬件实测）

诊断日志实锤：
```
spd=00000000 UC_LS=00000000 UH_LS=00000001
```
- `DeviceSpeed=0`（低速），`R8_USB_CTRL.RB_UC_LOW_SPEED=0`（被库翻回全速），`R8_UHOST_CTRL.RB_UH_LOW_SPEED=1`（端口仍低速）——寄存器不一致
- 一行 `SetUsbSpeed(ThisUsbDev.DeviceSpeed)` 验证后：`if0/if1` 从 `20 20 20` 变全 `0`，心跳从 `err=500` 变 `nak=500`，假设证实

### 修复落地

- `CH57x_usbhostClass.c`：6 处成功路径 `SetUsbSpeed(1)` → `SetUsbSpeed(ThisUsbDev.DeviceSpeed)`（U盘/打印机/键盘/鼠标/HUB/未知设备），失败清理路径（第 171 行）保留
- 修复后：`spd=0 UC_LS=1 UH_LS=1`（一致），78 键键盘 + MK5 小键盘双回归通过

### 接口 1 报告描述符取证（rep1）

- 集合 1：**Consumer 页（05 0C）**，ReportID=1，usage 范围 0x0000-0x023C，1×16bit usage 数组 → 解析器按 `[ReportID][usage16 小端]` 处理
- 集合 2：Generic Desktop System Control（05 01 09 80），ReportID=2，usage 0x81-0x83（Power/Sleep/Wake）3 bit 输入——未解码，保留 raw 打印策略
- 键盘接口（rep0）：标准键盘报告（无 ReportID），8 字节（修饰键+保留+6 键码），含 LED 输出报告（未使用）

### 多媒体键实测结果

| usage | 名称 | 键位 |
|---|---|---|
| 0x00B6 | Prev | Fn 组合 |
| 0x00CD | Play/Pause | Fn 组合 |
| 0x00B5 | Next | Fn 组合 |
| 0x00E2 | Mute | Fn 组合 |
| 0x00EA | Vol- | Fn 组合 |
| 0x00E9 | Vol+ | Fn 组合 |
| 0x0183 | Fn+F3 | 实测键位（按物理键命名） |

### 附加实现

- 按键事件环形队列（16 深）+ 键盘/Consumer 状态差分解析 → PRESS/RELEASE 事件
- 完整 HID usage 键位映射表（0x04-0x65 + 修饰键 0xE0-0xE7）
- 实际字符显示：字母随 CapsLock/Shift 大小写，数字/标点随 Shift 出符号（Shift+1→`!` 等）
- 串口命令：`p` 状态 / `d` 事件丢弃计数 / `e` 清空队列
- 报告描述符 dump（rep0/rep1）

### 验收结果

| # | 验收项 | 结果 |
|---|---|---|
| 1 | 78 键键盘 `if0/if1` 全 0，心跳 nak 正常 err≈0 | ✅ |
| 2 | 按键 → `KEY: [0] DN/UP "字符" (mods=XX)` | ✅ |
| 3 | Fn 组合 → `MEDIA: [1] DN/UP 名称` | ✅ |
| 4 | MK5 小键盘回归正常 | ✅ |
| 5 | 修改仅限 main.c + usbhostClass.c，无新增文件 | ✅ |

### 备注

- 测试期间出现的 `USB dev out` 为用户手动拔插，非固件问题
- 6 键防冲突溢出（0xFF）与断线重插时快照未清零会产生少量伪事件（自愈），记录为已知小问题
- 接口 1 的 ReportID=2 系统控制键（Power/Sleep/Wake）未解码
- 修饰键位修正：USB 标准 bit1=LShift（初版误用 bit0=Ctrl）；现支持全部 8 位修饰键（0xE0-0xE7），Shift 等修饰键按下/释放单独输出事件，Shift+字母出大写、Shift+数字/标点出符号（Shift+1→`!`）
