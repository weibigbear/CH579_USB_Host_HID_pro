# CH579_USB_Host_HID_pro

基于 CH579M（ARM Cortex-M0）的 USB Host 工程，将 USB 键盘 / HID 设备转换为可在 UART 上打印的按键事件。

本工程以 WCH 官方 USB Host 例程为基础，仅改动两个文件：`user/main.c` 与 `library/StdPeriphDriver/CH57x_usbhostClass.c`。

## 功能特性

- **多设备兼容**：支持低速（1.5Mbps）与全速（12Mbps）USB HID 键盘
- **键盘事件层**：按键按下 / 释放事件（环形队列，32 深度，溢出计数），支持同帧多键及修饰键变化
- **按住自动连发**：按住可打印字符键（字母/数字/符号/空格）≈500ms 后按 ~50ms 间隔持续重复，模拟 PC 键盘连发（`KBD_REPEAT_DELAY_CNT` / `KBD_REPEAT_INTERVAL_CNT` 可调）；Enter/Backspace/CapsLock/修饰键刻意不重复（Enter 重复会导致帧反复提交清空，Backspace 重复会误删整帧）
- **完整键位映射**：78 键常规键盘位映射，字母随 CapsLock/Shift 大小写，符号键随 Shift 移位
- **多媒体键（Consumer 页）**：解析接口 1 的 Consumer 报告（含 ReportID 0x01 前缀），支持 Fn 组合键
  - Fn+Vol+ / Fn+Vol- / Fn+Mute / Fn+播放暂停 / Fn+上一首 / Fn+下一首、F1~F12 等实测键位
  - 未识别 usage 原值打印（`MEDIA: 0xXXXX`）
- **串口调试命令**：`p` 状态打印 / `d` 打印丢弃事件数 / `e` 清空事件队列
- **诊断输出**：枚举速度、USB 寄存器状态、IN 轮询 NAK/错误统计
- **Modbus RTU 从机**：键盘输入字符按顺序写入保持寄存器 40001~40128（低字节 ASCII、高字节 0），9600 8N1，仅响应功能码 0x03

## 硬件接线

- MCU：CH579M，USB Host 口接 USB 键盘，UART1 115200 8N1 接调试串口
- 输出仅通过 UART1 打印，无屏幕 / 无额外外设依赖
- Modbus：UART3（TXD3=PA5 推挽、RXD3=PA4 上拉输入），PA6=RE/DE 方向控制（推挽，经 1k 电阻，RS485 收发器 5V 供电）
- **无 RS485 收发器也能测试**：Modbus RTU 本质是普通 UART 协议，直接用 USB-TTL 模块连 PA4/PA5 即可，PA6 悬空不影响（见下文「TTL 直连测试」）

## 目录结构

```
├─ user/                        # 应用主逻辑（main.c，全部功能在此）
├─ library/                     # 厂商库
│  ├─ StdPeriphDriver/          # 标准外设库（含 USB Host，已修改 CH57x_usbhostClass.c）
│  ├─ CMSIS/                    # Cortex-M0 内核头文件
│  ├─ Startup/                  # 启动文件
│  └─ sct/                      # 分散加载文件
├─ project/                     # Keil 工程（uvprojx / uvoptx）
├─ docs/                        # 设计与实现文档
└─ README.md
```

## Modbus RTU 从机（键盘 → 40001~40128 保持寄存器）

键盘输入的字符按顺序写入保持寄存器 `40001~40128`（Modbus 内部地址 `0x0000~0x007F`）。每个寄存器 16 位：**低字节 = ASCII 码，高字节 = 0**；无字符的寄存器值为 0。

**Modbus 参数**：从机地址 1、波特率 9600、8 数据位 / 无校验 / 1 停止位（8N1），仅支持功能码 `0x03` 读保持寄存器，单次读取数量 ≤125（超出需分段）。

**帧写入规则：**

| 键 | 行为 |
| ---- | ---- |
| 可打印字符（0x20~0x7E，含空格） | 追加到寄存器组当前位 |
| Enter（主键盘 0x28 / 小键盘 0x58） | 立即提交帧（余段清零，写入位复位） |
| Backspace | 回退一位并清零该寄存器 |
| 暂停输入 500ms | 空闲超时自动提交（`ASCII_IDLE_MS` 可调） |
| 其他键（F1、Esc、方向键等） | 忽略，不写入 |

寄存器答应对齐：读 `40001` 起始的 `N` 个寄存器，返回顺序即输入顺序。

### 错误处理

- 非 0x03 功能码 → 异常码 `0x01`（非法功能）
- 数量为 0、>125 或地址越界 → 异常码 `0x02`（非法数据地址）
- 从机地址不匹配 / CRC 错误 → 不应答

### TTL 直连测试（无需 RS485）

Modbus RTU 是标准 UART 协议，**没有 RS485 收发器也能完整验证**。直接通过 USB-TTL 模块（如 CH340 / CP2102 / FT232）连接，PA6 方向控制引脚悬空即可（TTL 全双工，无需方向切换）。

**接线：**

| CH579M | USB-TTL 模块 |
| ---- | ---- |
| PA5 (TXD3) | RX |
| PA4 (RXD3) | TX |
| GND | GND |
| PA6 | 悬空不接 |

注意：CH579M 为 3.3V 系统，USB-TTL 模块请选 **3.3V 电平**（5V 模块可能损坏 PA4/PA5）。

**测试方法一：Modbus Poll（推荐）**

1. Modbus Poll 免费版功能受限，仅能读，本功能正好只需要读
2. `Connection Setup` 选 **Modbus ASCII/TCP 之外**的串口类型（Modbus Poll 界面：`Connection → Connect`，Mode 选 **RTU**、串口选 USB-TTL 的 COM 口、9600、8N1）
3. `Setup → Read/Write Definition`：Function=**03 (Read Holding Registers)**，Slave ID=**1**，Address=**0**（对应 40001），Quantity=**5**
4. 按 `F8`（或菜单 Read Once）反复读取观察寄存器值

**测试方法二：串口助手手动发包**

帧格式：`从机地址(01) + 功能码(03) + 起始地址高/低 + 数量高/低 + CRC16低位 + CRC16高位`。

读 40001 起 5 个寄存器（预算好 CRC 的完整帧）：

```
01 03 00 00 00 05 85 C9
```

明文：`01` 地址、`03` 功能码、`00 00` 起始地址（40001）、`00 05` 数量 5、`85 C9` = CRC16（=0xC985，低字节在前）。每次发送间停顿 >3.5 字符时间（9600 下约 4ms），串口助手建议开启 **HEX 发送**、间隔 ≥100ms。收到应答第一个字节应为 `01`（地址），第 2 字节 `03`，第 3 字节为 `0A`（5 寄存器 × 2 字节）。

**测试方法三：pymodbus（Python 脚本）**

```python
from pymodbus.client import ModbusSerialClient

c = ModbusSerialClient('rtu', port='COM5', baudrate=9600,
                       bytesize=8, parity='N', stopbits=1, timeout=1)
c.connect()
r = c.read_holding_registers(0, 5, slave=1)   # 40001 起 5 个
print([hex(x) for x in r.registers])          # 低字节为 ASCII
c.close()
```

无 pymodbus 时可用 `pyserial` 手算 CRC：

```python
import serial

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

s = serial.Serial('COM5', 9600, timeout=1)
req = bytes.fromhex('01 03 00 00 00 05')          # 读 40001 起 5 个
crc = crc16(req)
s.write(req + bytes([crc & 0xFF, crc >> 8]))     # CRC 低字节在前
resp = s.read(50)
print(resp.hex(' '))                             # 期望: 01 03 0A 00 41 ...
```

**验证步骤：**

1. 上电烧录，插上键盘，UART1（调试口）可看到 `KEY: DN "A"` 等事件
2. 输入 `ABC` → 读 40001~40003 = `0x0041 / 0x0042 / 0x0043`，40004 起为 0
3. 按 Enter → 帧立即提交，40001 重新从 0 开始
4. 输入字符后停止 500ms → 空闲超时自动提交
5. 按 Backspace → 上一个字符变为 0
6. 分段读取验证：读 40001~40125（125 个）与 40126~40128（3 个），分别发包
7. 错误注入：错 CRC/错地址 → 无响应；数量=0 或 >125 → 异常码 0x02；非 0x03 功能码 → 异常码 0x01

测试通过后接 RS485 只需把 PA4/PA5/PA6 连到收发器（PA6 经 1k 电阻接 RE/DE，收发器 5V 供电），**代码无需改动**。

完整协议说明（寄存器映射/功能码/异常码/帧示例）见 [`docs/modbus.md`](docs/modbus.md)。

完整验收流程（上电自检/看门狗/大帧非阻塞/状态寄存器/错误注入/波特率回归/老化）见 [`docs/test-manual.md`](docs/test-manual.md)。

## 构建

使用 Keil MDK（UV4）打开 `project/CH579_USB_Host_HID.uvprojx` 编译。

命令行构建示例：

```
UV4.exe -b project\CH579_USB_Host_HID.uvprojx -o project\build_log.txt -j0
```

要求 `0 Error(s)`。

## 串口输出示例

```
KEY:  [0] DN "A"    (mods=00)
KEY:  [0] UP "A"
MEDIA: [1] DN Vol+
MEDIA: [1] UP Vol+
KEY:  [0] DN "1"    (mods=21)     // Shift+1 → !
```

## 关键技术点

### 键盘报告不丢失（UART1 非阻塞打印 + USB 轮询轻量化）

**历史缺陷**：UART1 日志用 THR 轮询忙等打印（每行 ~4ms），快速敲键时主循环被日志拖到 50ms+，USB 键盘轮询间隔超过键盘上报周期（8~10ms）导致报告丢失，表现为主键盘偶发丢键、**回车键失效**。

**修复（A+B+C 三管齐下）**：
- A：`uart_debug.c` 打印改中断驱动——putc1 快路径直接写发送 FIFO（首字符立即上线，不依赖中断触发；CH579 的 THR_EMPTY 中断依赖"写 THR→变空"边沿，若只入缓冲等中断，开中断时 THR 已空则永不触发），FIFO 满时入环形缓冲（512B）由 UART1 TX 中断续填，主循环 `uart_debug_poll()` 兜底重武装中断。
- B：`usb_host_hid.c` 轮询 `USBHostTransact(..., 50)` → `(..., 0)`——键盘 NAK 立即返回，不再忙等 1ms/端点。
- C：`main.c` 按键日志按批合并——快速打字时一轮最多 12 个事件，逐行打印突发 ~350B 会灌满 512B 缓冲；合并为一条汇总行（`KEY: 6dn 6up "abcdef"`）后一轮最多 ~35B，任何打字速度不溢出。单事件仍输出原详细格式（含 mods）。

### 低速设备无应答修复（0x20）

78 键键盘为**低速（1.5Mbps）**设备，初始示例在枚举成功后强制 `SetUsbSpeed(1)`（全速），导致低速设备响应全程超时，下层 `USBHostTransact` 返回 `0x20`（`ERR_USB_TRANSFER`）。

修复：`CH57x_usbhostClass.c` 中 6 处成功路径的 `SetUsbSpeed(1)` 改为 `SetUsbSpeed(ThisUsbDev.DeviceSpeed)`，按实际检测到的设备速度配置 USB 物理层（低速 / 全速链路）。回归验证：全速 MK5 键盘不受影响。

详细设计文档见 `docs/superpowers/specs/2026-08-10-ch579m-78key-keyboard-design.md`。

## 串口命令

| 命令 | 功能 |
| ---- | ---- |
| `p` | 打印状态（枚举速度、轮询统计等） |
| `d` | 打印环形队列丢弃的事件计数 |
| `e` | 清空事件队列 |