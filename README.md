# CH579_USB_Host_HID_pro

基于 CH579M（ARM Cortex-M0）的 USB Host 工程，将 USB 键盘 / HID 设备转换为可在 UART 上打印的按键事件。

本工程以 WCH 官方 USB Host 例程为基础，仅改动两个文件：`user/main.c` 与 `library/StdPeriphDriver/CH57x_usbhostClass.c`。

## 功能特性

- **多设备兼容**：支持低速（1.5Mbps）与全速（12Mbps）USB HID 键盘
- **键盘事件层**：按键按下 / 释放事件（环形队列，16 深度，溢出计数），支持同帧多键及修饰键变化
- **完整键位映射**：78 键常规键盘位映射，字母随 CapsLock/Shift 大小写，符号键随 Shift 移位
- **多媒体键（Consumer 页）**：解析接口 1 的 Consumer 报告（含 ReportID 0x01 前缀），支持 Fn 组合键
  - Fn+Vol+ / Fn+Vol- / Fn+Mute / Fn+播放暂停 / Fn+上一首 / Fn+下一首、F1~F12 等实测键位
  - 未识别 usage 原值打印（`MEDIA: 0xXXXX`）
- **串口调试命令**：`p` 状态打印 / `d` 打印丢弃事件数 / `e` 清空事件队列
- **诊断输出**：枚举速度、USB 寄存器状态、IN 轮询 NAK/错误统计

## 硬件接线

- MCU：CH579M，USB Host 口接 USB 键盘，UART1 115200 8N1 接调试串口
- 输出仅通过 UART1 打印，无屏幕 / 无额外外设依赖

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