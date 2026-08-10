# CH579M USB 键盘 → Modbus RTU 从机（40001~40128 存一帧 ASCII）设计文档

- 日期：2026-08-10
- 工程：CH579_USB_Host_HID_pro（CH579M / ARM Cortex-M0 / Keil MDK / UART1 115200 调试）

## 目标

USB 键盘输入的字符按顺序存入 Modbus 保持寄存器 40001~40128（共 128 个），一个寄存器存一个 ASCII（低字节），无字符的寄存器值为 0。外部 PLC/HMI 通过 Modbus RTU 读走。

## 硬件

- RS485 收发器 5V 供电
- TXD3 = PA5（UART3 默认引脚，推挽输出）
- RXD3 = PA4（UART3 默认引脚，上拉输入）
- PA6 = 收发器 RE/DE 控制（经 1k 电阻）：发送前拉高、发送完毕拉低
- UART3 波特率 9600，8N1
- 5V 供电下 3.3V 高电平驱动 RE/DE 门限：先按直驱设计，若不可靠再补电平转换

## 模块划分

```
bsp/
  uart_debug.c/.h      （已有）UART1 打印
  usb_host_hid.c/.h    （已有）USB 枚举/解析/事件队列
  modbus_rtu.c/.h      （新增）Modbus RTU 从机协议层（UART3 + RS485 方向控制）
user/
  keymap.c/.h          （已有）键位映射
  ascii_frame.c/.h     （新增）ASCII 帧缓冲（寄存器组映射）
  main.c               （已有）粘合：消费事件 → 写帧缓冲 + 每轮调 modbus_rtu_poll
```

依赖方向单向：`main → ascii_frame / modbus_rtu / usb_host_hid / uart_debug`，不产生反向依赖。

## modbus_rtu.c 职责

参考 `E:\MeWork\CH579M\modbus.c`（STM32F0 从机协议层）的结构：

- `CRC16(dataIn, length)`：位运算算法（Poly 0xA001），与参考一致
- `modbus_cmd03_ack(pRecBuf, pAckBuf)`：0x03 读保持寄存器应答；寄存器映射集中在此（Modbus 地址 0~127 → reg_ascii[0..127]）
- `modbus_frame_process(pRec, len, pAck)`：地址匹配 + CRC 校验 + 功能码分发（仅 0x03），纯解析不碰硬件，返回应答长度；失败返回 0
- UART3 初始化：`UART3_DefInit()` 后 `UART3_BaudRateCfg(9600)`；GPIO：PA5 推挽 TX、PA4 上拉 RX、PA6 推挽 DE
- 接收状态机：逐字节轮询 `R8_UART3_LSR`，3.5 字符空闲判帧边界
- 发送：PA6 拉高 → 等 1~2 字符时间 → THR 逐字节 → 等最后字节移位完 → PA6 拉低
- 对外 API：`modbus_rtu_init()` / `modbus_rtu_poll()`

## ascii_frame.c 职责

- `reg_ascii[128]` 寄存器组（40001~40128 映射，index 0..127），低字节 ASCII、高字节 0
- `ascii_frame_putch(char)`：追加字符（满 128 忽略），重置空闲计数
- `ascii_frame_backspace()`：删上一个字符（index>0 时回退并清零）
- `ascii_frame_commit()`：余段清零 + 复位 index，提交当前帧
- `ascii_frame_poll()`：主循环心跳驱动，空闲 500ms（宏 `ASCII_IDLE_MS` 可调）自动 commit
- `ascii_frame_get(index)`：读单个寄存器字节（供 modbus_rtu 应答）

## Modbus 协议细节

- 从机地址：1（宏 `MODBUS_ADDR` 可调）
- 寄存器：40001~40128 ↔ Modbus 地址 0x0000~0x007F ↔ reg_ascii[0..127]
- 每寄存器：高字节=0x00，低字节=ASCII；无字符=0x0000
- 功能码：仅 0x03 读保持寄存器
- 单次读取数量 ≤125（标准限制），超出或地址越界 → 异常码 0x02；数量=0 → 0x02；功能码非法 → 0x01

帧格式：
```
收帧: [地址][0x03][起始Hi][Lo][数量Hi][Lo][CRC Lo][Hi]
应答: [地址][0x03][字节数=数量×2][数据...][CRC Lo][Hi]
异常: [地址][0x83][异常码][CRC Lo][Hi]
```

## 数据流

```
USB 键盘 → usb_host_hid 解析 → 事件队列
  → main process_key_events:
      ├─ 可打印字符 → ascii_frame_putch()   (写 reg_ascii + 重置 500ms 超时)
      ├─ Backspace → ascii_frame_backspace()
      ├─ Enter → ascii_frame_commit()
      └─ UART1 打印（保留现有调试输出）
  → 主循环心跳: 空闲 500ms → ascii_frame 自动 commit
  → main 每轮调用 modbus_rtu_poll(): 收帧/解析/应答
```

按键处理约定：Enter（usage 0x28）立即提交；Backspace（usage 0x2A）删字；非 ASCII 键（Fn/媒体/箭头/F1~F12/修饰键）忽略不写入。

## 边界情况与错误处理

| 场景 | 行为 |
|---|---|
| 帧长 < 8 或 CRC 错 | 静默丢弃，不响应 |
| 地址不匹配（≠1，非广播） | 静默丢弃 |
| 广播地址 0 | 不响应（0x03 读无广播意义） |
| 起始地址+数量 > 128 | 异常码 0x02 |
| 数量 = 0 | 异常码 0x02 |
| 数量 > 125 | 异常码 0x02 |
| 帧缓冲满 128 字符 | 忽略后续字符，等待 commit |
| 半帧超时（收到部分后停滞） | 3.5 字符空闲自动复位接收状态机 |
| 应答发送期间主循环阻塞 | 9600 下 1 字节≈1.04ms，应答 ≤252 字节，短时阻塞可接受 |

## 测试验证

1. 构建：Keil CLI 编译 0 Error
2. PC 主站联调（RS485 转 USB 接 PC；可先 USB-TTL 直连验证协议，再上 485）：
   - Modbus Poll 读 40001 起 → 键盘输入字符正确、高字节 0
   - 无字符寄存器 = 0
   - Enter → 立即提交；停止输入 500ms → 超时提交
   - Backspace → 前一位清 0
   - 分段读：40001~40125（125 个）与 40126~40128 两次读取
   - 错误注入：错误 CRC、错误地址、越界地址 → 无响应或异常码正确
3. 回归：UART1 调试打印、USB 枚举、MK5 全速键盘不受影响
4. 硬件：示波器验证 PA6 方向切换时序（发送期间 DE 为高、最后一位完整发出）
