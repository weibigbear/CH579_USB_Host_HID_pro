# 2026-08-15 同键重复按下不输出 — 超时推断释放

## 目标

修复：同一按键连按两次（如 `s` → `s`）第二次不输出；必须先按别的键再按同键才能输出。
要求**键盘 + 媒体键（consumer 接口）一并修复**，日志格式 `1dn 1up` 保持不变，
其余功能（Modbus、看门狗、连发已禁用）一律不动。

## 根因

该键盘固件**只在按下时上报一次报告**，松开/保持不上报任何数据
（无空报告、无周期报告；修复 `HID_SetIdle` 字节序后实测确认仍如此）。
`parse_kbd_report` 是纯 diff 模型：上次快照 `last_keys` 收不到松开报告而不更新，
同键再次按下时报告与上次完全相同 → diff 无变化 → 不产生 PRESS → 该键丢失。
中间按了别的键会触发 RELEASE diff 清掉旧键，同键才恢复。

## 方案

**方案 A（选定）：超时推断释放**。键盘不报松开，固件以"距上一条报告超过阈值仍无
新报告"推断此前所有键已松开：补发 RELEASE、清空快照、再走原 diff → 同键重按视为新按下。

不选 B（每份报告=新按下，键盘若对一次按下发重复报告会出双字符、且放弃 diff 语义）。
不选 C（只对超时后仍出现的同键定向处理，逻辑更绕且对"按住+另一键"无更好表现）。

## 改动（仅 `bsp/usb_host_hid.c`）

### 1. tick 源

- 新增文件级静态 `static UINT16 usb_poll_tick = 0;`（放轮询统计区，约 line 55）
- `PollHIDEndpoints()` 内 `diag_poll++` 旁加 `usb_poll_tick++;`（每主循环轮询一次，约 2.4ms/轮）
- 设备未就绪时不轮询、也不会有报告，时间基准与报告到达保持一致

### 2. 超时宏

```c
#define KBD_RELEASE_TIMEOUT_TICKS  20    /* ≈50ms @ 2.4ms/轮 */
```

### 3. `parse_kbd_report` 开头插入

```c
if( ( UINT16 )( usb_poll_tick - last_tick ) > KBD_RELEASE_TIMEOUT_TICKS )
{
    for( i = 0; i < 6; i ++ )
        if( last_keys[ i ] )
            kbd_ev_push( KEV_RELEASE, last_mods, 0, last_keys[ i ] );
    for( i = 0; i < 6; i ++ ) last_keys[ i ] = 0;
}
last_tick = usb_poll_tick;
```

新增静态 `static UINT16 last_tick = 0;` 与 `last_keys`/`last_mods` 并列。
注意 `last_tick = usb_poll_tick` 放在 `if( len < 3 ) return;` **之前**，任何报告都刷新基准。

### 4. `parse_consumer_report` 同样处理

```c
static UINT16 last_tick = 0;   /* 与 last_cu/last_n 并列 */
if( ( UINT16 )( usb_poll_tick - last_tick ) > KBD_RELEASE_TIMEOUT_TICKS )
{
    for( i = 0; i < last_n; i ++ )
        kbd_ev_push( KEV_RELEASE, 0, 1, last_cu[ i ] );
    last_n = 0;
}
last_tick = usb_poll_tick;
```

## 边界与权衡

- **按住一个键 >50ms 再按另一键** → 补发 RELEASE + 重发该键，多发一个字符。
  已确认以逐个敲键为主、该场景罕见，接受此代价。
- **快速连打不同键**（间隔 <50ms）→ 仍在超时窗口内，走原 diff，保持精确。
- **修饰键（Shift/Ctrl）** 释放仍靠下一条报告 mods diff 检测，行为不变。
- 超时阈值 50ms 大于该键盘重复报告间隔（0，一次按下仅一条，无重复报告），
  远小于人手同键重按最小报告间隔（~80-120ms）→ 快速打 `ss` 也能识别。
- `UINT16` tick 回绕（约 157s）用无符号减法比较，安全。

## 验证

1. Keil 命令行构建（`UV4.exe -r ... -o build_log.txt -j0`）→ `0 Error(s)`
2. 功能测试：
   - `s`、`s` 连按 → 各输出 1 个
   - `s`、`d`、`s` → 各 1 个；快速连打 `asdf` → 各 1 个
   - 长按 `s` → 1 个（连发已禁用）
   - 媒体键连按同键 → 均输出
3. 日志格式回归：仍为 `KEY:  Ndn Nup "chars"`，`1dn 1up` 语义不变
4. 边界抽查：按住 `s` >50ms 再按 `d` → 多发 1 个 `s`（已知、可接受）

## 不做

- 不新增周期性空报告 / 不上报松开的键盘固件侧改动（键盘为成品，无法改）
- 不改 `HID_SetIdle`（字节序已修复于 c231073）
- 不动 `.embeddedskills/config.json`（本地串口配置）
