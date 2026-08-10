/*******************************************************************************
* ascii_frame.h — ASCII 帧缓冲模块接口
*
* 功能定位: 充当 Modbus 40001~40128 保持寄存器组的"写入侧代理"。
*   键盘输入的 ASCII 字符按顺序写入内部缓冲 reg_ascii[],
*   该缓冲同时就是 Modbus 寄存器组的内容来源
*   (modbus_rtu.c 通过 ascii_frame_get() 读取应答数据)。
*
* 寄存器映射约定:
*   register index 0   ->  Modbus 寄存器 40001 (内部地址 0x0000)
*   register index 127 ->  Modbus 寄存器 40128 (内部地址 0x007F)
*   每个 16 位寄存器: 高字节恒为 0x00, 低字节即输入字符的 ASCII 码。
*
* 帧的生命周期(组成"一帧"):
*   putch()      追加一个字符(写入下一位置, 重置空闲计时)
*   backspace()  回退一位并清零(可反复删)
*   commit()     立即提交: 将未写入的余段清零、指针复位(手动, 通常是 Enter)
*   poll()       主循环心跳驱动: 停止输入超过 ASCII_IDLE_MS 自动 commit()
*
* 与外围模块的依赖: 仅依赖 CH57x_common.h, 无反向依赖; 纯内部逻辑,
*   不接触任何硬件寄存器, 便于独立测试。
*******************************************************************************/
#ifndef __ASCII_FRAME_H
#define __ASCII_FRAME_H

#include "CH57x_common.h"

#define ASCII_FRAME_SIZE   128          /* 寄存器组大小: 40001~40128 */
#define ASCII_IDLE_MS      500          /* 空闲超时提交时间, 可调;
                                          主循环每 2ms 调一次 poll(),
                                          内部按 (500/2)=250 次心跳换算 */

void ascii_frame_init( void );          /* 清零寄存器组与状态(fill: 上电后调用一次) */
void ascii_frame_putch( char c );       /* 追加一个字符(满 128 忽略), 重置空闲计数 */
void ascii_frame_backspace( void );     /* 删除上一个字符(回退并清零), 重置空闲计数 */
void ascii_frame_commit( void );        /* 立即提交: 余段清零 + 复位 index */
void ascii_frame_poll( void );          /* 主循环调用(约 2ms 一次): 空闲超时自动提交 */
UINT8 ascii_frame_get( UINT8 index );   /* 读寄存器低字节(index 0~127), 越界返回 0 */

#endif /* __ASCII_FRAME_H */