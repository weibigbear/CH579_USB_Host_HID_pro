#ifndef __KEYMAP_H
#define __KEYMAP_H

#include "CH57x_common.h"

/*******************************************************************************
* 键位语义层: HID usage 码 → 显示名/字符
* 覆盖 0x04-0x65 全键盘区 + 修饰键 + Consumer 页(实测校正)
*******************************************************************************/
const char *usage_name( UINT16 code );
const char *consumer_usage_name( UINT16 u );
const char *key_display( UINT16 usage, UINT8 shift );   /* 字母随 shift^caps 大小写, 符号随 Shift 移位 */
void keymap_caps_toggle( void );                        /* CapsLock 状态切换 */

#endif /* __KEYMAP_H */
