/*******************************************************************************
* keymap.c — 键位语义层: HID usage 码 → 显示名/字符
* 新增键盘类型时只需修改本模块的映射表
*******************************************************************************/
#include "CH57x_common.h"
#include "keymap.h"

/*******************************************************************************
* HID usage 码 → 名称 (覆盖 0x04-0x65 全键盘区 + 修饰键)
* 78 键布局对照实物校正 (Task 4 硬件测试阶段)
*******************************************************************************/
const char *usage_name( UINT16 code )
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

/*******************************************************************************
* Consumer 页 usage 名字表 (Task 5 实测后校正)
*******************************************************************************/
const char *consumer_usage_name( UINT16 u )
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
        case 0x0183: return "Fn+F3";   /* 该键盘实测键位 */
        default:     return 0;
    }
}

static UINT8 caps_lock = 0;      /* CapsLock 状态跟踪(按下切换) */
static char  key_chr[ 2 ] = "?"; /* 单字符显示缓冲 */

void keymap_caps_toggle( void )
{
    caps_lock ^= 1;
}

/*******************************************************************************
* 按键显示: 字母随 shift^caps 大小写, 数字区/标点随 Shift 出符号
*******************************************************************************/
const char *key_display( UINT16 usage, UINT8 shift )
{
    if( usage >= 0x04 && usage <= 0x1D )                    /* A-Z */
    {
        UINT8 up = ( shift != 0 ) ^ ( caps_lock != 0 );
        key_chr[ 0 ] = up ? ( char )( 'A' + ( usage - 0x04 ) )
                          : ( char )( 'a' + ( usage - 0x04 ) );
        return key_chr;
    }
    switch( usage )
    {
        case 0x1E: return shift ? "!" : "1";
        case 0x1F: return shift ? "@" : "2";
        case 0x20: return shift ? "#" : "3";
        case 0x21: return shift ? "$" : "4";
        case 0x22: return shift ? "%" : "5";
        case 0x23: return shift ? "^" : "6";
        case 0x24: return shift ? "&" : "7";
        case 0x25: return shift ? "*" : "8";
        case 0x26: return shift ? "(" : "9";
        case 0x27: return shift ? ")" : "0";
        case 0x2D: return shift ? "_" : "-";
        case 0x2E: return shift ? "+" : "=";
        case 0x2F: return shift ? "{" : "[";
        case 0x30: return shift ? "}" : "]";
        case 0x31: return shift ? "|" : "\\";
        case 0x33: return shift ? ":" : ";";
        case 0x34: return shift ? "\"" : "'";
        case 0x35: return shift ? "~" : "`";
        case 0x36: return shift ? "<" : ",";
        case 0x37: return shift ? ">" : ".";
        case 0x38: return shift ? "?" : "/";
        default:   return usage_name( usage );
    }
}
