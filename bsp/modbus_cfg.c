/*******************************************************************************
* modbus_cfg.c — Modbus 参数配置模块实现(DataFlash 掉电保存)
*
* 存储布局: 结构体 modbus_cfg_t 共 8 字节(4 字节对齐), 存于
*   DATA_FLASH_ADDR(0x3E800)。写入前先整扇区擦除(512B), 再按双字写。
*
* 校验: magic + CRC16(Poly 0xA001) 双保险, 任一不符即视为无效配置,
*   回退默认值(addr=1, baud=0)并写回 Flash。
*
* 注意: 库的 Flash 擦写函数内部自带低压检测与操作码防呆(CH57x_flash.c),
*   返回非 0 表示擦写失败(如电源电压偏低), 调用方需据此处理。
*******************************************************************************/
#include "CH57x_common.h"
#include "modbus_cfg.h"

const UINT32 modbus_baud_table[ MODBUS_BAUD_NUM ] = { 9600, 19200, 38400, 57600, 115200 };

typedef struct
{
    UINT32 magic;          /* MODBUS_CFG_MAGIC */
    UINT8  addr;           /* 从机地址 1~247 */
    UINT8  baud;           /* 波特率索引 0~4 */
    UINT16 crc;            /* 结构校验(CRC16) */
} modbus_cfg_t;            /* 共 8 字节, 4 字节对齐, 无填充 */

static modbus_cfg_t g_cfg = { MODBUS_CFG_MAGIC, MODBUS_DEF_ADDR, MODBUS_DEF_BAUD, 0 };

/*******************************************************************************
* CRC16 Modbus (Poly 0xA001, 半字节查表) — 与 modbus_rtu.c 内实现一致
*******************************************************************************/
static const UINT16 cfg_crc_tab[ 16 ] = {
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

static UINT16 cfg_crc16( const UINT8 *dataIn, UINT16 length )
{
    UINT16 crc = 0xFFFF;
    UINT16 i;
    for( i = 0; i < length; i ++ )
    {
        crc = ( crc >> 4 ) ^ cfg_crc_tab[ ( crc ^ dataIn[ i ] ) & 0x0F ];
        crc = ( crc >> 4 ) ^ cfg_crc_tab[ ( crc ^ ( dataIn[ i ] >> 4 ) ) & 0x0F ];
    }
    return crc;
}

/*******************************************************************************
* 刷新结构 CRC 字段(前 6 字节参与计算)
*******************************************************************************/
static void cfg_update_crc( void )
{
    g_cfg.crc = cfg_crc16( ( const UINT8 * )&g_cfg, 6 );
}

/*******************************************************************************
* 上电加载: 直接从 DataFlash 地址指针读取, 校验 magic + CRC;
* 无效则用默认值并回写 Flash(首次上电完成初始化)。
*******************************************************************************/
void modbus_cfg_init( void )
{
    const modbus_cfg_t *p = ( const modbus_cfg_t * )MODBUS_CFG_ADDR;

    if( p->magic == MODBUS_CFG_MAGIC && p->crc == cfg_crc16( ( const UINT8 * )p, 6 ) )
    {
        g_cfg.magic = p->magic;
        g_cfg.addr  = p->addr;
        g_cfg.baud  = p->baud;
        g_cfg.crc   = p->crc;
        /* 防御: 加载值仍须在合法范围, 否则回退默认并回写 */
        if( g_cfg.addr < 1 || g_cfg.addr > 247 || g_cfg.baud >= MODBUS_BAUD_NUM )
        {
            g_cfg.addr = MODBUS_DEF_ADDR;
            g_cfg.baud = MODBUS_DEF_BAUD;
            cfg_update_crc();
            modbus_cfg_save();
        }
    }
    else
    {
        cfg_update_crc();
        modbus_cfg_save();
    }
}

/*******************************************************************************
* 保存: 整扇区擦除后按双字写入结构体(8 字节 = 2 个双字)。
* 返回 0=成功; 非 0=擦写失败(库返回值)。
*******************************************************************************/
UINT8 modbus_cfg_save( void )
{
    UINT8 s;
    s = FlashBlockErase( MODBUS_CFG_ADDR );
    if( s != 0 ) return s;
    return FlashWriteBuf( MODBUS_CFG_ADDR, ( PUINT32 )&g_cfg, sizeof( modbus_cfg_t ) );
}

UINT8 modbus_cfg_get_addr( void ) { return g_cfg.addr; }
UINT8 modbus_cfg_get_baud( void ) { return g_cfg.baud; }

UINT8 modbus_cfg_set_addr( UINT8 a )
{
    if( a < 1 || a > 247 ) return 1;
    g_cfg.addr = a;
    cfg_update_crc();
    return 0;
}

UINT8 modbus_cfg_set_baud( UINT8 b )
{
    if( b >= MODBUS_BAUD_NUM ) return 1;
    g_cfg.baud = b;
    cfg_update_crc();
    return 0;
}
