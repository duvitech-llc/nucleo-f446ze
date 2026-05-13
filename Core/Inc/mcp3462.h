#ifndef MCP3462_H
#define MCP3462_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Handle ---------- */
typedef struct {
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;     // Chip Select (required)

    uint8_t dev_addr;         // 0..3 (matching your MCP3462 ADx pins)

    uint8_t chan_count;

} MCP3462_Handle;


/* ---------- Registers (ADDR = CMD[5:2]) ---------- */
typedef enum {
    MCP3462_REG_ADCDATA   = 0x0,
    MCP3462_REG_CONFIG0   = 0x1,
    MCP3462_REG_CONFIG1   = 0x2,
    MCP3462_REG_CONFIG2   = 0x3,
    MCP3462_REG_CONFIG3   = 0x4,
    MCP3462_REG_IRQ       = 0x5,
    MCP3462_REG_MUX       = 0x6,
    MCP3462_REG_SCAN      = 0x7,
    MCP3462_REG_TIMER     = 0x8,
    MCP3462_REG_OFFSETCAL = 0x9,
    MCP3462_REG_GAINCAL   = 0xA,
    MCP3462_REG_LOCK      = 0xD,
    MCP3462_REG_CRCCFG    = 0xF
} MCP3462_Reg;

/* CONFIG0 bitfields */
typedef enum {
    MCP346X_VREF_EXT          = (0u << 7),
    MCP346X_VREF_INT_2V4_BUF  = (1u << 7),
} mcp346x_vref_sel_t;

typedef enum {
    MCP346X_CFG0_NORMAL       = (0u << 6),
    MCP346X_CFG0_RESERVED_BIT = (1u << 6)
} mcp346x_cfg0_bit6_t;

typedef enum {
    MCP346X_CLK_EXT_00        = (0u << 4),
    MCP346X_CLK_EXT_01        = (1u << 4),
    MCP346X_CLK_INT_NO_OUT    = (2u << 4),
    MCP346X_CLK_INT_AMCLK_OUT = (3u << 4),
} mcp346x_clk_sel_t;

typedef enum {
    MCP346X_CS_NONE           = (0u << 2),
    MCP346X_CS_0P9UA          = (1u << 2),
    MCP346X_CS_3P7UA          = (2u << 2),
    MCP346X_CS_15UA           = (3u << 2),
} mcp346x_cs_sel_t;

typedef enum {
    MCP346X_ADC_SHUTDOWN_00   = 0u,
    MCP346X_ADC_SHUTDOWN_01   = 1u,
    MCP346X_ADC_STANDBY       = 2u,
    MCP346X_ADC_CONVERSION    = 3u,
} mcp346x_adc_mode_t;

/* ---------- SCAN register helpers ---------- */
/* SCAN[15:0] bit meanings (MCP3462: CH0..CH3 + internal sources) */
typedef enum {
    MCP3462_SCAN_CH0_SE   = (1u << 0),   // single-ended CH0
    MCP3462_SCAN_CH1_SE   = (1u << 1),   // single-ended CH1
    MCP3462_SCAN_CH2_SE   = (1u << 2),
    MCP3462_SCAN_CH3_SE   = (1u << 3),

    MCP3462_SCAN_DIFF_A   = (1u << 8),   // CH0 – CH1
    MCP3462_SCAN_DIFF_B   = (1u << 9),   // CH2 – CH3

    MCP3462_SCAN_TEMP     = (1u << 12),
    MCP3462_SCAN_AVDD     = (1u << 13),
    MCP3462_SCAN_VCM      = (1u << 14),
    MCP3462_SCAN_OFFSET   = (1u << 15),
} MCP3462_ScanBits;

typedef struct {
    uint16_t scan_mask;   // OR of MCP3462_ScanBits
    uint8_t  dly_clocks;  // DLY[2:0], 0..7 => 0..512 * DMCLK between channels
    uint32_t timer_clocks;// 24-bit TIMER value between SCAN cycles (if continuous)
} MCP3462_ScanConfig;

/* ---------- Command types (CMD[1:0]) ---------- */
typedef enum {
    MCP3462_CMDTYPE_FAST       = 0x00,  // 00
    MCP3462_CMDTYPE_STATIC_RD  = 0x01,  // 01
    MCP3462_CMDTYPE_STATIC_WR  = 0x02,  // 10
    MCP3462_CMDTYPE_INC_RD     = 0x03   // 11
} MCP3462_CmdType;

/* ---------- Fast commands (use ADDR field as code) ---------- */
typedef enum {
    MCP3462_FC_CONV_START = 0x0A,
    MCP3462_FC_STANDBY    = 0x0B,
    MCP3462_FC_SHUTDOWN   = 0x0C,
    MCP3462_FC_FULL_SHDN  = 0x0D,
    MCP3462_FC_FULL_RESET = 0x0E
} MCP3462_FastCmd;

/* CONFIG1 */
#define MCP3462_CONFIG1_PRE_SHIFT 6
#define MCP3462_CONFIG1_OSR_SHIFT 2
typedef enum {
    MCP3462_OSR_32=0, MCP3462_OSR_64, MCP3462_OSR_128, MCP3462_OSR_256,
    MCP3462_OSR_512, MCP3462_OSR_1024, MCP3462_OSR_2048, MCP3462_OSR_4096,
    MCP3462_OSR_8192, MCP3462_OSR_16384, MCP3462_OSR_20480, MCP3462_OSR_24576,
    MCP3462_OSR_40960, MCP3462_OSR_49152, MCP3462_OSR_81920, MCP3462_OSR_98304
} MCP3462_OSR;

/* CONFIG2 */
#define MCP3462_CONFIG2_BOOST_SHIFT 6
#define MCP3462_CONFIG2_GAIN_SHIFT  3
#define MCP3462_CONFIG2_AZ_MUX_BIT  2
typedef enum {
    MCP3462_GAIN_0P33=0, MCP3462_GAIN_1, MCP3462_GAIN_2, MCP3462_GAIN_4,
    MCP3462_GAIN_8, MCP3462_GAIN_16, MCP3462_GAIN_32, MCP3462_GAIN_64
} MCP3462_Gain;

/* CONFIG3 */
#define MCP3462_CONFIG3_CONVMODE_SHIFT 6
#define MCP3462_CONFIG3_DATAFMT_SHIFT  4
#define MCP3462_CONFIG3_CRCFMT_BIT     3
#define MCP3462_CONFIG3_EN_CRCCOM_BIT  2
#define MCP3462_CONFIG3_EN_OFFCAL_BIT  1
#define MCP3462_CONFIG3_EN_GAINCAL_BIT 0
typedef enum {
    MCP3462_CONV_CONT        = 0x3,
    MCP3462_CONV_1SHOT_STBY  = 0x2,
    MCP3462_CONV_1SHOT_SHDN  = 0x0
} MCP3462_ConvMode;
typedef enum {
    MCP3462_DATAFMT_16      = 0x0,
    MCP3462_DATAFMT_32_ZP   = 0x1,
    MCP3462_DATAFMT_32_SIGN = 0x2,
    MCP3462_DATAFMT_32_FULL = 0x3
} MCP3462_DataFmt;

typedef enum {
    MCP3462_CH0,
    MCP3462_CH1,
    MCP3462_CH2,
    MCP3462_CH3,
    MCP3462_CH4,
    MCP3462_CH5,
    MCP3462_CH6,
    MCP3462_CH7,
    MCP3462_AGND,
    MCP3462_AVDD,
    MCP3462_RESERVED,
    MCP3462_REFIN_POZ,
    MCP3462_REFIN_NEG,
    MCP3462_TEMP_DIODE_P,
    MCP3462_TEMP_DIODE_M,
    MCP3462_INTERNAL_VCM,
} MCP3462_MuxNames;


/* ---------- API ---------- */
HAL_StatusTypeDef MCP3462_Init(MCP3462_Handle *dev);
HAL_StatusTypeDef MCP3462_FastCommand(MCP3462_Handle *dev, MCP3462_FastCmd fc);

HAL_StatusTypeDef MCP3462_WriteReg(MCP3462_Handle *dev, MCP3462_Reg r,
                                   const uint8_t *data, uint8_t len);
HAL_StatusTypeDef MCP3462_ReadReg (MCP3462_Handle *dev, MCP3462_Reg r,
                                   uint8_t *data, uint8_t len);

/* Simple single-channel configuration */
HAL_StatusTypeDef MCP3462_ConfigSimple(MCP3462_Handle *dev,
                                       MCP3462_OSR osr, MCP3462_Gain gain,
                                       MCP3462_DataFmt fmt, MCP3462_ConvMode mode,
                                       uint8_t vinp_sel, uint8_t vinn_sel);

/* Data reads (use format that matches CONFIG3.DATA_FORMAT) */
HAL_StatusTypeDef MCP3462_ReadData16(MCP3462_Handle *dev, int16_t *out);
HAL_StatusTypeDef MCP3462_ReadData32(MCP3462_Handle *dev, int32_t *out);
HAL_StatusTypeDef MCP3462_ReadData16_INC(MCP3462_Handle *dev, int16_t *out);
HAL_StatusTypeDef MCP3462_ReadData32_INC(MCP3462_Handle *dev, int32_t *out);

/* Poll DRDY via STATUS (no IRQ pin needed) */
bool MCP3462_DataReadyStatus(MCP3462_Handle *dev);

/* Debug: dump a few key registers (CONFIGx, MUX, IRQ) into your buffer */
HAL_StatusTypeDef MCP3462_DumpRegs(MCP3462_Handle *dev, uint8_t *buf, uint8_t buflen);

/* SCAN mode configuration + read */
HAL_StatusTypeDef MCP3462_ConfigScan(MCP3462_Handle *dev,
                                     MCP3462_OSR   osr,
                                     MCP3462_Gain  gain,
									 MCP3462_ConvMode mode,
                                     const MCP3462_ScanConfig *scan_cfg);

HAL_StatusTypeDef MCP3462_ReadScanSample(MCP3462_Handle *dev,
                                         uint8_t *ch_id,
                                         int32_t *code);

/* Translate a single MCP3462_ScanBits selection into the MUX register byte
 * (VIN+ << 4) | VIN- needed to convert that source in non-SCAN mode.
 * Returns 0xFF if `bit` is not a single recognised scan bit. */
uint8_t MCP3462_MuxByteForScanBit(MCP3462_ScanBits bit);

#ifdef __cplusplus
}
#endif
#endif
