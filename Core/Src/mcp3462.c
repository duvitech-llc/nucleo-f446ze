#include "mcp3462.h"
#include <string.h>
#include <stdio.h>
#include <util.h>

/* ---------- GPIO helpers ---------- */
#define CS_LOW(d)   HAL_GPIO_WritePin((d)->cs_port, (d)->cs_pin, GPIO_PIN_RESET)
#define CS_HIGH(d)  HAL_GPIO_WritePin((d)->cs_port, (d)->cs_pin, GPIO_PIN_SET)

/* ---------- SPI core ---------- */
static HAL_StatusTypeDef txrx(MCP3462_Handle *dev, uint8_t *tx, uint8_t *rx, uint16_t n) {
    return HAL_SPI_TransmitReceive(dev->hspi, tx, rx, n, 500);
}
static inline uint8_t CMD(MCP3462_Handle *dev, uint8_t addr, MCP3462_CmdType t) {
    return (uint8_t)(((dev->dev_addr & 0x3) << 6) | ((addr & 0xF) << 2) | (t & 0x3));
}

uint8_t MCP3462_MuxByteForScanBit(MCP3462_ScanBits bit) {
    /* MUX register: high nibble = VIN+ select, low nibble = VIN- select.
     * Selects use MCP3462_MuxNames codes (CH0..CH3 = 0..3, AGND = 0x8,
     * AVDD = 0x9, REFIN+/- = 0xB/0xC, TEMP_DIODE_P/M = 0xD/0xE, VCM = 0xF). */
    switch (bit) {
        case MCP3462_SCAN_CH0_SE: return (MCP3462_CH0  << 4) | MCP3462_AGND;        /* 0x08 */
        case MCP3462_SCAN_CH1_SE: return (MCP3462_CH1  << 4) | MCP3462_AGND;        /* 0x18 */
        case MCP3462_SCAN_CH2_SE: return (MCP3462_CH2  << 4) | MCP3462_AGND;        /* 0x28 */
        case MCP3462_SCAN_CH3_SE: return (MCP3462_CH3  << 4) | MCP3462_AGND;        /* 0x38 */
        case MCP3462_SCAN_DIFF_A: return (MCP3462_CH0  << 4) | MCP3462_CH1;         /* 0x01 */
        case MCP3462_SCAN_DIFF_B: return (MCP3462_CH2  << 4) | MCP3462_CH3;         /* 0x23 */
        case MCP3462_SCAN_TEMP:   return (MCP3462_TEMP_DIODE_P << 4) | MCP3462_TEMP_DIODE_M; /* 0xDE */
        case MCP3462_SCAN_AVDD:   return (MCP3462_AVDD << 4) | MCP3462_AGND;        /* 0x98 */
        case MCP3462_SCAN_VCM:    return (MCP3462_INTERNAL_VCM << 4) | MCP3462_AGND;/* 0xF8 */
        case MCP3462_SCAN_OFFSET: return (MCP3462_INTERNAL_VCM << 4) | MCP3462_INTERNAL_VCM; /* 0xFF */
        default: return 0xFF;
    }
}

/* ---------- Public API ---------- */
HAL_StatusTypeDef MCP3462_FastCommand(MCP3462_Handle *dev, MCP3462_FastCmd fc) {
    if (!dev) return HAL_ERROR;
    uint8_t c = CMD(dev, (uint8_t)fc, MCP3462_CMDTYPE_FAST);
    uint8_t dummy;
    CS_LOW(dev);
    HAL_StatusTypeDef st = txrx(dev, &c, &dummy, 1);
    CS_HIGH(dev);
    return st;
}

HAL_StatusTypeDef MCP3462_WriteReg(MCP3462_Handle *dev, MCP3462_Reg r, const uint8_t *data, uint8_t len) {
    if (!dev || !data || !len) return HAL_ERROR;
    uint8_t c = CMD(dev, (uint8_t)r, MCP3462_CMDTYPE_STATIC_WR);
    uint8_t status;
    CS_LOW(dev);
    HAL_StatusTypeDef st = txrx(dev, &c, &status, 1); // STATUS
    if (st == HAL_OK) st = HAL_SPI_Transmit(dev->hspi, (uint8_t*)data, len, 500);
    CS_HIGH(dev);
    return st;
}

HAL_StatusTypeDef MCP3462_ReadReg(MCP3462_Handle *dev, MCP3462_Reg r, uint8_t *data, uint8_t len) {
    if (!dev || !data || !len) return HAL_ERROR;
    uint8_t c = CMD(dev, (uint8_t)r, MCP3462_CMDTYPE_STATIC_RD);
    uint8_t status;
    CS_LOW(dev);
    HAL_StatusTypeDef st = txrx(dev, &c, &status, 1); // STATUS
    if (st == HAL_OK) st = HAL_SPI_Receive(dev->hspi, data, len, 500);
    CS_HIGH(dev);
    return st;
}

HAL_StatusTypeDef MCP3462_Init(MCP3462_Handle *dev) {
    if (!dev || !dev->hspi || !dev->cs_port) return HAL_ERROR;
    CS_HIGH(dev);

    /* Full reset → unlock map (LOCK=0xA5) per datasheet */
    HAL_StatusTypeDef st = MCP3462_FastCommand(dev, MCP3462_FC_FULL_RESET);
    if (st != HAL_OK) return st;
    delay_us(100);

    uint8_t key = 0xA5;
    return MCP3462_WriteReg(dev, MCP3462_REG_LOCK, &key, 1);
}

/* Helper to build CONFIG0 */
static inline uint8_t mcp346x_build_config0(mcp346x_vref_sel_t vref,
                                            mcp346x_clk_sel_t clk,
                                            mcp346x_cs_sel_t cs,
                                            mcp346x_adc_mode_t mode)
{
    uint8_t cfg0 = 0u;
    cfg0 |= vref;     // bit 7
    cfg0 |= clk;      // bits 5-4
    cfg0 |= cs;       // bits 3-2
    cfg0 |= mode;     // bits 1-0
    return cfg0;
}

/* Simple single-channel setup (your existing CH0 code path) */
HAL_StatusTypeDef MCP3462_ConfigSimple(MCP3462_Handle *dev,
                                       MCP3462_OSR osr, MCP3462_Gain gain,
                                       MCP3462_DataFmt fmt, MCP3462_ConvMode mode,
                                       uint8_t vinp, uint8_t vinn)
{
    if (!dev) return HAL_ERROR;

    uint8_t cfg0 = mcp346x_build_config0(
        MCP346X_VREF_EXT,
        MCP346X_CLK_INT_NO_OUT,
        MCP346X_CS_NONE,
        MCP346X_ADC_CONVERSION
    );
    printf("Set CFG0: 0x%02X\r\n", cfg0);
    HAL_StatusTypeDef st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG0, &cfg0, 1);
    if (st != HAL_OK) return st;

    uint8_t cfg1 = (0u << MCP3462_CONFIG1_PRE_SHIFT)
                 | ((uint8_t)osr << MCP3462_CONFIG1_OSR_SHIFT)
                 | (0u << 0);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG1, &cfg1, 1);
    if (st != HAL_OK) return st;

    uint8_t cfg2 = (0x0u << MCP3462_CONFIG2_BOOST_SHIFT)
                 | ((uint8_t)gain << MCP3462_CONFIG2_GAIN_SHIFT)
                 | (1u << MCP3462_CONFIG2_AZ_MUX_BIT)
                 | 0x03u;  // reserved bits [1:0] = '11'
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG2, &cfg2, 1);
    if (st != HAL_OK) return st;

    uint8_t cfg3 = ((uint8_t)mode << MCP3462_CONFIG3_CONVMODE_SHIFT)
                 | ((uint8_t)fmt  << MCP3462_CONFIG3_DATAFMT_SHIFT);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG3, &cfg3, 1);
    if (st != HAL_OK) return st;

    uint8_t mux = (uint8_t)((vinp & 0xF) << 4) | (uint8_t)(vinn & 0xF);
    st = MCP3462_WriteReg(dev, MCP3462_REG_MUX, &mux, 1);
    if (st != HAL_OK) return st;

    uint8_t irq;
    MCP3462_ReadReg(dev, MCP3462_REG_IRQ, &irq, 1);

    return MCP3462_FastCommand(dev, MCP3462_FC_CONV_START);
}

/* Data reads */
HAL_StatusTypeDef MCP3462_ReadData16_INC(MCP3462_Handle *dev, int16_t *out)
{
    if (!dev || !out) return HAL_ERROR;

    uint8_t cmd = (uint8_t)(((dev->dev_addr & 0x3) << 6)
                   | ((MCP3462_REG_ADCDATA & 0xF) << 2)
                   | MCP3462_CMDTYPE_INC_RD);

    uint8_t rx[3] = {0};
    uint8_t tx0 = cmd;

    CS_LOW(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, &tx0, &rx[0], 1, 500);
    if (st == HAL_OK) {
        st = HAL_SPI_Receive(dev->hspi, &rx[1], 2, 500);
    }
    CS_HIGH(dev);
    if (st != HAL_OK) return st;

    /* Status byte layout: [DEV_ADDR1][DEV_ADDR0][nDR][nCRCCFG][nPOR][0][0][0]
     * nDR is bit 5 (mask 0x20), 0 = data ready. */
    bool ready = ((rx[0] & 0x20u) == 0u);
    if (!ready) return HAL_BUSY;

    *out = (int16_t)((rx[1] << 8) | rx[2]);
    return HAL_OK;
}

HAL_StatusTypeDef MCP3462_ReadData16(MCP3462_Handle *dev, int16_t *code) {
    if (!dev || !code) return HAL_ERROR;
    uint8_t buf[2];
    HAL_StatusTypeDef st = MCP3462_ReadReg(dev, MCP3462_REG_ADCDATA, buf, 2);
    if (st == HAL_OK) {
        *code = (int16_t)((buf[0] << 8) | buf[1]);
    }
    return st;
}

HAL_StatusTypeDef MCP3462_ReadData32_INC(MCP3462_Handle *dev, int32_t *out)
{
    if (!dev || !out) return HAL_ERROR;

    uint8_t cmd = (uint8_t)(((dev->dev_addr & 0x3) << 6)
                   | ((MCP3462_REG_ADCDATA & 0xF) << 2)
                   | MCP3462_CMDTYPE_INC_RD);
    uint8_t rx[5] = {0};

    CS_LOW(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, &cmd, &rx[0], 1, 500);
    if (st == HAL_OK) st = HAL_SPI_Receive(dev->hspi, &rx[1], 4, 500);
    CS_HIGH(dev);
    if (st != HAL_OK) return st;

    /* nDR is bit 5 of status (mask 0x20); 0 = data ready. */
    if ((rx[0] & 0x20u) != 0u) return HAL_BUSY;

    *out = (int32_t)((uint32_t)rx[1]<<24 | (uint32_t)rx[2]<<16
                   | (uint32_t)rx[3]<<8  | (uint32_t)rx[4]);
    return HAL_OK;
}

HAL_StatusTypeDef MCP3462_ReadData32(MCP3462_Handle *dev, int32_t *out) {
    if (!dev || !out) return HAL_ERROR;
    uint8_t b[4];
    HAL_StatusTypeDef st = MCP3462_ReadReg(dev, MCP3462_REG_ADCDATA, b, 4);
    if (st == HAL_OK) {
        *out = (int32_t)((uint32_t)b[0] << 24 | (uint32_t)b[1] << 16
                       | (uint32_t)b[2] << 8  | (uint32_t)b[3]);
    }
    return st;
}

/* Ready check via STATUS (SPI polling) */
bool MCP3462_DataReadyStatus(MCP3462_Handle *dev) {
    if (!dev) return false;
    uint8_t c = CMD(dev, (uint8_t)MCP3462_REG_ADCDATA, MCP3462_CMDTYPE_STATIC_RD);
    uint8_t status = 0;
    CS_LOW(dev);
    if (txrx(dev, &c, &status, 1) != HAL_OK) { CS_HIGH(dev); return false; }
    CS_HIGH(dev);
    /* nDR bit (bit5) = 0 when data is ready */
    return (status & 0x20u) == 0u;
}

/* Debug: dump registers */
HAL_StatusTypeDef MCP3462_DumpRegs(MCP3462_Handle *dev, uint8_t *buf, uint8_t buflen) {
    if (!buf || buflen < 8) return HAL_ERROR;
    uint8_t off = 0; uint8_t v;

    printf("MCP3462 Register Dump:\r\n");
    printf("=====================\r\n");

    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG0, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG0: 0x%02X\r\n", v);
    } else printf("CONFIG0: READ FAILED\r\n");

    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG1, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG1: 0x%02X\r\n", v);
    } else printf("CONFIG1: READ FAILED\r\n");

    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG2, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG2: 0x%02X\r\n", v);
    } else printf("CONFIG2: READ FAILED\r\n");

    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG3, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG3: 0x%02X\r\n", v);
    } else printf("CONFIG3: READ FAILED\r\n");

    if (MCP3462_ReadReg(dev, MCP3462_REG_MUX, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("MUX:     0x%02X\r\n", v);
    } else printf("MUX:     READ FAILED\r\n");

    if (MCP3462_ReadReg(dev, MCP3462_REG_IRQ, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("IRQ:     0x%02X\r\n", v);
    } else printf("IRQ:     READ FAILED\r\n");

    printf("=====================\r\n");
    return HAL_OK;
}

/* ---------- SCAN mode ---------- */
HAL_StatusTypeDef MCP3462_ConfigScan(MCP3462_Handle *dev,
                                     MCP3462_OSR   osr,
                                     MCP3462_Gain  gain,
									 MCP3462_ConvMode mode,
                                     const MCP3462_ScanConfig *scan_cfg)
{
    if (!dev || !scan_cfg) return HAL_ERROR;

    HAL_StatusTypeDef st;

    /* CONFIG0: internal RC, no burnout, ADC in CONVERSION mode */
    uint8_t cfg0 = mcp346x_build_config0(
        MCP346X_VREF_EXT,
        MCP346X_CLK_INT_NO_OUT,
        MCP346X_CS_NONE,
        MCP346X_ADC_CONVERSION
    );
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG0, &cfg0, 1);
    if (st != HAL_OK) return st;

    /* CONFIG1: PRE=0, OSR as requested */
    uint8_t cfg1 = (0u << MCP3462_CONFIG1_PRE_SHIFT)
                 | ((uint8_t)osr << MCP3462_CONFIG1_OSR_SHIFT);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG1, &cfg1, 1);
    if (st != HAL_OK) return st;

    /* CONFIG2: BOOST=1x, gain, AZ_MUX=1, RESERVED[1:0]=11 */
    uint8_t cfg2 = (0x0u << MCP3462_CONFIG2_BOOST_SHIFT)
                 | ((uint8_t)gain << MCP3462_CONFIG2_GAIN_SHIFT)
                 | (1u << MCP3462_CONFIG2_AZ_MUX_BIT)
                 | 0x03u;
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG2, &cfg2, 1);
    if (st != HAL_OK) return st;

    /* CONFIG3: continuous SCAN + 32-bit FULL (CH_ID in MSbits) */
    uint8_t cfg3 = ((uint8_t)mode        << MCP3462_CONFIG3_CONVMODE_SHIFT) |
                   ((uint8_t)MCP3462_DATAFMT_32_FULL  << MCP3462_CONFIG3_DATAFMT_SHIFT);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG3, &cfg3, 1);
    if (st != HAL_OK) return st;

    /* MUX: Set to default/don't care for SCAN mode, but write 0x01 to match working config */
    /* In SCAN mode, the SCAN register controls which channels are measured */
    uint8_t mux = 0x01;  // VIN+ = CH0, VIN- = CH1 (though SCAN should override)
    st = MCP3462_WriteReg(dev, MCP3462_REG_MUX, &mux, 1);
    if (st != HAL_OK) return st;

    /* SCAN: DLY[2:0] + SCAN[15:0] */
    uint32_t scan_val = (((uint32_t)(scan_cfg->dly_clocks & 0x7u)) << 21)
                      | ((uint32_t)scan_cfg->scan_mask & 0xFFFFu);
    uint8_t scan_bytes[3] = {
        (uint8_t)((scan_val >> 16) & 0xFF),
        (uint8_t)((scan_val >>  8) & 0xFF),
        (uint8_t)( scan_val        & 0xFF)
    };
    
    st = MCP3462_WriteReg(dev, MCP3462_REG_SCAN, scan_bytes, 3);
    if (st != HAL_OK) return st;
    
    /* Read back SCAN register to verify */
    uint8_t scan_readback[3];
    st = MCP3462_ReadReg(dev, MCP3462_REG_SCAN, scan_readback, 3);
    
    /* TIMER: delay between SCAN cycles (only used when continuous) */
    uint32_t t = scan_cfg->timer_clocks & 0x00FFFFFFu;
    uint8_t timer_bytes[3] = {
        (uint8_t)((t >> 16) & 0xFF),
        (uint8_t)((t >>  8) & 0xFF),
        (uint8_t)( t        & 0xFF)
    };
    st = MCP3462_WriteReg(dev, MCP3462_REG_TIMER, timer_bytes, 3);
    if (st != HAL_OK) return st;

    /* Read back IRQ to clear any pending flags */
    uint8_t irq_dummy;
    (void)MCP3462_ReadReg(dev, MCP3462_REG_IRQ, &irq_dummy, 1);

    /* Start / restart conversions (SCAN mode is active because SCAN[15:0] != 0) */
    return MCP3462_FastCommand(dev, MCP3462_FC_CONV_START);
}

HAL_StatusTypeDef MCP3462_ReadScanSample(MCP3462_Handle *dev,
                                         uint8_t *ch_id,
                                         int32_t *code)
{
    if (!dev || !ch_id || !code) return HAL_ERROR;

    /* In SCAN mode, use STATIC read instead of INC read to properly advance through channels */
    uint8_t cmd = (uint8_t)(((dev->dev_addr & 0x3) << 6)
                   | ((MCP3462_REG_ADCDATA & 0xF) << 2)
                   | MCP3462_CMDTYPE_STATIC_RD);
    uint8_t rx[5] = {0};

    CS_LOW(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, &cmd, &rx[0], 1, 500);
    if (st == HAL_OK) st = HAL_SPI_Receive(dev->hspi, &rx[1], 4, 500);
    CS_HIGH(dev);
    if (st != HAL_OK) return st;

    /* Status byte: nDR is bit 5 (mask 0x20); 0 = data ready. */
    if ((rx[0] & 0x20u) != 0u) return HAL_BUSY;

    int32_t raw32 = (int32_t)((uint32_t)rx[1]<<24 | (uint32_t)rx[2]<<16
                             | (uint32_t)rx[3]<<8  | (uint32_t)rx[4]);

    /* DATA_FORMAT = 32_FULL (DATA_FORMAT[1:0] = 0b11)
     *
     * 32-bit layout (MSB..LSB):
     *   [31:28] = CH_ID[3:0]
     *   [27:16] = sign extension (SGN)
     *   [15:0]  = DATA[15:0] (16-bit code)
     *
     * For inputs within [-VREF ; +VREF − 1 LSB], the lower 16 bits
     * match the 16-bit output coding. So we can safely recover the
     * measurement as a signed int16 from bits [15:0].
     */

    uint8_t cid   = (uint8_t)((raw32 >> 28) & 0x0F);        // CH_ID[3:0]
    int16_t code16 = (int16_t)(raw32 & 0x0000FFFFu);        // DATA[15:0]

    *ch_id = cid;
    *code  = (int32_t)code16;   // keep API signature, but value is 16-bit
    return HAL_OK;
}

