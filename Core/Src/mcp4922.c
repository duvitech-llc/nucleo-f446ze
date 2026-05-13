/*
 * mcp4922.c
 *
 *  Created on: Sep 12, 2025
 *      Author: gvigelet
 */
#include <util.h>
#include "mcp4922.h"

/* ===== Local helpers ===== */
#define CS_LOW(d)   HAL_GPIO_WritePin((d)->cs_port,   (d)->cs_pin,   GPIO_PIN_RESET)
#define CS_HIGH(d)  HAL_GPIO_WritePin((d)->cs_port,   (d)->cs_pin,   GPIO_PIN_SET)
#define LDAC_LOW(d) if ((d)->ldac_port) HAL_GPIO_WritePin((d)->ldac_port,(d)->ldac_pin,GPIO_PIN_RESET)
#define LDAC_HIGH(d)if ((d)->ldac_port) HAL_GPIO_WritePin((d)->ldac_port,(d)->ldac_pin,GPIO_PIN_SET)

static inline uint16_t build_word(MCP4922_Channel ch,
                                  MCP4922_Buffer buf,
                                  MCP4922_Gain gain,
                                  MCP4922_Power pwr,
                                  uint16_t code12)
{
    /* MCP4922 16-bit frame:
       [15]  A/B   (0=Ch A, 1=Ch B)
       [14]  BUF   (1=buffered VREF, 0=unbuffered)
       [13]  GAIN  (1=1x, 0=2x)
       [12]  SHDN  (1=active, 0=shutdown)
       [11:0] D11..D0 (data)
    */
    code12 &= 0x0FFF;
    return (uint16_t)(( (ch & 1u)  << 15 ) |
                      ( (buf & 1u) << 14 ) |
                      ( (gain& 1u) << 13 ) |
                      ( (pwr & 1u) << 12 ) |
                      code12);
}

/* ===== Public API ===== */

HAL_StatusTypeDef MCP4922_Init(MCP4922_Handle *dev)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL) {
    	return HAL_ERROR;
    }

    /* Ensure pins start in safe states */
    CS_HIGH(dev);
    if (dev->ldac_port) {
    	LDAC_LOW(dev); // LDAC low -> updates occur immediately (common default)
    }
    if (dev->shdn_port) {
    	HAL_GPIO_WritePin(dev->shdn_port, dev->shdn_pin, GPIO_PIN_SET); // not required by part, optional external gate
    }

    return HAL_OK;
}

HAL_StatusTypeDef MCP4922_WriteRaw(MCP4922_Handle *dev,
                                   MCP4922_Channel ch,
                                   MCP4922_Buffer buf,
                                   MCP4922_Gain gain,
                                   MCP4922_Power pwr,
                                   uint16_t code12)
{
    if (!dev || !dev->hspi) {
    	return HAL_ERROR;
    }

    /* Build 16-bit command word */
    uint16_t w = build_word(ch, buf, gain, pwr, code12);
    uint8_t tx[2] = { (uint8_t)(w >> 8), (uint8_t)(w & 0xFF) };

    CS_LOW(dev);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, tx, 2, 500);
    CS_HIGH(dev);

    return st;
}

HAL_StatusTypeDef MCP4922_WritemV(MCP4922_Handle *dev,
                                  MCP4922_Channel ch,
                                  MCP4922_Buffer buf,
                                  MCP4922_Gain gain,
                                  MCP4922_Power pwr,
                                  uint32_t out_mV)
{
    if (!dev || dev->vref_mV == 0) {
    	return HAL_ERROR;
    }

    /* Full-scale depends on gain:
       gain=1x: Vout = (Code/4096) * VREF
       gain=2x: Vout = (2*Code/4096) * VREF  => FS = 2*VREF  (clip as needed)
    */
    uint32_t fs_mV = (gain == MCP4922_GAIN_1X) ? dev->vref_mV : (2u * dev->vref_mV);
    if (out_mV > fs_mV) out_mV = fs_mV;

    /* Code = round( out_mV / FS * 4095 ) */
    uint32_t code = ( (uint64_t)out_mV * 4095u + (fs_mV/2) ) / fs_mV;
    if (code > 4095u) code = 4095u;

    return MCP4922_WriteRaw(dev, ch, buf, gain, pwr, (uint16_t)code);
}

void MCP4922_Latch(MCP4922_Handle *dev)
{
    /* If LDAC is held high, bringing it low latches both channels simultaneously.
       If LDAC is tied low, this does nothing (updates already immediate). */
    if (!dev || !dev->ldac_port) {
    	return;
    }

    // Low pulse ≥100 ns;
    LDAC_LOW(dev);
    delay_us(1);
    LDAC_HIGH(dev);
}

void MCP4922_GPIO_ShutdownSet(MCP4922_Handle *dev, bool enable)
{
    /* Optional helper for an external shutdown gate if you wired one.
       (Not related to the SPI “SHDN” bit in the command word.) */
    if (!dev || !dev->shdn_port) {
    	return;
    }
    HAL_GPIO_WritePin(dev->shdn_port, dev->shdn_pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
