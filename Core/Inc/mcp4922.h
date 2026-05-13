/*
 * mcp4922.h
 *
 *  Created on: Sep 12, 2025
 *      Author: gvigelet
 */

#ifndef INC_MCP4922_H_
#define INC_MCP4922_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Device handle ===== */
typedef struct {
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_port;   // Required: manual CS
    uint16_t      cs_pin;

    GPIO_TypeDef *ldac_port; // Optional: NULL if LDAC tied low
    uint16_t      ldac_pin;

    GPIO_TypeDef *shdn_port; // Optional: NULL if unused
    uint16_t      shdn_pin;

    /* Analog reference in millivolts for convenience helpers */
    uint32_t vref_mV;        // e.g., 3300 for 3.3V reference

    uint8_t chan_count;

} MCP4922_Handle;

/* ===== Options ===== */
typedef enum {
    MCP4922_CH_A = 0,
    MCP4922_CH_B = 1
} MCP4922_Channel;

typedef enum {
    MCP4922_BUF_OFF = 0,   // VREF unbuffered
    MCP4922_BUF_ON  = 1    // VREF buffered
} MCP4922_Buffer;

typedef enum {
    MCP4922_GAIN_2X = 0,   // Output = 2 * (Code/4096) * VREF
    MCP4922_GAIN_1X = 1    // Output = 1 * (Code/4096) * VREF
} MCP4922_Gain;

typedef enum {
    MCP4922_SHDN = 0,  // DAC powered down (output ~Hi-Z)
    MCP4922_ACTIVE = 1 // DAC active
} MCP4922_Power;

/* ===== API ===== */
HAL_StatusTypeDef MCP4922_Init(MCP4922_Handle *dev);

/* Raw 12-bit write (0..4095). Updates immediately unless LDAC is held high. */
HAL_StatusTypeDef MCP4922_WriteRaw(MCP4922_Handle *dev,
                                   MCP4922_Channel ch,
                                   MCP4922_Buffer buf,
                                   MCP4922_Gain gain,
                                   MCP4922_Power pwr,
                                   uint16_t code12);

/* Millivolt helper (clips to range); uses dev->vref_mV and selected gain */
HAL_StatusTypeDef MCP4922_WritemV(MCP4922_Handle *dev,
                                  MCP4922_Channel ch,
                                  MCP4922_Buffer buf,
                                  MCP4922_Gain gain,
                                  MCP4922_Power pwr,
                                  uint32_t out_mV);

/* Latch both channels together when LDAC is GPIO-held high */
void MCP4922_Latch(MCP4922_Handle *dev);

/* Convenience: toggle SHDN pin if wired as GPIO (digital, not the SPI “shutdown” bit) */
void MCP4922_GPIO_ShutdownSet(MCP4922_Handle *dev, bool enable);

#ifdef __cplusplus
}
#endif
#endif /* INC_MCP4922_H_ */
