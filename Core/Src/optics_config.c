/*
 * optics_config.c — per-product hardware mapping for optics devices
 */

#include "optics.h"
#include <stdbool.h>

extern SPI_HandleTypeDef hspi1;

/* Map each optics device to its ADC/DAC handles */

static OpticsDevice OPTICS_MAP[] = {
    {
        .adc_handle = {
            .dev_addr = 1,
            .cs_port = ADC_CS_GPIO_Port,
            .cs_pin = ADC_CS_Pin,
            .hspi = &hspi1,
			.chan_count = 4
        },
        .dac_handle = {
            .cs_port = DAC_CS_GPIO_Port,
            .cs_pin = DAC_CS_Pin,
            .ldac_port = NULL,
            .ldac_pin = 0,
            .shdn_port = NULL,
            .shdn_pin = 0,
            .vref_mV = 3300,
            .hspi = &hspi1,
			.chan_count = 2
        },
        .enOneshot = true,
        /* Channel schedule for this device.
         *
         * Each entry names an ADC source from MCP3462_ScanBits. The array
         * index is the logical channel and the host-side mask bit.
         *
         * Current product: differential CH0/CH1 -> logical 0,
         *                  differential CH2/CH3 -> logical 1.
         * Host mask 0x03 selects both.
         *
         *   .channels = { MCP3462_SCAN_DIFF_A, MCP3462_SCAN_DIFF_B },
         *   .channel_count = 2,
         * 
         * To switch to single-ended CH0 + CH2 use:
         *   .channels = { MCP3462_SCAN_CH0_SE, MCP3462_SCAN_CH2_SE },
         *   .channel_count = 2,
         * (host mask stays 0x03 — bits are by position, not raw channel #).
         */
        .channels = { MCP3462_SCAN_CH0_SE, MCP3462_SCAN_CH2_SE },
        .channel_count = 2,
    }
};

/* Return descriptor for this product */
OpticsHwDesc* OpticsConfig(void)
{
    /* Set .count to the number of active entries you actually use */
    static OpticsHwDesc optics_config = {
        .count = 1,
        .map   = OPTICS_MAP,
    };
    return &optics_config;
}   
