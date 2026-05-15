#include "main.h"
#include "optics.h"
#include "util.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// #define BUFFER_SIZE 128
#define OPTICS_SAMPLE_PERIOD_MS 250

/* internal: basic handle validation */
static inline bool adc_handle_valid(const MCP3462_Handle *a) {
    return (a && a->hspi && a->cs_port);
}
static inline bool dac_handle_valid(const MCP4922_Handle *d) {
    return (d && d->hspi && d->cs_port);
}

static OpticsHwDesc* hw = NULL;
static uint8_t OpticsCount = 0;
static uint32_t active_optics_mask = 0;
static uint32_t active_laser_mask = 0;

/* Per-device: index into dev->channels[] for the channel currently being converted */
static uint8_t current_mux_idx[8] = {0};

/* Per-device cached MUX register bytes derived from dev->channels[] at init.
 * Sized to MAX_ADC_CHANNELS so [optic_index][i] addresses any logical channel. */
static uint8_t mux_bytes[8][MAX_ADC_CHANNELS] = {{0}};

/* Weak default: links even if the product doesn’t provide a mapping */
__attribute__((weak))
OpticsHwDesc* OpticsConfig(void) {
	static OpticsHwDesc null_config = {
		.count = 0,
		.map = NULL
	};

	return &null_config;
}

HAL_StatusTypeDef optics_adcStartConversion(int optic_index) {
    if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    if(dev->enOneshot){
    	return MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
    }

    return HAL_OK;
}

HAL_StatusTypeDef optics_startLaser(int optic_index, uint16_t power) {
    if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];

    dev->dacValue = (power > 100) ? 100 : power;
    dev->dacValue = (dev->dacValue * 4095) / 100;
    return MCP4922_WriteRaw(&dev->dac_handle,
    					   (MCP4922_Channel)optic_index,
                           MCP4922_BUF_OFF,
                           MCP4922_GAIN_1X,
                           MCP4922_ACTIVE,
                           dev->dacValue);
#if 0
    uint32_t fs_mV = dev->dac_handle.vref_mV;
    uint32_t out_mV = (uint32_t)dev->dacValue * fs_mV / 100u;


    return MCP4922_WritemV(&dev->dac_handle,
                           MCP4922_CH_A,
                           MCP4922_BUF_ON,
                           MCP4922_GAIN_1X,
                           MCP4922_ACTIVE,
                           out_mV);
#endif
}

HAL_StatusTypeDef optics_stopLaser(int optic_index) {
    if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    dev->dacValue = 0;
    return MCP4922_WriteRaw(&dev->dac_handle,
    					   (MCP4922_Channel)optic_index,
                           MCP4922_BUF_OFF,
                           MCP4922_GAIN_1X,
                           MCP4922_ACTIVE,
                           0);
}

static HAL_StatusTypeDef initialize_optic_device(int optic_index) {
	HAL_StatusTypeDef st = HAL_OK;
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }
    
    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];

    /* Validate wiring/handles before touching hardware */
    if (!adc_handle_valid(&dev->adc_handle)) {
        return HAL_ERROR;
    }
    if (!dac_handle_valid(&dev->dac_handle)) {
        return HAL_ERROR;
    }

    /* require a nonzero VREF for mV helper usage */
    if (dev->dac_handle.vref_mV == 0u) {
        return HAL_ERROR;
    }    

    /* Bring up devices */

    st  = MCP4922_Init(&dev->dac_handle);
    if (st != HAL_OK) {
    	return st;
    }

    st = MCP3462_Init(&dev->adc_handle);
    if (st != HAL_OK) {
    	return st;
    }

    /* Validate the per-device channel list and pre-compute MUX bytes */
    if (dev->channel_count == 0 || dev->channel_count > MAX_ADC_CHANNELS) {
        return HAL_ERROR;
    }
    for (uint8_t i = 0; i < dev->channel_count; ++i) {
        uint8_t mb = MCP3462_MuxByteForScanBit(dev->channels[i]);
        if (mb == 0xFF && dev->channels[i] != MCP3462_SCAN_OFFSET) {
            return HAL_ERROR;  /* unrecognised scan bit */
        }
        mux_bytes[optic_index][i] = mb;
    }
	
	// Build SCAN mask from the device's declared channel list so the chip
	// actually scans the same channels the host expects (CH0_SE + CH1_SE,
	// or whatever optics_config.c declares).
	uint16_t scan_mask = 0;
	for (uint8_t i = 0; i < dev->channel_count; ++i) {
		scan_mask |= (uint16_t)dev->channels[i];
	}
	MCP3462_ScanConfig scan_cfg = {
		.scan_mask    = scan_mask,
		.dly_clocks   = 0,   // no extra delay between channels
		.timer_clocks = 0    // no extra delay between SCAN cycles
	};

	st = MCP3462_ConfigScan(&dev->adc_handle, MCP3462_OSR_32, MCP3462_GAIN_1, MCP3462_CONV_1SHOT_STBY, &scan_cfg);
    if (st != HAL_OK) {
    	return st;
    }

    /* Clear capture buffer */
	for(int i = 0; i <MAX_ADC_CHANNELS; i++){
		memset(&dev->adcSamples[i], 0, ADC_UART_BUFFER_SIZE);
		dev->dataPtr[i] = 0;
	}

    return HAL_OK;
}

HAL_StatusTypeDef optics_init() {
    
    hw = OpticsConfig();

    if (!hw || !hw->map || hw->count == 0) {
        return HAL_ERROR;
    }

    OpticsCount = hw->count;
    for (uint8_t i = 0; i < OpticsCount; ++i) {
        HAL_StatusTypeDef st = initialize_optic_device(i);
        if (st != HAL_OK) {
            return st;
        }
    }
    
    active_optics_mask = 0;
    active_laser_mask = 0;

	printf("Optics initialized with %d device(s)\r\n", OpticsCount);
    return HAL_OK;
}

void optics_clearBuffers(int optic_index)
{
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    for(int i=0; i<MAX_ADC_CHANNELS; i++){
		memset(&dev->adcSamples[i], 0, ADC_UART_BUFFER_SIZE);
		dev->dataPtr[i] = 0;
    }

}

void optics_clearBuffer(int optic_index, uint8_t ch_id) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS)) {
		return;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	memset(&dev->adcSamples[ch_id], 0, ADC_UART_BUFFER_SIZE);
	dev->dataPtr[ch_id] = 0;
}

uint16_t optics_getSize(int optic_index, uint8_t ch_id) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS)) {
		return 0;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	return dev->dataPtr[ch_id];
}

uint8_t* optics_getBuffer(int optic_index, uint8_t ch_id) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS)) {
		return NULL;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	return (uint8_t*)&(dev->adcSamples[ch_id]);
}

HAL_StatusTypeDef optics_adcReadSamples(int optic_index) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return HAL_ERROR;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    HAL_StatusTypeDef st = HAL_OK;

    uint8_t ch_id;
    int32_t code32;

	/* 1-shot SCAN: kick off a fresh scan cycle for this device, then poll
	 * until we've captured one sample for each enabled channel. The chip
	 * auto-returns to STANDBY at the end of the scan (CONV_1SHOT_STBY),
	 * so there is no race against a free-running converter overwriting
	 * the ADCDATA latch. */
	st = MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
	if (st != HAL_OK) return st;

	const uint8_t expected = dev->channel_count;
	const int max_poll = 10;   /* 40us total budget */

	/* captured_mask / want_mask are indexed by the RAW MCP3462 channel id
	 * (CH0 = bit 0, CH1 = bit 1, ... CH7 = bit 7). This matches the
	 * convention used by optics_getBuffer_byMask() / optics_clearBuffer_byMask(),
	 * so a host-side mask of 0x05 means "chip channels 0 and 2". */
	uint32_t want_mask = 0;
	for (uint8_t k = 0; k < expected; ++k) {
		/* dev->channels[k] is already (1u << raw_ch) — OR them together. */
		want_mask |= (uint32_t)dev->channels[k];
	}
	uint32_t captured_mask = 0;
	uint8_t  got_count = 0;

	for (int i = 0; i < max_poll && captured_mask != want_mask; i++) {
		st = MCP3462_ReadScanSample(&dev->adc_handle, &ch_id, &code32);
		if (st == HAL_BUSY) {
			delay_us(1);
			continue;
		}
		if (st != HAL_OK) {
			return st;
		}

		/* ch_id is the raw MCP3462 channel ID (0..7). Reject anything that
		 * isn't part of the configured scan, and skip duplicates within
		 * this scan cycle. */
		if (ch_id >= MAX_ADC_CHANNELS) {
			continue;
		}
		uint32_t ch_bit = (1u << ch_id);
		if ((want_mask & ch_bit) == 0u) {
			continue;
		}
		if (captured_mask & ch_bit) {
			continue;
		}

		uint16_t code16 = (uint16_t)code32;
		if (dev->dataPtr[ch_id] + 2 <= ADC_UART_BUFFER_SIZE) {
			dev->adcSamples[ch_id][dev->dataPtr[ch_id]++] = (uint8_t)(code16 >> 8);
			dev->adcSamples[ch_id][dev->dataPtr[ch_id]++] = (uint8_t)(code16 & 0xFF);
		}
		captured_mask |= ch_bit;
		got_count++;
	}

	return (got_count == expected) ? HAL_OK : HAL_TIMEOUT;
}

int optics_getDeviceCount(void) {
    return (int)OpticsCount;
}

HAL_StatusTypeDef optics_adcStart(uint32_t mask)
{
	HAL_StatusTypeDef status = HAL_OK;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

	/* Only start devices that aren't already active */
	uint32_t new_mask = mask & ~active_optics_mask;

	/* Track which devices have been started this call to avoid duplicate CONV_STARTs */
	uint32_t started_devices = 0;

    for (uint8_t bit = 0; bit < 32; ++bit) {
        if ((new_mask & (1u << bit)) == 0) {
            continue; /* this bit not set */
        }

        uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

        if (optic_index >= OpticsCount) {
            /* mark error but continue processing other bits */
            status = HAL_ERROR;
            break;
        }

        /* Skip device if already started during this call */
        if (started_devices & (1u << optic_index)) {
            continue;
        }

        OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	    if(dev->enOneshot) // start first conversion
	    {
	    	if(MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START) != HAL_OK)
	    	{
	    		status = HAL_ERROR;
	    	}
	    }
        started_devices |= (1u << optic_index);
    }

	/* update active mask with newly started devices */
    active_optics_mask |= mask;

	return status;
}

HAL_StatusTypeDef optics_adcStop(uint32_t mask)
{
	HAL_StatusTypeDef status = HAL_OK;
    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

	/* Only stop devices that are currently active */
	uint32_t devices_to_stop = mask & active_optics_mask;

	/* Remove stopped devices from active mask */
    active_optics_mask &= ~devices_to_stop;

	// stop all devices when mask is zero
    if(active_optics_mask == 0)
    {
    	// all stopped
        for(int x=0; x<OpticsCount; x++)
        {

            OpticsDevice* dev = (OpticsDevice*)&hw->map[x];
            if(MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_STANDBY) != HAL_OK)
            {
                status = HAL_ERROR;
            }
        }
    }

	return status;
}

uint32_t optics_get_active_optics_mask()
{
	return active_optics_mask;
}

HAL_StatusTypeDef optics_adcRead()
{
	HAL_StatusTypeDef status = HAL_OK;

	if ((!hw) || (!hw->map)) {
		return HAL_ERROR;
	}

	if (active_optics_mask == 0) {
		return HAL_OK; /* nothing to read */
	}

	/* Per interrupt: read ALL enabled channels for each active device, then restart
	 * the first channel's conversion so it is ready on the next interrupt.
	 *
	 * first_ch result:      already ready (conversion was started at end of previous
	 *                       interrupt, ~250 ms ago) — read immediately, no wait.
	 * subsequent channels:  switch MUX, start conversion, wait ~1.5 ms, read. */
	uint32_t handled_devices = 0;

	for (uint8_t bit = 0; bit < 32; ++bit) {
		if ((active_optics_mask & (1u << bit)) == 0) continue;

		uint8_t optic_index = (uint8_t)(bit >> 3);
		if (optic_index >= OpticsCount) {
			status = HAL_ERROR;
			continue;
		}
		if (handled_devices & (1u << optic_index)) continue;
		handled_devices |= (1u << optic_index);

		OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
		uint8_t enabled_ch_mask = (uint8_t)((active_optics_mask >> (optic_index * 8)) & 0xFF);

		if (dev->channel_count == 0) continue;

		/* Find first enabled channel — its result is already waiting in the ADC. */
		uint8_t first_ch = 0xFF;
		for (uint8_t i = 0; i < dev->channel_count; i++) {
			if (enabled_ch_mask & (1u << i)) { first_ch = i; break; }
		}
		if (first_ch == 0xFF) continue;

		/* Read every enabled channel in order. */
		uint8_t data[2];
		HAL_StatusTypeDef st;

		for (uint8_t ch = 0; ch < dev->channel_count; ch++) {
			if (!(enabled_ch_mask & (1u << ch))) continue;

			if (ch != first_ch) {
				/* Switch MUX, start conversion, wait for it to complete (~1.2 ms). */
				uint8_t mux = mux_bytes[optic_index][ch];
				MCP3462_WriteReg(&dev->adc_handle, MCP3462_REG_MUX, &mux, 1);
				MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
				delay_us(1500);
			}

			data[0] = 0; data[1] = 0;
			st = MCP3462_ReadReg(&dev->adc_handle, MCP3462_REG_ADCDATA, data, 2);
			if (st == HAL_OK) {
				if (dev->dataPtr[ch] + 2 <= ADC_UART_BUFFER_SIZE) {
					dev->adcSamples[ch][dev->dataPtr[ch]++] = data[0];
					dev->adcSamples[ch][dev->dataPtr[ch]++] = data[1];
				}
			} else {
				status = st;
			}
		}

		/* Restart first channel's conversion so its result is ready next interrupt. */
		current_mux_idx[optic_index] = first_ch;
		uint8_t mux = mux_bytes[optic_index][first_ch];
		MCP3462_WriteReg(&dev->adc_handle, MCP3462_REG_MUX, &mux, 1);
		MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
	}

	return status;
}

HAL_StatusTypeDef optics_getBuffer_byMask(uint32_t mask, uint8_t** out_buffer,  uint16_t* out_size)
{
    if((!hw) || (!hw->map) || !out_buffer || !out_size || mask == 0){
        return HAL_ERROR;
    }

    /* require exactly one bit set in mask */
    if ((mask & (mask - 1)) != 0u) return HAL_ERROR;  

	/* Find which bit is set */
	uint8_t bit = 0;
	uint32_t temp_mask = mask;
	while ((temp_mask & 1) == 0) {
		temp_mask >>= 1;
		bit++;
	}

	/* Extract optic_index and channel ID from bit position */
	uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
	uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

	/* Validate that the optic_index and channel are within bounds */
	if (optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS) {
		return HAL_ERROR;
	}

	/* Get the device and return the buffer and size */
	OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	

    *out_buffer = (uint8_t*)&(dev->adcSamples[ch_id]);
    *out_size = dev->dataPtr[ch_id];

	return HAL_OK;
}

HAL_StatusTypeDef optics_clearBuffer_byMask(uint32_t mask)
{
	HAL_StatusTypeDef status = HAL_OK;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

    for (uint8_t bit = 0; bit < 32; ++bit) {
        if ((mask & (1u << bit)) == 0) {
            continue; /* this bit not set */
        }

        uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
        uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

        if (optic_index >= OpticsCount) {
            /* mark error but continue processing other bits */
            status = HAL_ERROR;
            break;
        }

        // printf("Clear OPTICS IDX: %d  CH: %d\r\n", optic_index,  ch_id);
        /* call the provided clear function */
        OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    	memset(&dev->adcSamples[ch_id], 0, ADC_UART_BUFFER_SIZE);
    	dev->dataPtr[ch_id] = 0;
    }

	return status;
}

HAL_StatusTypeDef optics_startLaser_byMask(uint32_t mask, uint16_t power) {
	HAL_StatusTypeDef status = HAL_OK;
	uint16_t set_power_level = 0;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

	if (mask == 0) {
		return HAL_OK; /* nothing to set */
	}

    active_laser_mask |= mask;
    set_power_level = (power > 100) ? 100 : power;

	/* Iterate through all possible bits in the mask parameter (not active_laser_mask) */
	for (uint8_t bit = 0; bit < 32; ++bit) {
		if ((mask & (1u << bit)) == 0) {
			continue; /* this channel not in the requested mask */
		}

		/* Extract optic_index and channel ID from bit position */
		uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
		uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

	    if (optic_index >= OpticsCount) {
	    	status = HAL_ERROR;
	    	continue;
	    }

	    OpticsDevice* dev = &hw->map[optic_index];

	    if (!dev) {
	    	status = HAL_ERROR;
	    	continue;
	    }

	    uint32_t calc = (((uint32_t)set_power_level * 4095) + 50) / 100;
	    dev->dacValue = (uint16_t)calc;

	    status = MCP4922_WriteRaw(&dev->dac_handle,
	    					   (MCP4922_Channel)ch_id,
	                           MCP4922_BUF_OFF,
	                           MCP4922_GAIN_1X,
	                           MCP4922_ACTIVE,
	                           dev->dacValue);
	}

    return status;
}

HAL_StatusTypeDef optics_stopLaser_byMask(uint32_t mask) {
	HAL_StatusTypeDef status = HAL_OK;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }


	if (mask == 0) {
		return HAL_OK; /* nothing to set */
	}

	// Clear bits from the active laser mask
	active_laser_mask &= ~mask;

	/* Iterate through mask */
	for (uint8_t bit = 0; bit < 32; ++bit) {
		if ((mask & (1u << bit)) == 0) {
			continue; /* this channel not active */
		}

		/* Extract optic_index and channel ID from bit position */
		uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
		uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */
	    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];

	    dev->dacValue = 0;


	    status = MCP4922_WriteRaw(&dev->dac_handle,
	    					   (MCP4922_Channel)ch_id,
	                           MCP4922_BUF_OFF,
	                           MCP4922_GAIN_1X,
	                           MCP4922_ACTIVE,
	                           dev->dacValue);
	}
    return status;
}

uint32_t optics_get_active_laser_mask(void)
{
	return active_laser_mask;
}
