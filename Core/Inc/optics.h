#ifndef INC_OPTICS_H_
#define INC_OPTICS_H_

#include "main.h"
#include "mcp3462.h"
#include "mcp4922.h"
#include "stdint.h"
#include "stdbool.h"

#define MAX_DAC_CHANNELS			2   // max DAC channels
#define MAX_ADC_CHANNELS			8   // max ADC channels
#define ADC_UART_BUFFER_SIZE		256   // max 128 samples before rollover 128 * 2 bytes per sample
#define OPTICS_TIM_PRESCALER		8999     // 180MHz / 9000 = 20kHz timer clock
#define OPTICS_CAPTURE_FREQUENCY	5000     // 20kHz / 5000 = 250ms per full scan

/* Per-device channel selection.
 *
 * `channels[i]` is one MCP3462_ScanBits value naming the source to convert
 * (single-ended CH0..CH3, differential pair, internal sources, ...).
 * The array index `i` is the logical channel — it is the bit position within
 * the device's 8-bit lane of the host-side mask used by the *_byMask APIs and
 * the index into adcSamples[]/dataPtr[].
 *
 * Example: { MCP3462_SCAN_DIFF_A, MCP3462_SCAN_DIFF_B }, channel_count = 2
 *          -> host mask 0x03 streams both differentials.
 *          { MCP3462_SCAN_CH0_SE, MCP3462_SCAN_CH2_SE }, channel_count = 2
 *          -> host mask 0x03 streams CH0_SE on logical 0, CH2_SE on logical 1.
 */
typedef struct {
    MCP4922_Handle dac_handle;
    MCP3462_Handle adc_handle;

    uint8_t adcSamples[MAX_ADC_CHANNELS][ADC_UART_BUFFER_SIZE];
	uint16_t dataPtr[MAX_ADC_CHANNELS];

	bool enOneshot;

    MCP3462_ScanBits channels[MAX_ADC_CHANNELS];
    uint8_t          channel_count;

    uint16_t dacValue;
} OpticsDevice;

typedef struct {
    uint8_t count;
    OpticsDevice* map;
} OpticsHwDesc;

HAL_StatusTypeDef optics_init();
HAL_StatusTypeDef optics_startLaser(int optic_index, uint16_t power);
HAL_StatusTypeDef optics_stopLaser(int optic_index);
HAL_StatusTypeDef optics_adcReadSamples(int optic_index);
HAL_StatusTypeDef optics_adcStartConversion(int optic_index);
void optics_clearBuffer(int optic_index, uint8_t ch_id);
void optics_clearBuffers(int optic_index);

uint32_t optics_get_active_optics_mask(void);
uint32_t optics_get_active_laser_mask(void);
HAL_StatusTypeDef optics_startLaser_byMask(uint32_t mask, uint16_t power);
HAL_StatusTypeDef optics_stopLaser_byMask(uint32_t mask);
HAL_StatusTypeDef optics_adcStart(uint32_t mask);
HAL_StatusTypeDef optics_adcStop(uint32_t mask);
HAL_StatusTypeDef optics_adcRead();
HAL_StatusTypeDef optics_getBuffer_byMask(uint32_t mask, uint8_t** out_buffer,  uint16_t* out_size);
HAL_StatusTypeDef optics_clearBuffer_byMask(uint32_t mask);

uint8_t* optics_getBuffer(int optic_index, uint8_t ch_id);
uint16_t optics_getSize(int optic_index, uint8_t ch_id);
int optics_getDeviceCount(void);

#endif /* INC_OPTICS_H_ */
