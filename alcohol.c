/**
 * alcohol.c — MQ-3 Alcohol Sensor Driver
 * ─────────────────────────────────────────────────────────────
 * Reads and interprets alcohol concentration from the MQ-3 sensor.
 * Uses ADC averaging and Rs/R0 ratio for BAC estimation.
 * ─────────────────────────────────────────────────────────────
 */

#include "alcohol.h"
#include "config.h"

/* ─────────────────────────────────────────────────────────────
   PLATFORM ADC WRAPPER
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Read 12-bit ADC value from MQ-3 analog pin.
 */
static uint16_t adc_read_mq3(void)
{
#ifdef PLATFORM_STM32
    /* SPL: trigger ADC1 software conversion and wait for result */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return (uint16_t)ADC_GetConversionValue(ADC1);
#else
    /* Arduino: analogRead returns 0–1023 (10-bit), scale to 12-bit */
    return (uint16_t)(analogRead(A0) << 2);    /* Multiply by 4 to get ~12-bit */
#endif
}

/* ─────────────────────────────────────────────────────────────
   PUBLIC FUNCTIONS
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Warm up the MQ-3 sensor.
 *         MQ-3 requires ~30s warm-up for stable readings.
 *         Call this once at startup, optionally with a progress callback.
 * @param  progress_cb Optional callback(pct 0-100), can be NULL
 */
void alcohol_sensor_warmup(void (*progress_cb)(uint8_t pct))
{
    uint32_t steps = MQ3_WARMUP_MS / 1000;
    for (uint32_t i = 0; i < steps; i++) {
        DELAY_MS(1000);
        if (progress_cb) {
            progress_cb((uint8_t)((i * 100) / steps));
        }
    }
    if (progress_cb) progress_cb(100);
}

/**
 * @brief  Read and process MQ-3 alcohol sensor.
 *         Takes multiple averaged samples and computes BAC level.
 * @param  data  Output structure to fill
 * @return true if reading was successful
 */
bool alcohol_read(AlcoholData_t *data)
{
    uint32_t adc_sum = 0;

    /* Average multiple samples for stability */
    for (uint8_t i = 0; i < MQ3_SAMPLE_COUNT; i++) {
        adc_sum += adc_read_mq3();
        DELAY_MS(MQ3_SAMPLE_DELAY_MS);
    }

    data->raw_adc = (uint16_t)(adc_sum / MQ3_SAMPLE_COUNT);

    /* Convert ADC count to voltage */
    data->voltage = ((float)data->raw_adc / (float)MQ3_ADC_MAX) * MQ3_VCC;

    /*
     * Compute Rs (sensor resistance) from voltage divider:
     * The MQ-3 and load resistor form a voltage divider.
     * Vout = Vcc * RL / (Rs + RL)
     * Rs = RL * (Vcc / Vout - 1)
     * (Values in kΩ)
     */
    float rs_kohm = 0.0f;
    if (data->voltage > 0.01f) {   /* Avoid division by zero */
        rs_kohm = MQ3_RL_KOHM * ((MQ3_VCC / data->voltage) - 1.0f);
    } else {
        rs_kohm = MQ3_RL_KOHM * 99.0f;    /* Sensor open circuit */
    }

    /* Rs/R0 ratio — lower ratio = more alcohol present */
    data->rs_r0_ratio = rs_kohm / MQ3_R0_KOHM;

    /*
     * BAC level classification based on Rs/R0 ratio and voltage:
     * Rs/R0 > 1.5  : Sober (clean air baseline)
     * Rs/R0 0.8–1.5: Warning level (low alcohol)
     * Rs/R0 < 0.8  : Intoxicated (alcohol clearly detected)
     *
     * Equivalent check using voltage threshold is simpler and more
     * reliable for this application, so we use both:
     */
    if (data->voltage >= MQ3_VOLTAGE_THRESHOLD || data->rs_r0_ratio < 0.8f) {
        data->alcohol_detected = true;
        data->bac_level = (data->voltage > (MQ3_VOLTAGE_THRESHOLD * 1.5f)) ? 2 : 1;
    } else {
        data->alcohol_detected = false;
        data->bac_level = 0;
    }

    return true;
}

/**
 * @brief  Returns a human-readable BAC level string.
 */
const char *alcohol_level_str(uint8_t bac_level)
{
    switch (bac_level) {
        case 0:  return "SOBER - OK to drive";
        case 1:  return "WARNING - Low alcohol detected";
        case 2:  return "DRUNK - Engine locked";
        default: return "UNKNOWN";
    }
}
