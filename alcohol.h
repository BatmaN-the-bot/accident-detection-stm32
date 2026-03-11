/**
 * alcohol.h — MQ-3 Alcohol Sensor Public Interface
 * ─────────────────────────────────────────────────────────────
 * Reads the MQ-3 gas sensor via ADC and converts the voltage to
 * an Rs/R0 ratio used to estimate blood-alcohol level (BAC).
 * ─────────────────────────────────────────────────────────────
 */

#ifndef ALCOHOL_H
#define ALCOHOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════
   BAC LEVEL ENUMERATION
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Qualitative BAC classification derived from Rs/R0 ratio.
 *         Thresholds defined in config.h (MQ3_RS_R0_*).
 */
typedef enum {
    BAC_SOBER  = 0,     /**< Rs/R0 above sober threshold — no alcohol detected */
    BAC_LOW    = 1,     /**< Low alcohol presence                              */
    BAC_MEDIUM = 2,     /**< Moderate intoxication                             */
    BAC_HIGH   = 3,     /**< High intoxication                                 */
} BacLevel_t;

/* ═══════════════════════════════════════════════════════════
   ALCOHOL DATA STRUCTURE
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Alcohol sensor reading including raw and derived values.
 *         Populated by alcohol_read().
 */
struct AlcoholData {
    uint16_t    raw_adc;            /**< Averaged raw ADC counts                   */
    float       voltage;            /**< Averaged ADC voltage (V)                  */
    float       rs_r0_ratio;        /**< Sensor resistance ratio Rs/R0             */
    BacLevel_t  bac_level;          /**< Qualitative BAC classification            */
    bool        alcohol_detected;   /**< true if any alcohol above sober threshold */
};

/* Single typedef — do NOT redefine in alcohol.c */
typedef struct AlcoholData AlcoholData_t;

/* ═══════════════════════════════════════════════════════════
   WARM-UP CALLBACK TYPE
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Progress callback invoked during sensor warm-up.
 * @param  pct  Warm-up completion percentage 0–100.
 */
typedef void (*AlcoholWarmupCb_t)(uint8_t pct);

/* ═══════════════════════════════════════════════════════════
   PUBLIC API
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Block for MQ3_WARMUP_MS while the sensor heats up.
 *         Fires the optional progress callback every ~1 % of warmup.
 *
 * @param  progress_cb  Callback for UI updates; may be NULL.
 */
void alcohol_sensor_warmup(AlcoholWarmupCb_t progress_cb);

/**
 * @brief  Take MQ3_SAMPLES ADC readings, average them, and compute
 *         the Rs/R0 ratio and BAC level.
 *
 * @param  out  Output structure to populate.
 * @return true  — ADC read succeeded and *out is valid.
 * @return false — ADC hardware error.
 */
bool alcohol_read(AlcoholData_t *out);

/**
 * @brief  Return a human-readable string for a BAC level.
 *         Useful for debug UART output.
 *
 * @param  level  BacLevel_t value.
 * @return Null-terminated constant string, e.g. "SOBER" or "HIGH".
 */
const char *alcohol_level_str(BacLevel_t level);

#ifdef __cplusplus
}
#endif

#endif /* ALCOHOL_H */
