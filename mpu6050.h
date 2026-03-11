/**
 * mpu6050.h — MPU-6050 IMU Driver Public Interface
 * ─────────────────────────────────────────────────────────────
 * Include this header in any module that needs accelerometer,
 * gyroscope, angle, or crash-event data from the MPU-6050.
 * ─────────────────────────────────────────────────────────────
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════
   CRASH EVENT TYPES
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Classification returned by mpu6050_detect_event().
 */
typedef enum {
    CRASH_NONE     = 0,     /**< Normal operation, no event detected         */
    CRASH_MODERATE = 1,     /**< G-force > CRASH_GFORCE_THRESHOLD — with cancel window */
    CRASH_SEVERE   = 2,     /**< G-force > CRASH_GFORCE_SEVERE   — immediate dispatch  */
    CRASH_ROLLOVER = 3,     /**< Roll angle > ROLLOVER_ANGLE_DEG or roll rate > threshold */
} CrashType_t;

/* ═══════════════════════════════════════════════════════════
   IMU DATA STRUCTURE
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Fully-processed IMU sample with angles and event context.
 *         Populated by mpu6050_read() on each successful I2C read.
 */
struct ImuData {
    /* Accelerometer — g-force per axis (moving-average filtered) */
    float       ax;             /**< X-axis acceleration (g)                   */
    float       ay;             /**< Y-axis acceleration (g)                   */
    float       az;             /**< Z-axis acceleration (g, ~1g at rest)      */

    /* Gyroscope — angular velocity (°/s) */
    float       gx;             /**< X-axis rotation rate (°/s)                */
    float       gy;             /**< Y-axis rotation rate (°/s)                */
    float       gz;             /**< Z-axis rotation rate (°/s)                */

    /* Derived values */
    float       total_g;        /**< Vector magnitude of impact G (gravity-subtracted) */
    float       roll_angle;     /**< Complementary-filter roll  angle (°)      */
    float       pitch_angle;    /**< Complementary-filter pitch angle (°)      */

    /* Timestamp of this sample */
    uint32_t    timestamp_ms;   /**< Value of GET_TICK_MS() at read time       */
};

/* Convenience typedef matching config.h forward declaration */
typedef struct ImuData ImuData_t;

/* ═══════════════════════════════════════════════════════════
   PUBLIC API
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Initialize the MPU-6050 sensor over I2C.
 *         Verifies WHO_AM_I, wakes the chip, configures full-scale
 *         ranges, sample rate divider, and DLPF.
 *
 * @return true  — sensor found and configured.
 * @return false — I2C error or unexpected device ID.
 */
bool mpu6050_init(void);

/**
 * @brief  Read one IMU sample from the MPU-6050.
 *         Applies the moving-average filter and complementary filter
 *         internally; results are ready to use directly from *data.
 *
 * @param  data  Output structure to populate.
 * @return true  — read and processing succeeded.
 * @return false — I2C communication error.
 */
bool mpu6050_read(ImuData_t *data);

/**
 * @brief  Analyse an IMU sample for crash or rollover conditions.
 *         Also updates the caller's running peak-G tracker.
 *
 * @param  data    Pointer to a freshly-populated ImuData_t.
 * @param  peak_g  Running maximum G-force (read/write). Updated when
 *                 data->total_g exceeds the current value.
 * @return CrashType_t indicating the severity (or CRASH_NONE).
 */
CrashType_t mpu6050_detect_event(const ImuData_t *data, float *peak_g);

/**
 * @brief  Reset the complementary filter and moving-average buffers.
 *         Call this once after the vehicle has been stationary
 *         (e.g. right before entering the monitoring loop) to clear
 *         any accumulated drift from the initialisation period.
 */
void mpu6050_reset_filter(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
