/**
 * config.h — Central System Configuration
 * ─────────────────────────────────────────────────────────────
 * All tunable parameters, pin assignments, timing constants,
 * platform macros, shared types, and error codes for the
 * Automatic Accident Detection & Response (AADR) system.
 * ─────────────────────────────────────────────────────────────
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════
   PLATFORM — STM32F10x Standard Peripheral Library (SPL)
   ═══════════════════════════════════════════════════════════ */
#ifndef PLATFORM_STM32
  #define PLATFORM_STM32
#endif
/* #define PLATFORM_ARDUINO */

#ifdef PLATFORM_STM32
    #include "stm32f10x.h"          /* SPL core header for STM32F10x family  */

    /* Tick counter — SysTick-based millisecond counter.
     * Declare the variable here; define it in your main/systick handler.    */
    extern volatile uint32_t g_tick_ms;
    #define GET_TICK_MS()   (g_tick_ms)
    #define DELAY_MS(ms)    do {                                    \
                                uint32_t _start = GET_TICK_MS();    \
                                while ((GET_TICK_MS() - _start) < (ms)); \
                            } while(0)

#elif defined(PLATFORM_ARDUINO)
    #define GET_TICK_MS()   millis()
    #define DELAY_MS(ms)    delay(ms)
#endif

/* ─────────────────────────────────────────────────────────────
   MATH HELPERS
   ───────────────────────────────────────────────────────────── */
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#define RAD_TO_DEG(r)       ((r) * (180.0f / (float)M_PI))
#define DEG_TO_RAD(d)       ((d) * ((float)M_PI / 180.0f))
#define VECTOR_MAG(x, y, z) sqrtf((x)*(x) + (y)*(y) + (z)*(z))

/* ═══════════════════════════════════════════════════════════
   FIRMWARE IDENTITY
   ═══════════════════════════════════════════════════════════ */
#define FIRMWARE_VERSION        "1.0.0"
#define DEVICE_ID               "AADR-001"

/* ═══════════════════════════════════════════════════════════
   GPS CONFIGURATION
   ═══════════════════════════════════════════════════════════ */
#define GPS_UART_BAUD           9600
#define GPS_BUFFER_SIZE         128
#define GPS_MIN_SATELLITES      3
#define GPS_FIX_TIMEOUT_MS      30000

/* ═══════════════════════════════════════════════════════════
   GSM / SIM800L CONFIGURATION
   ═══════════════════════════════════════════════════════════ */
#define GSM_UART_BAUD           9600
#define GSM_BUFFER_SIZE         256
#define GSM_CMD_TIMEOUT_MS      5000
#define GSM_CALL_TIMEOUT_MS     30000
#define GSM_NETWORK_TIMEOUT_MS  60000
#define MAX_EMERGENCY_CONTACTS  3
#define SMS_RETRY_COUNT         2
#define SMS_RETRY_DELAY_MS      3000
#define CALL_DURATION_MS        30000
#define EMERGENCY_CALL_NUMBER   "911"       /* Change to local emergency number */

/* Emergency contact phone numbers — replace before flashing */
#define EMERGENCY_NUMBER_1      "+10000000000"
#define EMERGENCY_NUMBER_2      "+10000000001"
#define EMERGENCY_NUMBER_3      "+10000000002"

/* Aliases */
#define EMERGENCY_CONTACT_0     EMERGENCY_NUMBER_1
#define EMERGENCY_CONTACT_1     EMERGENCY_NUMBER_2
#define EMERGENCY_CONTACT_2     EMERGENCY_NUMBER_3

/* ═══════════════════════════════════════════════════════════
   MPU-6050 / IMU CONFIGURATION
   ═══════════════════════════════════════════════════════════ */
#define MPU6050_I2C_ADDR         0x68

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CFG     0x1B
#define MPU6050_REG_ACCEL_CFG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define MPU6050_GYRO_FS_SEL      0x08      /* ±500 °/s  */
#define MPU6050_ACCEL_FS_SEL     0x10      /* ±8 g      */

#define MPU6050_GYRO_SCALE       65.5f     /* LSB/(°/s) for ±500 °/s */
#define MPU6050_ACCEL_SCALE      4096.0f   /* LSB/g    for ±8 g      */

#define IMU_SAMPLE_PERIOD_MS     10        /* 100 Hz */
#define IMU_MOVING_AVG_SIZE      5

/* ═══════════════════════════════════════════════════════════
   CRASH / ROLLOVER THRESHOLDS
   ═══════════════════════════════════════════════════════════ */
#define CRASH_GFORCE_THRESHOLD   4.0f
#define CRASH_GFORCE_SEVERE      8.0f
#define ROLLOVER_ANGLE_DEG       60.0f
#define ROLLOVER_GYRO_DPS        100.0f

/* ═══════════════════════════════════════════════════════════
   ALCOHOL SENSOR (MQ-3) CONFIGURATION
   ═══════════════════════════════════════════════════════════ */
#define MQ3_ANALOG_PIN           0
#define MQ3_WARMUP_MS            30000

#define MQ3_SAMPLES              10
#define MQ3_SAMPLE_COUNT         MQ3_SAMPLES
#define MQ3_SAMPLE_DELAY_MS      50

#define MQ3_R0_KOHM              10.0f
#define MQ3_RL_KOHM              10.0f
#define MQ3_VCC                  3.3f           /* STM32 ADC reference = 3.3V */

#define MQ3_ADC_RESOLUTION       4095.0f        /* STM32 12-bit ADC */
#define MQ3_ADC_MAX              MQ3_ADC_RESOLUTION

#define MQ3_VOLTAGE_THRESHOLD    1.4f
#define MQ3_RS_R0_SOBER          1.6f
#define MQ3_RS_R0_LOW            1.0f
#define MQ3_RS_R0_MEDIUM         0.6f

/* ═══════════════════════════════════════════════════════════
   ALERT / CANCEL WINDOW
   ═══════════════════════════════════════════════════════════ */
#define ALERT_DELAY_MS           7000
#define ALERT_CANCEL_WINDOW_MS   ALERT_DELAY_MS
#define ALERT_BEEP_INTERVAL_MS   500

/* ═══════════════════════════════════════════════════════════
   ERROR CODES
   ═══════════════════════════════════════════════════════════ */
typedef enum {
    ERR_NONE            = 0x00,
    ERR_IMU_INIT_FAIL   = 0x01,
    ERR_GSM_INIT_FAIL   = 0x02,
    ERR_SMS_SEND_FAIL   = 0x03,
    ERR_ADC_FAIL        = 0x04,
    ERR_GPS_TIMEOUT     = 0x05,
    ERR_GPS_NO_FIX      = 0x06,
} ErrorCode_t;

/* ═══════════════════════════════════════════════════════════
   SYSTEM STATE MACHINE
   ═══════════════════════════════════════════════════════════ */
typedef enum {
    STATE_INIT          = 0,
    STATE_WARMUP,
    STATE_ALCOHOL_TEST,
    STATE_MONITORING,
    STATE_ALERT_PENDING,
    STATE_EMERGENCY,
    STATE_LOCKOUT,
    STATE_ERROR,
} SystemState_t;

/* ═══════════════════════════════════════════════════════════
   AGGREGATE SYSTEM DATA STRUCTURE
   ═══════════════════════════════════════════════════════════ */
#include "gps.h"
#include "mpu6050.h"
#include "alcohol.h"

typedef struct {
    SystemState_t   state;
    ErrorCode_t     error_code;

    GpsData_t       gps;
    ImuData_t       imu;
    AlcoholData_t   alcohol;

    bool            crash_detected;
    bool            rollover_detected;
    float           peak_g_force;
    uint32_t        crash_timestamp;

    bool            engine_allowed;
    bool            emergency_sent;
} SystemData_t;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
