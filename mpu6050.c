/**
 * mpu6050.c — MPU-6050 IMU Driver
 * ─────────────────────────────────────────────────────────────
 * Handles I2C communication with the MPU-6050 6-axis IMU.
 * Provides:
 *   - Initialization and self-test
 *   - Raw data reading (accelerometer + gyroscope)
 *   - Scaled float conversion (g-force, °/s)
 *   - Roll/pitch angle computation (complementary filter)
 *   - Crash detection (high-G and rollover)
 * ─────────────────────────────────────────────────────────────
 */

#include "mpu6050.h"
#include "config.h"

/* ─── Private state ─────────────────────────────────────────── */
static float   s_gyro_roll  = 0.0f;
static float   s_gyro_pitch = 0.0f;
static uint32_t s_last_time = 0;

/* Moving average filter buffers */
static float s_ax_buf[IMU_MOVING_AVG_SIZE] = {0};
static float s_ay_buf[IMU_MOVING_AVG_SIZE] = {0};
static float s_az_buf[IMU_MOVING_AVG_SIZE] = {0};
static uint8_t s_avg_idx = 0;

/* ─────────────────────────────────────────────────────────────
   HAL I2C Wrappers  (replace with your platform's I2C calls)
   ─────────────────────────────────────────────────────────────
   STM32 HAL:  HAL_I2C_Mem_Write / HAL_I2C_Mem_Read
   Arduino:    Wire.beginTransmission / Wire.write / Wire.endTransmission
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Write one byte to an MPU-6050 register over I2C.
 * @param  reg   Register address
 * @param  data  Byte to write
 * @return true on success
 */
static bool mpu6050_write_reg(uint8_t reg, uint8_t data)
{
#ifdef PLATFORM_STM32
    /* SPL: I2C1 blocking write — START, addr+W, reg, data, STOP */
    uint32_t timeout;

    /* START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && --timeout);
    if (!timeout) return false;

    /* Address + Write */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Transmitter);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --timeout);
    if (!timeout) { I2C_GenerateSTOP(I2C1, ENABLE); return false; }

    /* Register address */
    I2C_SendData(I2C1, reg);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --timeout);

    /* Data byte */
    I2C_SendData(I2C1, data);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --timeout);

    /* STOP */
    I2C_GenerateSTOP(I2C1, ENABLE);
    return true;
#else
    /* Arduino Wire implementation */
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
#endif
}

/**
 * @brief  Read N bytes from MPU-6050 starting at reg over I2C.
 * @param  reg    Start register address
 * @param  buf    Output buffer
 * @param  len    Number of bytes to read
 * @return true on success
 */
static bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
#ifdef PLATFORM_STM32
    /* SPL: I2C1 blocking read — START, addr+W, reg, repeated START, addr+R, read N bytes, STOP */
    uint32_t timeout;

    /* START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && --timeout);
    if (!timeout) return false;

    /* Address + Write (send register pointer) */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Transmitter);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --timeout);
    if (!timeout) { I2C_GenerateSTOP(I2C1, ENABLE); return false; }

    I2C_SendData(I2C1, reg);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && --timeout);

    /* Repeated START */
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && --timeout);

    /* Address + Read */
    I2C_Send7bitAddress(I2C1, MPU6050_I2C_ADDR << 1, I2C_Direction_Receiver);
    timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --timeout);

    for (uint8_t i = 0; i < len; i++) {
        if (i == (len - 1)) {
            I2C_AcknowledgeConfig(I2C1, DISABLE);   /* NACK on last byte */
            I2C_GenerateSTOP(I2C1, ENABLE);
        }
        timeout = 10000; while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) && --timeout);
        buf[i] = I2C_ReceiveData(I2C1);
    }
    I2C_AcknowledgeConfig(I2C1, ENABLE);   /* Re-enable ACK for next transaction */
    return true;
#else
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
    return true;
#endif
}

/* ─────────────────────────────────────────────────────────────
   PUBLIC FUNCTIONS
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize the MPU-6050 sensor.
 *         Sets full-scale ranges, sample rate, enables clock.
 * @return true on success, false if sensor not found
 */
bool mpu6050_init(void)
{
    uint8_t who_am_i = 0;

    /* 1. Verify sensor identity */
    if (!mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1)) {
        return false;
    }
    if (who_am_i != 0x68) {         /* MPU-6050 WHO_AM_I = 0x68 */
        return false;
    }

    /* 2. Wake up sensor — clear sleep bit, use PLL with X-gyro as clock */
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01)) return false;
    DELAY_MS(100);

    /* 3. Configure sample rate divider: Sample Rate = Gyro Rate / (1 + SMPLRT_DIV)
     *    With gyro at 1kHz and SMPLRT_DIV=9 → 100 Hz sample rate             */
    if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x09)) return false;

    /* 4. Configure DLPF (Digital Low-Pass Filter) — bandwidth ~44Hz
     *    Reduces noise without significantly affecting impact detection       */
    if (!mpu6050_write_reg(MPU6050_REG_CONFIG, 0x03)) return false;

    /* 5. Gyroscope full-scale range: ±500°/s
     *    Good for detecting vehicle rollover angular velocities               */
    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CFG, MPU6050_GYRO_FS_SEL)) return false;

    /* 6. Accelerometer full-scale range: ±8g
     *    Covers both normal driving (< 1g) and crashes (up to 8g+)           */
    if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CFG, MPU6050_ACCEL_FS_SEL)) return false;

    /* 7. Initialize filter state */
    s_gyro_roll  = 0.0f;
    s_gyro_pitch = 0.0f;
    s_last_time  = GET_TICK_MS();

    return true;
}

/**
 * @brief  Read raw 16-bit accelerometer and gyroscope values from MPU-6050.
 *         Converts to engineering units (g and °/s) and computes angles.
 * @param  data  Output structure to fill
 * @return true on successful read
 */
bool mpu6050_read(ImuData_t *data)
{
    uint8_t raw[14];   /* 6 bytes accel + 2 bytes temp + 6 bytes gyro */

    if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, raw, 14)) {
        return false;
    }

    /* ─── Parse raw 16-bit big-endian values ─── */
    int16_t raw_ax = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t raw_ay = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t raw_az = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6:7] = temperature (unused here) */
    int16_t raw_gx = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t raw_gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t raw_gz = (int16_t)((raw[12] << 8) | raw[13]);

    /* ─── Convert to physical units ─── */
    float ax = (float)raw_ax / MPU6050_ACCEL_SCALE;
    float ay = (float)raw_ay / MPU6050_ACCEL_SCALE;
    float az = (float)raw_az / MPU6050_ACCEL_SCALE;
    float gx = (float)raw_gx / MPU6050_GYRO_SCALE;
    float gy = (float)raw_gy / MPU6050_GYRO_SCALE;
    float gz = (float)raw_gz / MPU6050_GYRO_SCALE;

    /* ─── Moving average filter on accelerometer ─── */
    s_ax_buf[s_avg_idx] = ax;
    s_ay_buf[s_avg_idx] = ay;
    s_az_buf[s_avg_idx] = az;
    s_avg_idx = (s_avg_idx + 1) % IMU_MOVING_AVG_SIZE;

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    for (uint8_t i = 0; i < IMU_MOVING_AVG_SIZE; i++) {
        sum_ax += s_ax_buf[i];
        sum_ay += s_ay_buf[i];
        sum_az += s_az_buf[i];
    }
    ax = sum_ax / IMU_MOVING_AVG_SIZE;
    ay = sum_ay / IMU_MOVING_AVG_SIZE;
    az = sum_az / IMU_MOVING_AVG_SIZE;

    /* ─── Complementary filter for angle estimation ────────────────
     * Combines gyroscope (accurate short-term) with accelerometer
     * (accurate long-term) to get stable angle estimates.
     * Formula: angle = α * (angle + gyro * dt) + (1-α) * accel_angle
     * α = 0.96 means 96% trust in gyro, 4% in accel for correction.
     * ─────────────────────────────────────────────────────────────── */
    uint32_t now     = GET_TICK_MS();
    float    dt      = (float)(now - s_last_time) / 1000.0f;  /* seconds */
    s_last_time      = now;

    /* Angle from accelerometer (static, gravity-based) */
    float accel_roll  = RAD_TO_DEG(atan2f(ay, az));
    float accel_pitch = RAD_TO_DEG(atan2f(-ax, sqrtf(ay*ay + az*az)));

    /* Complementary filter update */
    float alpha       = 0.96f;
    s_gyro_roll       = alpha * (s_gyro_roll  + gx * dt) + (1.0f - alpha) * accel_roll;
    s_gyro_pitch      = alpha * (s_gyro_pitch + gy * dt) + (1.0f - alpha) * accel_pitch;

    /* ─── Compute total G-force vector magnitude ─── */
    /* Subtract 1g from Z (Earth gravity) for true impact force */
    float impact_az   = az - 1.0f;
    float total_g     = VECTOR_MAG(ax, ay, impact_az);

    /* ─── Fill output structure ─── */
    data->ax           = ax;
    data->ay           = ay;
    data->az           = az;
    data->gx           = gx;
    data->gy           = gy;
    data->gz           = gz;
    data->total_g      = total_g;
    data->roll_angle   = s_gyro_roll;
    data->pitch_angle  = s_gyro_pitch;
    data->timestamp_ms = now;

    return true;
}

/**
 * @brief  Analyze IMU data for crash or rollover conditions.
 * @param  data     Current IMU reading
 * @param  peak_g   Pointer to running peak G-force (updated here)
 * @return CrashType_t indicating what (if anything) was detected
 */
CrashType_t mpu6050_detect_event(const ImuData_t *data, float *peak_g)
{
    /* Update running peak */
    if (data->total_g > *peak_g) {
        *peak_g = data->total_g;
    }

    /* ─── ROLLOVER DETECTION ─── */
    /* Check 1: Static roll angle > threshold (tilted/on side) */
    float abs_roll = fabsf(data->roll_angle);
    if (abs_roll > ROLLOVER_ANGLE_DEG) {
        return CRASH_ROLLOVER;
    }

    /* Check 2: Dynamic roll rate > threshold (actively rolling) */
    float abs_gx = fabsf(data->gx);
    if (abs_gx > ROLLOVER_GYRO_DPS) {
        return CRASH_ROLLOVER;
    }

    /* ─── HIGH-G IMPACT DETECTION ─── */
    /* Severe crash: no delay, immediate response */
    if (data->total_g > CRASH_GFORCE_SEVERE) {
        return CRASH_SEVERE;
    }

    /* Moderate crash: alert with cancel window */
    if (data->total_g > CRASH_GFORCE_THRESHOLD) {
        return CRASH_MODERATE;
    }

    return CRASH_NONE;
}

/**
 * @brief  Reset the complementary filter state.
 *         Call when vehicle is stationary (parked/restarted).
 */
void mpu6050_reset_filter(void)
{
    s_gyro_roll  = 0.0f;
    s_gyro_pitch = 0.0f;
    s_last_time  = GET_TICK_MS();

    for (uint8_t i = 0; i < IMU_MOVING_AVG_SIZE; i++) {
        s_ax_buf[i] = 0.0f;
        s_ay_buf[i] = 0.0f;
        s_az_buf[i] = 1.0f;   /* Rest position: 1g on Z axis */
    }
}
