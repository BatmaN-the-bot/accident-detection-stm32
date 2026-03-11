/**
 * main.c — Automatic Accident Detection, Prevention & Emergency Response
 * ═══════════════════════════════════════════════════════════════════════
 *
 * SYSTEM OVERVIEW:
 * ────────────────
 * This firmware implements a multi-stage vehicle safety system:
 *
 *  Stage 1 — PREVENTION (Drunk Driving Block)
 *    • MQ-3 alcohol sensor checked before engine start
 *    • Engine relay held LOW if alcohol detected
 *    • Periodic re-checks if driver insists
 *
 *  Stage 2 — DETECTION (Crash/Rollover Monitoring)
 *    • MPU-6050 sampled at 100Hz while driving
 *    • Detects: high-G impact (> 4g) and rollover (> 60° or > 100°/s)
 *    • Moving average filter eliminates road bump false positives
 *
 *  Stage 3 — VERIFICATION (Cancel Window)
 *    • 7-second countdown with buzzer pattern after detection
 *    • User presses reset button to cancel false alarm
 *    • Severe crash (> 8g) or rollover skips cancel window
 *
 *  Stage 4 — RESPONSE (Emergency Dispatch)
 *    • GPS fix obtained (or last-known position used)
 *    • SMS with coordinates sent to all emergency contacts
 *    • Automated voice call to emergency services
 *
 * STATE MACHINE:
 * ──────────────
 *  INIT → WARMUP → ALCOHOL_TEST → MONITORING → ALERT_PENDING → EMERGENCY
 *                        ↓                                           ↓
 *                    LOCKOUT                                    (loop back)
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#include "config.h"
#include "mpu6050.h"
#include "gps.h"
#include "gsm.h"
#include "alcohol.h"

/* ═══════════════════════════════════════════════════════════
   SPL HARDWARE INITIALISATION
   Clocks, GPIO, I2C, USART3 (debug) — called first in aadr_main()
   ═══════════════════════════════════════════════════════════ */
#ifdef PLATFORM_STM32
static void hardware_init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;
    I2C_InitTypeDef   i2c;

    /* ── Enable peripheral clocks ── */
    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_GPIOB |
        RCC_APB2Periph_GPIOC |
        RCC_APB2Periph_USART1 |     /* GPS  */
        RCC_APB2Periph_ADC1   |     /* MQ-3 */
        RCC_APB2Periph_AFIO,
        ENABLE
    );
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_USART2 |     /* GSM   */
        RCC_APB1Periph_USART3 |     /* Debug — USART3 is on APB1, not APB2 */
        RCC_APB1Periph_I2C1,        /* MPU-6050 */
        ENABLE
    );

    /* ── GPIOB Pin 1 — Engine relay (output push-pull) ── */
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);

    /* ── GPIOB Pin 0 — Cancel button (input pull-up) ── */
    gpio.GPIO_Pin  = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    /* ── GPIOA Pin 8 — Buzzer (output push-pull) ── */
    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    /* ── GPIOC Pin 13 — Status LED (output push-pull, active LOW) ── */
    gpio.GPIO_Pin   = GPIO_Pin_13;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);

    /* ── GPIOA Pin 0 — MQ-3 ADC input (analog) ── */
    gpio.GPIO_Pin  = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    /* ── USART3 (debug) — PB10=TX, PB11=RX ── */
    gpio.GPIO_Pin   = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin  = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    uart.USART_BaudRate            = 115200;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Tx;
    USART_Init(USART3, &uart);
    USART_Cmd(USART3, ENABLE);

    /* ── ADC1 — single conversion, channel 0 (PA0 = MQ-3) ── */
    ADC_InitTypeDef adc;
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    ADC_Cmd(ADC1, ENABLE);
    /* Calibrate ADC */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    /* ── I2C1 — PB6=SCL, PB7=SDA (MPU-6050) ── */
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &gpio);

    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed          = 400000;   /* 400 kHz Fast Mode */
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);

    /* ── SysTick — 1ms tick for GET_TICK_MS() ── */
    SysTick_Config(SystemCoreClock / 1000);
}
#endif  /* PLATFORM_STM32 */


/* SysTick millisecond counter — incremented every 1ms by SysTick_Handler */
volatile uint32_t g_tick_ms = 0;

#ifdef PLATFORM_STM32
void SysTick_Handler(void) { g_tick_ms++; }
#endif

static SystemData_t g_sys;

/* ═══════════════════════════════════════════════════════════
   PERIPHERAL CONTROL WRAPPERS
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Control the engine ignition relay.
 *         HIGH = relay energized = engine START circuit connected.
 */
static void engine_relay_set(bool enabled)
{
#ifdef PLATFORM_STM32
    /* SPL: GPIOB Pin 1 = Relay control */
    GPIO_WriteBit(GPIOB, GPIO_Pin_1, enabled ? Bit_SET : Bit_RESET);
#else
    digitalWrite(7, enabled ? HIGH : LOW);   /* Arduino: relay on pin 7 */
#endif
    g_sys.engine_allowed = enabled;
}

/**
 * @brief  Read the cancel/reset button state.
 * @return true if button is pressed (active LOW with pull-up)
 */
static bool button_is_pressed(void)
{
#ifdef PLATFORM_STM32
    /* SPL: GPIOB Pin 0 = Reset button (active LOW with pull-up) */
    return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_RESET);
#else
    return (digitalRead(6) == LOW);   /* Arduino: button on pin 6 */
#endif
}

/**
 * @brief  Set buzzer state (on/off for simple beep pattern).
 */
static void buzzer_set(bool on)
{
#ifdef PLATFORM_STM32
    GPIO_WriteBit(GPIOA, GPIO_Pin_8, on ? Bit_SET : Bit_RESET);
#else
    digitalWrite(8, on ? HIGH : LOW);
#endif
}

/**
 * @brief  Set status LED state.
 */
static void led_set(bool on)
{
#ifdef PLATFORM_STM32
    /* PC13 is active LOW on Blue Pill */
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, on ? Bit_RESET : Bit_SET);
#else
    digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#endif
}

/**
 * @brief  Debug/status UART print (USART3 on STM32, Serial on Arduino).
 */
static void debug_print(const char *msg)
{
#ifdef PLATFORM_STM32
    /* SPL: USART3 for debug output */
    while (*msg) {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
        USART_SendData(USART3, (uint16_t)(*msg++));
    }
#else
    Serial.print(msg);
#endif
}

/* ═══════════════════════════════════════════════════════════
   SYSTEM INITIALIZATION
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Initialize all peripherals and modules.
 * @return true if all critical systems initialized successfully
 */
static bool system_init(void)
{
    bool ok = true;

    /* ── Initialize GPIO outputs ── */
    engine_relay_set(false);    /* Engine locked until sobriety check */
    buzzer_set(false);
    led_set(true);              /* LED on = system starting up */

    debug_print("\r\n=== AADR System v");
    debug_print(FIRMWARE_VERSION);
    debug_print(" ===\r\n");
    debug_print("Device ID: ");
    debug_print(DEVICE_ID);
    debug_print("\r\nInitializing...\r\n");

    /* ── GPS module ── */
    debug_print("[GPS] Initializing...\r\n");
    gps_init();
    debug_print("[GPS] OK\r\n");

    /* ── GSM module ── */
    debug_print("[GSM] Initializing...\r\n");
    if (!gsm_init()) {
        debug_print("[GSM] WARNING: Init failed or no network\r\n");
        g_sys.error_code = ERR_GSM_INIT_FAIL;
        /* Non-fatal: continue but SMS may fail */
    } else {
        debug_print("[GSM] OK - Network registered\r\n");
    }

    /* ── MPU-6050 IMU ── */
    debug_print("[IMU] Initializing MPU-6050...\r\n");
    if (!mpu6050_init()) {
        debug_print("[IMU] ERROR: MPU-6050 not found!\r\n");
        g_sys.error_code = ERR_IMU_INIT_FAIL;
        ok = false;    /* IMU is critical — cannot detect crashes without it */
    } else {
        debug_print("[IMU] OK\r\n");
    }

    return ok;
}

/* ═══════════════════════════════════════════════════════════
   STATE: WARMUP
   Warm up alcohol sensor while showing countdown
   ═══════════════════════════════════════════════════════════ */

static void warmup_progress_cb(uint8_t pct)
{
    static uint8_t last_pct = 0;
    if (pct != last_pct) {
        char buf[40];
        snprintf(buf, sizeof(buf), "[MQ3] Warmup: %d%%\r\n", pct);
        debug_print(buf);
        last_pct = pct;
    }
    /* Fast blink LED during warmup */
    led_set((GET_TICK_MS() / 500) % 2);
}

/* ═══════════════════════════════════════════════════════════
   STATE: ALCOHOL_TEST
   Check if driver is sober before allowing engine start
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Run the pre-ignition alcohol test.
 *         Blocks until sober reading or locks out permanently.
 * @return true if sober (engine may start), false if drunk (locked out)
 */
static bool run_alcohol_test(void)
{
    AlcoholData_t result;
    uint8_t attempts = 0;
    const uint8_t MAX_ATTEMPTS = 3;

    while (attempts < MAX_ATTEMPTS) {
        debug_print("[ALC] Testing... please breathe normally near sensor\r\n");

        /* Alert beep: one short beep before test */
        buzzer_set(true);  DELAY_MS(200);  buzzer_set(false);
        DELAY_MS(500);

        if (!alcohol_read(&result)) {
            debug_print("[ALC] ADC read failed\r\n");
            g_sys.error_code = ERR_ADC_FAIL;
            return false;
        }

        g_sys.alcohol = result;

        char buf[80];
        snprintf(buf, sizeof(buf),
                 "[ALC] Voltage=%.3fV | Rs/R0=%.2f | Level=%s\r\n",
                 result.voltage, result.rs_r0_ratio,
                 alcohol_level_str(result.bac_level));
        debug_print(buf);

        if (!result.alcohol_detected) {
            /* Sober — allow engine */
            buzzer_set(true);  DELAY_MS(100);
            buzzer_set(false); DELAY_MS(100);
            buzzer_set(true);  DELAY_MS(100);
            buzzer_set(false);
            debug_print("[ALC] SOBER - Engine unlocked\r\n");
            return true;
        }

        /* Alcohol detected */
        attempts++;
        debug_print("[ALC] ALCOHOL DETECTED! Attempt ");

        char num[4];
        snprintf(num, sizeof(num), "%d", attempts);
        debug_print(num);
        debug_print("/");
        snprintf(num, sizeof(num), "%d", MAX_ATTEMPTS);
        debug_print(num);
        debug_print("\r\n");

        /* Long alarm beep */
        for (uint8_t i = 0; i < 5; i++) {
            buzzer_set(true);  DELAY_MS(500);
            buzzer_set(false); DELAY_MS(300);
        }

        if (attempts < MAX_ATTEMPTS) {
            debug_print("[ALC] Waiting 30s before next test...\r\n");
            DELAY_MS(30000);    /* Wait 30s before re-test */
        }
    }

    /* All attempts failed — lock engine */
    debug_print("[ALC] MAX ATTEMPTS EXCEEDED - ENGINE LOCKED\r\n");
    g_sys.state = STATE_LOCKOUT;
    return false;
}

/* ═══════════════════════════════════════════════════════════
   STATE: ALERT_PENDING
   Crash detected — 7-second cancel window
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Run the post-crash alert countdown.
 *         Beeps urgently for ALERT_DELAY_MS milliseconds.
 *         User can cancel by pressing reset button.
 * @param  skip_delay  If true, skip the delay (severe crash)
 * @return true if alert should proceed (no cancel), false if cancelled
 */
static bool run_alert_countdown(bool skip_delay)
{
    if (skip_delay) {
        debug_print("[ALERT] SEVERE CRASH - Immediate response, no delay\r\n");
        return true;
    }

    debug_print("[ALERT] Crash detected! Press RESET within 7 seconds to cancel\r\n");

    uint32_t start        = GET_TICK_MS();
    uint32_t beep_time    = 0;
    bool     beep_on      = false;

    while ((GET_TICK_MS() - start) < ALERT_DELAY_MS) {

        /* Check for cancel button press */
        if (button_is_pressed()) {
            buzzer_set(false);
            debug_print("[ALERT] CANCELLED by user - False alarm\r\n");

            /* Reset peak G reading after cancel */
            g_sys.peak_g_force = 0.0f;
            g_sys.crash_detected = false;
            g_sys.rollover_detected = false;

            /* Confirmation double-beep */
            DELAY_MS(200);
            buzzer_set(true);  DELAY_MS(100);
            buzzer_set(false); DELAY_MS(100);
            buzzer_set(true);  DELAY_MS(100);
            buzzer_set(false);

            return false;    /* Alert cancelled */
        }

        /* Urgent beep pattern: escalating frequency as deadline approaches */
        uint32_t elapsed  = GET_TICK_MS() - start;
        uint32_t remaining = ALERT_DELAY_MS - elapsed;
        uint32_t interval = (remaining > 4000) ? 800 :
                            (remaining > 2000) ? 400 : 200;

        if ((GET_TICK_MS() - beep_time) > interval) {
            beep_on = !beep_on;
            buzzer_set(beep_on);
            beep_time = GET_TICK_MS();

            /* Print countdown every second */
            if (!beep_on) {
                char buf[40];
                snprintf(buf, sizeof(buf), "[ALERT] Sending in %lus...\r\n",
                         (unsigned long)(remaining / 1000));
                debug_print(buf);
            }
        }

        DELAY_MS(10);
    }

    buzzer_set(false);
    return true;    /* Proceed with emergency response */
}

/* ═══════════════════════════════════════════════════════════
   STATE: EMERGENCY
   Send SMS and make emergency call
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Execute the emergency response sequence.
 *         1. Get GPS fix  2. Send SMS  3. Make call
 */
static void run_emergency_response(void)
{
    char buf[128];
    g_sys.state = STATE_EMERGENCY;

    debug_print("\r\n=============================\r\n");
    debug_print("  EMERGENCY RESPONSE ACTIVE  \r\n");
    debug_print("=============================\r\n");

    /* ── 1. Get GPS location ── */
    debug_print("[GPS] Acquiring location...\r\n");

    bool gps_ok = gps_wait_for_fix(&g_sys.gps, GPS_FIX_TIMEOUT_MS);

    if (gps_ok) {
        snprintf(buf, sizeof(buf),
                 "[GPS] Fix OK: %s, %s | Sats: %d | Speed: %.1f km/h\r\n",
                 g_sys.gps.lat_str, g_sys.gps.lon_str,
                 g_sys.gps.satellites, g_sys.gps.speed_kmh);
    } else {
        g_sys.error_code = ERR_GPS_NO_FIX;
        snprintf(buf, sizeof(buf), "[GPS] No fix - using last known position\r\n");
    }
    debug_print(buf);

    /* ── 2. Send emergency SMS ── */
    debug_print("[GSM] Sending emergency SMS to all contacts...\r\n");

    uint8_t sms_count = gsm_send_emergency_sms(
        &g_sys.gps,
        g_sys.peak_g_force,
        g_sys.rollover_detected
    );

    snprintf(buf, sizeof(buf), "[GSM] SMS sent to %d/%d contacts\r\n",
             sms_count, MAX_EMERGENCY_CONTACTS);
    debug_print(buf);

    if (sms_count == 0) {
        g_sys.error_code = ERR_SMS_SEND_FAIL;
        /* Try once more after a pause */
        debug_print("[GSM] Retrying SMS...\r\n");
        DELAY_MS(5000);
        gsm_send_emergency_sms(&g_sys.gps, g_sys.peak_g_force, g_sys.rollover_detected);
    }

    /* ── 3. Make emergency call ── */
    debug_print("[GSM] Initiating emergency call to ");
    debug_print(EMERGENCY_CALL_NUMBER);
    debug_print("...\r\n");

    if (gsm_make_emergency_call()) {
        debug_print("[GSM] Call completed\r\n");
    } else {
        debug_print("[GSM] Call failed\r\n");
    }

    g_sys.emergency_sent = true;

    /* Continuous SOS pattern: 3 short, 3 long, 3 short */
    debug_print("[BUZZER] SOS pattern active\r\n");
    for (uint8_t rep = 0; rep < 3; rep++) {
        /* 3 short */
        for (uint8_t i = 0; i < 3; i++) {
            buzzer_set(true);  DELAY_MS(200);
            buzzer_set(false); DELAY_MS(200);
        }
        DELAY_MS(300);
        /* 3 long */
        for (uint8_t i = 0; i < 3; i++) {
            buzzer_set(true);  DELAY_MS(600);
            buzzer_set(false); DELAY_MS(200);
        }
        DELAY_MS(300);
        /* 3 short */
        for (uint8_t i = 0; i < 3; i++) {
            buzzer_set(true);  DELAY_MS(200);
            buzzer_set(false); DELAY_MS(200);
        }
        DELAY_MS(1000);
    }
}

/* ═══════════════════════════════════════════════════════════
   MAIN MONITORING LOOP
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Main crash monitoring loop.
 *         Samples IMU at ~100Hz, checks for crash/rollover conditions.
 *         Also updates GPS periodically.
 */
static void run_monitoring_loop(void)
{
    uint32_t last_imu_time = 0;
    uint32_t last_gps_time = 0;
    uint32_t last_heartbeat = 0;
    char buf[128];

    debug_print("[MON] Monitoring started. Drive safely.\r\n");

    while (g_sys.state == STATE_MONITORING) {

        uint32_t now = GET_TICK_MS();

        /* ── IMU sampling at 100Hz (every 10ms) ── */
        if ((now - last_imu_time) >= IMU_SAMPLE_PERIOD_MS) {
            last_imu_time = now;

            if (mpu6050_read(&g_sys.imu)) {

                CrashType_t event = mpu6050_detect_event(
                    &g_sys.imu, &g_sys.peak_g_force
                );

                if (event != CRASH_NONE) {
                    g_sys.crash_detected    = (event == CRASH_MODERATE || event == CRASH_SEVERE);
                    g_sys.rollover_detected = (event == CRASH_ROLLOVER);
                    g_sys.crash_timestamp   = now;

                    snprintf(buf, sizeof(buf),
                             "[CRASH] Event=%d | G=%.2f | Roll=%.1f | Pitch=%.1f\r\n",
                             event, g_sys.imu.total_g,
                             g_sys.imu.roll_angle, g_sys.imu.pitch_angle);
                    debug_print(buf);

                    g_sys.state = STATE_ALERT_PENDING;

                    /* Determine if we skip the cancel delay */
                    bool immediate = (event == CRASH_SEVERE || event == CRASH_ROLLOVER);

                    if (run_alert_countdown(immediate)) {
                        run_emergency_response();
                        /* After response, keep monitoring (reset detection flags) */
                        g_sys.crash_detected    = false;
                        g_sys.rollover_detected = false;
                        g_sys.peak_g_force      = 0.0f;
                    }

                    g_sys.state = STATE_MONITORING;
                }
            }
        }

        /* ── GPS update every 5 seconds ── */
        if ((now - last_gps_time) >= 5000) {
            last_gps_time = now;
            gps_update(&g_sys.gps);
        }

        /* ── Heartbeat LED blink every 2 seconds ── */
        if ((now - last_heartbeat) >= 2000) {
            last_heartbeat = now;
            led_set(true);
            DELAY_MS(50);
            led_set(false);
        }

        /* ── Periodic debug status print every 30 seconds ── */
        static uint32_t last_status = 0;
        if ((now - last_status) >= 30000) {
            last_status = now;
            snprintf(buf, sizeof(buf),
                     "[STATUS] G=%.2f | Roll=%.1f | Pitch=%.1f | GPS=%s | Sats=%d\r\n",
                     g_sys.imu.total_g, g_sys.imu.roll_angle, g_sys.imu.pitch_angle,
                     g_sys.gps.valid ? "OK" : "NO FIX", g_sys.gps.satellites);
            debug_print(buf);
        }
    }
}

/* ═══════════════════════════════════════════════════════════
   LOCKOUT STATE
   Engine denied — drunk driver
   ═══════════════════════════════════════════════════════════ */

static void run_lockout_state(void)
{
    debug_print("[LOCK] Engine locked due to alcohol detection\r\n");
    debug_print("[LOCK] Please wait or contact vehicle owner\r\n");

    engine_relay_set(false);

    /* Periodic alarm every 10 seconds to deter bypass attempts */
    uint32_t last_alarm = 0;
    while (1) {   /* Only way out is power cycle / hardware reset */
        uint32_t now = GET_TICK_MS();
        if ((now - last_alarm) >= 10000) {
            last_alarm = now;
            for (uint8_t i = 0; i < 3; i++) {
                buzzer_set(true);  DELAY_MS(300);
                buzzer_set(false); DELAY_MS(200);
            }
        }
        led_set((now / 1000) % 2);   /* Slow LED blink in lockout */
        DELAY_MS(100);
    }
}

/* ═══════════════════════════════════════════════════════════
   MAIN ENTRY POINT
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Main function — system entry point.
 *         For STM32: called after HAL/clock init in main.c stub.
 *         For Arduino: called after setup().
 */
void aadr_main(void)
{
    /* Initialize state */
    memset(&g_sys, 0, sizeof(g_sys));
    g_sys.state = STATE_INIT;

#ifdef PLATFORM_STM32
    hardware_init();   /* Clocks, GPIO, USART, I2C, ADC, SysTick */
#endif

    /* ── STAGE 1: Hardware init ── */
    if (!system_init()) {
        debug_print("[FATAL] System init failed!\r\n");
        g_sys.state = STATE_ERROR;
        /* Flash SOS on LED forever */
        while (1) {
            led_set(true);  DELAY_MS(100);
            led_set(false); DELAY_MS(100);
        }
    }

    /* ── STAGE 2: Sensor warmup ── */
    g_sys.state = STATE_WARMUP;
    debug_print("[WARM] Warming up MQ-3 sensor (30s)...\r\n");
    alcohol_sensor_warmup(warmup_progress_cb);
    debug_print("[WARM] Sensors ready\r\n");

    /* ── STAGE 3: Alcohol test before engine start ── */
    g_sys.state = STATE_ALCOHOL_TEST;
    bool sober = run_alcohol_test();

    if (!sober) {
        run_lockout_state();    /* Never returns */
    }

    /* Allow engine start */
    engine_relay_set(true);
    debug_print("[SYS] Engine unlocked. Starting monitoring.\r\n");

    /* ── STAGE 4: Continuous monitoring ── */
    g_sys.state = STATE_MONITORING;
    mpu6050_reset_filter();    /* Zero out filter after vehicle stationary period */
    run_monitoring_loop();     /* Should run forever */

    /* Should never reach here */
    debug_print("[SYS] ERROR: Monitoring loop exited unexpectedly\r\n");
}

/* ─────────────────────────────────────────────────────────────
   ARDUINO COMPATIBILITY WRAPPERS
   If building for Arduino, these replace aadr_main()
   ───────────────────────────────────────────────────────────── */

/* ─────────────────────────────────────────────────────────────
   STM32 ENTRY POINT
   The C runtime calls main() — we just forward to aadr_main().
   ───────────────────────────────────────────────────────────── */
#ifndef PLATFORM_ARDUINO
int main(void)
{
    aadr_main();
    return 0;   /* Never reached */
}
#endif

#ifdef PLATFORM_ARDUINO
void setup(void)
{
    Serial.begin(115200);      /* Debug UART     */
    Serial1.begin(9600);       /* GPS UART       */
    Serial2.begin(9600);       /* GSM UART       */
    Wire.begin();              /* I2C for MPU    */

    pinMode(A0, INPUT);        /* MQ-3 analog    */
    pinMode(6,  INPUT_PULLUP); /* Reset button   */
    pinMode(7,  OUTPUT);       /* Engine relay   */
    pinMode(8,  OUTPUT);       /* Buzzer         */

    aadr_main();
}

void loop(void)
{
    /* Never reached — aadr_main() loops internally */
}
#endif
