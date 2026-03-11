/**
 * gsm.c — SIM800L GSM Module Driver
 * ─────────────────────────────────────────────────────────────
 * Handles AT command communication with SIM800L/SIM900A.
 * Provides:
 *   - Module initialization and network registration
 *   - SMS sending to multiple contacts
 *   - Emergency voice call initiation
 *   - Signal strength and status queries
 * ─────────────────────────────────────────────────────────────
 */

#include "gsm.h"
#include "config.h"

/* ─── Private receive buffer ────────────────────────────────── */
static char s_rx_buf[GSM_BUFFER_SIZE];

/* Emergency contact list */
static const char *s_emergency_contacts[MAX_EMERGENCY_CONTACTS] = {
    EMERGENCY_NUMBER_1,
    EMERGENCY_NUMBER_2,
    EMERGENCY_NUMBER_3
};

/* ─────────────────────────────────────────────────────────────
   PLATFORM UART WRAPPERS
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Send a string over GSM UART.
 */
static void gsm_uart_send(const char *str)
{
#ifdef PLATFORM_STM32
    /* SPL: send each byte via USART2 */
    while (*str) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, (uint16_t)(*str++));
    }
#else
    Serial2.print(str);
#endif
}

/**
 * @brief  Send a single byte over GSM UART (for Ctrl+Z, etc).
 */
static void gsm_uart_send_byte(uint8_t byte)
{
#ifdef PLATFORM_STM32
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, (uint16_t)byte);
#else
    Serial2.write(byte);
#endif
}

/**
 * @brief  Read response from GSM UART with timeout.
 * @param  buf       Output buffer
 * @param  buf_size  Max bytes to read
 * @param  timeout   Timeout in ms
 * @return Number of bytes read
 */
static uint16_t gsm_uart_read_response(char *buf, uint16_t buf_size,
                                        uint32_t timeout_ms)
{
    uint16_t idx = 0;
    uint32_t start = GET_TICK_MS();

#ifdef PLATFORM_STM32
    while ((GET_TICK_MS() - start) < timeout_ms && idx < (buf_size - 1)) {
        if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) {
            buf[idx++] = (char)(USART_ReceiveData(USART2) & 0xFF);
            buf[idx] = '\0';
            if (strstr(buf, "OK\r\n")       != NULL) break;
            if (strstr(buf, "ERROR")        != NULL) break;
            if (strstr(buf, ">")            != NULL) break;
            if (strstr(buf, "CONNECT")      != NULL) break;
            if (strstr(buf, "NO DIALTONE")  != NULL) break;
        }
    }
#else
    while ((GET_TICK_MS() - start) < timeout_ms &&
           idx < (buf_size - 1) && Serial2.available()) {
        buf[idx++] = Serial2.read();
        buf[idx] = '\0';
        if (strstr(buf, "OK\r\n")  != NULL) break;
        if (strstr(buf, "ERROR")   != NULL) break;
        if (strstr(buf, ">")       != NULL) break;
        DELAY_MS(2);
    }
#endif

    buf[idx] = '\0';
    return idx;
}

/* ─────────────────────────────────────────────────────────────
   AT COMMAND HELPERS
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Send an AT command and wait for expected response.
 * @param  cmd      AT command string (without \r\n — added automatically)
 * @param  expected Expected response substring (e.g., "OK")
 * @param  timeout  Timeout in milliseconds
 * @return true if expected response found in reply
 */
static bool gsm_send_at(const char *cmd, const char *expected, uint32_t timeout)
{
    /* Flush any pending data */
    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    /* Send command with CR+LF terminator */
    gsm_uart_send(cmd);
    gsm_uart_send("\r\n");

    /* Wait for response */
    gsm_uart_read_response(s_rx_buf, sizeof(s_rx_buf), timeout);

    if (expected == NULL) return true;
    return (strstr(s_rx_buf, expected) != NULL);
}

/* ─────────────────────────────────────────────────────────────
   PUBLIC FUNCTIONS
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize and configure the SIM800L module.
 * @return true on success
 */
bool gsm_init(void)
{
#ifdef PLATFORM_STM32
    /* SPL: configure USART2 at GSM_UART_BAUD, 8N1
     * PA2 = TX, PA3 = RX */
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    uart.USART_BaudRate            = GSM_UART_BAUD;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart);
    USART_Cmd(USART2, ENABLE);
    DELAY_MS(1000);
#endif

    /* 1. Basic AT handshake — try a few times (module may be booting) */
    bool alive = false;
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        if (gsm_send_at("AT", "OK", 1000)) {
            alive = true;
            break;
        }
        DELAY_MS(500);
    }
    if (!alive) return false;

    /* 2. Disable echo (reduces parsing complexity) */
    gsm_send_at("ATE0", "OK", 1000);

    /* 3. Set SMS text mode (vs PDU mode — easier to use) */
    if (!gsm_send_at("AT+CMGF=1", "OK", 1000)) return false;

    /* 4. Set SMS character set to GSM (supports basic ASCII) */
    gsm_send_at("AT+CSCS=\"GSM\"", "OK", 1000);

    /* 5. Set new message indication (optional — for future incoming SMS) */
    gsm_send_at("AT+CNMI=1,2,0,0,0", "OK", 1000);

    /* 6. Wait for network registration */
    uint32_t net_start = GET_TICK_MS();
    bool registered = false;
    while ((GET_TICK_MS() - net_start) < GSM_NETWORK_TIMEOUT_MS) {
        if (gsm_send_at("AT+CREG?", "+CREG: 0,1", 2000) ||   /* Home network */
            gsm_send_at("AT+CREG?", "+CREG: 0,5", 2000)) {   /* Roaming      */
            registered = true;
            break;
        }
        DELAY_MS(2000);
    }

    return registered;
}

/**
 * @brief  Check if GSM module has network signal.
 * @return Signal strength 0–31 (99 = unknown), -1 on error
 */
int8_t gsm_get_signal_strength(void)
{
    if (!gsm_send_at("AT+CSQ", "+CSQ:", 2000)) return -1;

    /* Response format: +CSQ: <rssi>,<ber>
     * RSSI 0 = -113 dBm, 31 = -51 dBm, 99 = unknown                        */
    char *csq = strstr(s_rx_buf, "+CSQ: ");
    if (csq == NULL) csq = strstr(s_rx_buf, "+CSQ:");
    if (csq == NULL) return -1;

    return (int8_t)atoi(csq + 5);
}

/**
 * @brief  Send an SMS message to a phone number.
 * @param  number   Destination phone number (e.g., "+919876543210")
 * @param  message  Message text (max 160 chars for single SMS)
 * @return true on success
 */
bool gsm_send_sms(const char *number, const char *message)
{
    char cmd[64];

    /* Step 1: Set recipient number */
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", number);
    if (!gsm_send_at(cmd, ">", 5000)) {
        return false;
    }

    /* Step 2: Send message body */
    gsm_uart_send(message);

    /* Step 3: Send Ctrl+Z (ASCII 26) to terminate and transmit */
    DELAY_MS(100);
    gsm_uart_send_byte(0x1A);    /* Ctrl+Z = ASCII 26 */

    /* Step 4: Wait for +CMGS confirmation (can take up to 60s) */
    gsm_uart_read_response(s_rx_buf, sizeof(s_rx_buf), 30000);

    return (strstr(s_rx_buf, "+CMGS:") != NULL ||
            strstr(s_rx_buf, "OK") != NULL);
}

/**
 * @brief  Send emergency SMS to all configured contacts.
 *         Builds message with GPS coordinates and crash details.
 * @param  gps       Current GPS data
 * @param  crash_g   Peak G-force detected
 * @param  rollover  true if rollover was detected
 * @return Number of messages successfully sent
 */
uint8_t gsm_send_emergency_sms(const GpsData_t *gps, float crash_g, bool rollover)
{
    char message[160];  /* SMS max length */
    char maps_url[80];
    uint8_t sent = 0;

    /* Build Google Maps URL */
    if (gps->valid) {
        snprintf(maps_url, sizeof(maps_url),
                 "maps.google.com/?q=%s,%s",
                 gps->lat_str, gps->lon_str);
    } else {
        strncpy(maps_url, "GPS unavailable", sizeof(maps_url));
    }

    /* Build SMS message */
    /* Keep under 160 chars for single SMS (no concatenation needed)      */
    snprintf(message, sizeof(message),
             "ACCIDENT ALERT! Vehicle %s\n"
             "%s\n"
             "Location: %s\n"
             "Impact: %.1fg%s\n"
             "Speed before: %.0fkm/h\n"
             "SEND HELP NOW!",
             DEVICE_ID,
             rollover ? "ROLLOVER DETECTED" : "COLLISION DETECTED",
             maps_url,
             crash_g,
             rollover ? " + ROLLOVER" : "",
             gps->valid ? gps->speed_kmh : 0.0f
    );

    /* Send to all emergency contacts */
    for (uint8_t i = 0; i < MAX_EMERGENCY_CONTACTS; i++) {
        bool success = false;

        /* Retry logic per contact */
        for (uint8_t retry = 0; retry < SMS_RETRY_COUNT; retry++) {
            if (gsm_send_sms(s_emergency_contacts[i], message)) {
                success = true;
                sent++;
                break;
            }
            DELAY_MS(SMS_RETRY_DELAY_MS);
        }
        (void)success;    /* Suppress unused warning if not logging */
    }

    return sent;
}

/**
 * @brief  Initiate an emergency voice call.
 *         Calls EMERGENCY_CALL_NUMBER and hangs up after timeout.
 * @return true if call was dialed (not necessarily connected)
 */
bool gsm_make_emergency_call(void)
{
    char cmd[40];

    /* Dial emergency number */
    snprintf(cmd, sizeof(cmd), "ATD%s;", EMERGENCY_CALL_NUMBER);
    if (!gsm_send_at(cmd, "OK", 5000)) {
        return false;
    }

    /* Keep line open for CALL_DURATION_MS milliseconds */
    DELAY_MS(CALL_DURATION_MS);

    /* Hang up */
    gsm_send_at("ATH", "OK", 3000);

    return true;
}
