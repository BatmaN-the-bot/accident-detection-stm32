/**
 * gps.c — GPS NMEA Parser (NEO-6M / NEO-8M)
 * ─────────────────────────────────────────────────────────────
 * Reads NMEA 0183 sentences from GPS UART and extracts:
 *   - Latitude / Longitude (decimal degrees)
 *   - Speed, heading, altitude
 *   - Satellite count and fix validity
 *   - UTC time and date
 *
 * Primary sentences parsed:
 *   $GPRMC  — Recommended minimum GPS data (lat, lon, speed, time)
 *   $GPGGA  — Fix data (altitude, satellites)
 *   $GPGLL  — Geographic position (backup)
 * ─────────────────────────────────────────────────────────────
 */

#include "gps.h"
#include "config.h"

/* ─── Private receive buffer ────────────────────────────────── */
static char  s_nmea_buf[GPS_BUFFER_SIZE];
static uint16_t s_buf_idx = 0;
static GpsData_t s_last_valid_gps = {0};

/* ─────────────────────────────────────────────────────────────
   PLATFORM UART WRAPPERS
   Replace with your UART read function.
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Read one byte from GPS UART.
 *         Returns -1 if no data available.
 */
static int16_t gps_uart_read_byte(void)
{
#ifdef PLATFORM_STM32
    /* SPL: poll USART1 receive register */
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) {
        return (int16_t)(USART_ReceiveData(USART1) & 0xFF);
    }
    return -1;
#else
    /* Arduino Serial1 (GPS on hardware serial 1) */
    if (Serial1.available()) {
        return Serial1.read();
    }
    return -1;
#endif
}

/* ─────────────────────────────────────────────────────────────
   NMEA UTILITIES
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Validate NMEA sentence checksum.
 *         Checksum = XOR of all bytes between '$' and '*'.
 * @return true if checksum matches
 */
static bool nmea_validate_checksum(const char *sentence)
{
    const char *p = sentence;
    uint8_t computed = 0;
    uint8_t received = 0;

    if (*p++ != '$') return false;

    /* XOR all characters between $ and * */
    while (*p && *p != '*') {
        computed ^= (uint8_t)(*p++);
    }

    if (*p != '*') return false;
    p++;  /* Skip '*' */

    /* Parse two-hex-digit checksum */
    for (uint8_t i = 0; i < 2; i++) {
        received <<= 4;
        char c = *p++;
        if      (c >= '0' && c <= '9') received |= (c - '0');
        else if (c >= 'A' && c <= 'F') received |= (c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') received |= (c - 'a' + 10);
        else return false;
    }

    return (computed == received);
}

/**
 * @brief  Extract Nth comma-separated field from NMEA sentence.
 * @param  sentence  Full NMEA string
 * @param  field_num 0-indexed field number
 * @param  out       Output buffer
 * @param  out_len   Max output length
 * @return true if field found
 */
static bool nmea_get_field(const char *sentence, uint8_t field_num,
                            char *out, uint8_t out_len)
{
    uint8_t field = 0;
    uint8_t i = 0;

    /* Skip to desired field */
    while (*sentence && field < field_num) {
        if (*sentence++ == ',') field++;
    }
    if (field != field_num) return false;

    /* Copy field until next comma or end of sentence */
    while (*sentence && *sentence != ',' && *sentence != '*' && i < (out_len - 1)) {
        out[i++] = *sentence++;
    }
    out[i] = '\0';

    return (i > 0);
}

/**
 * @brief  Convert NMEA lat/lon format (DDDMM.MMMM) to decimal degrees.
 * @param  nmea_val  NMEA coordinate string
 * @param  direction 'N', 'S', 'E', or 'W'
 * @return Decimal degrees (negative for S/W)
 */
static float nmea_to_decimal_degrees(const char *nmea_val, char direction)
{
    float raw = strtof(nmea_val, NULL);

    /* NMEA format: DDDMM.MMMM
     * Degrees = integer part of (raw / 100)
     * Minutes = raw - (degrees * 100)                                        */
    int degrees = (int)(raw / 100.0f);
    float minutes = raw - (float)(degrees * 100);
    float decimal = (float)degrees + (minutes / 60.0f);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

/* ─────────────────────────────────────────────────────────────
   NMEA SENTENCE PARSERS
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Parse $GPRMC sentence.
 * Format: $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,x.x,a*hh
 *   Field 0: $GPRMC
 *   Field 1: UTC time HHMMSS.ss
 *   Field 2: Status A=active, V=void
 *   Field 3: Latitude DDMM.MMMM
 *   Field 4: N/S indicator
 *   Field 5: Longitude DDDMM.MMMM
 *   Field 6: E/W indicator
 *   Field 7: Speed over ground (knots)
 *   Field 8: Track/heading (degrees)
 *   Field 9: Date DDMMYY
 */
static bool parse_gprmc(const char *sentence, GpsData_t *gps)
{
    char field[20];

    /* Check status (field 2) — A = valid, V = invalid */
    if (!nmea_get_field(sentence, 2, field, sizeof(field))) return false;
    gps->valid = (field[0] == 'A');
    if (!gps->valid) return false;

    /* Time (field 1) */
    if (nmea_get_field(sentence, 1, field, sizeof(field))) {
        strncpy(gps->time_utc, field, sizeof(gps->time_utc) - 1);
    }

    /* Latitude (fields 3 & 4) */
    char lat_val[16], lat_dir[4];
    if (nmea_get_field(sentence, 3, lat_val, sizeof(lat_val)) &&
        nmea_get_field(sentence, 4, lat_dir, sizeof(lat_dir))) {
        gps->latitude = nmea_to_decimal_degrees(lat_val, lat_dir[0]);
        snprintf(gps->lat_str, sizeof(gps->lat_str), "%.6f", gps->latitude);
    }

    /* Longitude (fields 5 & 6) */
    char lon_val[16], lon_dir[4];
    if (nmea_get_field(sentence, 5, lon_val, sizeof(lon_val)) &&
        nmea_get_field(sentence, 6, lon_dir, sizeof(lon_dir))) {
        gps->longitude = nmea_to_decimal_degrees(lon_val, lon_dir[0]);
        snprintf(gps->lon_str, sizeof(gps->lon_str), "%.6f", gps->longitude);
    }

    /* Speed in knots → km/h (field 7) */
    if (nmea_get_field(sentence, 7, field, sizeof(field)) && field[0] != '\0') {
        gps->speed_kmh = strtof(field, NULL) * 1.852f;
    }

    /* Heading (field 8) */
    if (nmea_get_field(sentence, 8, field, sizeof(field)) && field[0] != '\0') {
        gps->heading_deg = strtof(field, NULL);
    }

    /* Date (field 9) */
    if (nmea_get_field(sentence, 9, field, sizeof(field))) {
        strncpy(gps->date, field, sizeof(gps->date) - 1);
    }

    return true;
}

/**
 * @brief  Parse $GPGGA sentence for altitude and satellite count.
 * Format: $GPGGA,HHMMSS.ss,LLLL.LL,a,YYYYY.YY,a,x,xx,x.x,x.x,M,...
 *   Field 6:  Fix quality (0=invalid, 1=GPS, 2=DGPS)
 *   Field 7:  Number of satellites
 *   Field 9:  Altitude above MSL (meters)
 */
static bool parse_gpgga(const char *sentence, GpsData_t *gps)
{
    char field[20];

    /* Fix quality (field 6) */
    if (!nmea_get_field(sentence, 6, field, sizeof(field))) return false;
    if (field[0] == '0') return false;   /* No fix */

    /* Satellite count (field 7) */
    if (nmea_get_field(sentence, 7, field, sizeof(field))) {
        gps->satellites = (uint8_t)atoi(field);
    }

    /* Altitude (field 9) */
    if (nmea_get_field(sentence, 9, field, sizeof(field)) && field[0] != '\0') {
        gps->altitude_m = strtof(field, NULL);
    }

    return true;
}

/* ─────────────────────────────────────────────────────────────
   PUBLIC FUNCTIONS
   ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialize GPS module.
 *         Sends optional configuration commands to NEO-6M.
 */
void gps_init(void)
{
#ifdef PLATFORM_STM32
    /* SPL: configure USART1 at GPS_UART_BAUD, 8N1
     * Assumes GPIOA clock and USART1 clock already enabled by SystemInit */
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 = TX (AF push-pull), PA10 = RX (input floating) */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    uart.USART_BaudRate            = GPS_UART_BAUD;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);

#else
    Serial1.begin(GPS_UART_BAUD);
#endif

    memset(&s_last_valid_gps, 0, sizeof(s_last_valid_gps));
    s_buf_idx = 0;
}

/**
 * @brief  Poll GPS UART and process any complete NMEA sentences.
 *         Call this in your main loop as frequently as possible.
 * @param  gps  Output GPS data structure
 * @return true if new valid fix data was parsed
 */
bool gps_update(GpsData_t *gps)
{
    bool new_fix = false;
    int16_t byte;

    /* Read all available bytes from UART */
    while ((byte = gps_uart_read_byte()) >= 0) {

        char c = (char)byte;

        /* Start of new NMEA sentence */
        if (c == '$') {
            s_buf_idx = 0;
        }

        /* Buffer overflow protection */
        if (s_buf_idx >= GPS_BUFFER_SIZE - 1) {
            s_buf_idx = 0;
            continue;
        }

        s_nmea_buf[s_buf_idx++] = c;

        /* End of sentence (\r\n) */
        if (c == '\n' && s_buf_idx > 5) {
            s_nmea_buf[s_buf_idx] = '\0';

            /* Validate checksum before parsing */
            if (!nmea_validate_checksum(s_nmea_buf)) {
                s_buf_idx = 0;
                continue;
            }

            /* Identify and parse sentence type */
            if (strncmp(s_nmea_buf, "$GPRMC", 6) == 0 ||
                strncmp(s_nmea_buf, "$GNRMC", 6) == 0) {
                if (parse_gprmc(s_nmea_buf, gps)) {
                    memcpy(&s_last_valid_gps, gps, sizeof(GpsData_t));
                    new_fix = true;
                }
            }
            else if (strncmp(s_nmea_buf, "$GPGGA", 6) == 0 ||
                     strncmp(s_nmea_buf, "$GNGGA", 6) == 0) {
                parse_gpgga(s_nmea_buf, gps);
            }

            s_buf_idx = 0;
        }
    }

    return new_fix;
}

/**
 * @brief  Block until a valid GPS fix is obtained (or timeout).
 * @param  gps          Output GPS data
 * @param  timeout_ms   Maximum time to wait
 * @return true if valid fix obtained, false if timed out
 */
bool gps_wait_for_fix(GpsData_t *gps, uint32_t timeout_ms)
{
    uint32_t start = GET_TICK_MS();

    while ((GET_TICK_MS() - start) < timeout_ms) {
        if (gps_update(gps) && gps->valid &&
            gps->satellites >= GPS_MIN_SATELLITES) {
            return true;
        }
        DELAY_MS(100);
    }

    /* Fall back to last known position if available */
    if (s_last_valid_gps.valid) {
        memcpy(gps, &s_last_valid_gps, sizeof(GpsData_t));
        return true;
    }

    return false;
}

/**
 * @brief  Format GPS coordinates as a Google Maps URL.
 * @param  gps     GPS data with valid lat/lon
 * @param  url_buf Output buffer for URL
 * @param  buf_len Buffer size
 */
void gps_format_maps_url(const GpsData_t *gps, char *url_buf, uint16_t buf_len)
{
    snprintf(url_buf, buf_len,
             "https://maps.google.com/?q=%s,%s",
             gps->lat_str, gps->lon_str);
}
