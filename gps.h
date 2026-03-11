/**
 * gps.h — GPS NMEA Parser Public Interface (NEO-6M / NEO-8M)
 * ─────────────────────────────────────────────────────────────
 * Include this header in any module that needs GPS position,
 * speed, heading, altitude, or time data.
 * ─────────────────────────────────────────────────────────────
 */

#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════
   GPS DATA STRUCTURE
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Complete GPS fix data from parsed NMEA sentences.
 *         Populated by gps_update() on each valid $GPRMC/$GPGGA.
 */
struct GpsData {
    /* Fix validity */
    bool        valid;                  /**< true = active fix (GPRMC status 'A') */
    uint8_t     satellites;             /**< Number of tracked satellites         */

    /* Position — decimal degrees */
    float       latitude;               /**< Positive = North, Negative = South   */
    float       longitude;              /**< Positive = East,  Negative = West    */
    float       altitude_m;             /**< Altitude above mean sea level (m)    */

    /* String representations (%.6f format, including sign) */
    char        lat_str[16];            /**< e.g. "12.345678"  or "-12.345678"    */
    char        lon_str[16];            /**< e.g. "77.123456"  or "-77.123456"    */

    /* Motion */
    float       speed_kmh;             /**< Speed over ground, km/h              */
    float       heading_deg;           /**< Track made good, degrees true 0–360  */

    /* Time / Date (UTC, raw NMEA strings) */
    char        time_utc[12];           /**< HHMMSS.ss from $GPRMC field 1        */
    char        date[8];                /**< DDMMYY  from $GPRMC field 9          */
};

/* Convenience typedef matching config.h forward declaration */
typedef struct GpsData GpsData_t;

/* ═══════════════════════════════════════════════════════════
   PUBLIC API
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Initialize the GPS UART and internal buffers.
 *         Must be called once before any other gps_* function.
 */
void gps_init(void);

/**
 * @brief  Poll GPS UART and parse any complete NMEA sentences.
 *         Non-blocking — drains whatever bytes are available.
 *         Call as frequently as possible (e.g. every main-loop tick).
 *
 * @param  gps  Pointer to GpsData_t to update on valid parse.
 * @return true if a new valid fix was obtained this call.
 */
bool gps_update(GpsData_t *gps);

/**
 * @brief  Block until a valid GPS fix is obtained or timeout expires.
 *         Falls back to last-known-good position if available.
 *
 * @param  gps          Output GPS data structure.
 * @param  timeout_ms   Maximum time to wait in milliseconds.
 * @return true  — valid fix (or last-known) returned in *gps.
 * @return false — timed out with no fix and no prior data.
 */
bool gps_wait_for_fix(GpsData_t *gps, uint32_t timeout_ms);

/**
 * @brief  Build a Google Maps URL from the GPS coordinates.
 *         Output: "https://maps.google.com/?q=<lat>,<lon>"
 *
 * @param  gps      GPS data with populated lat_str / lon_str.
 * @param  url_buf  Caller-supplied output buffer.
 * @param  buf_len  Size of url_buf in bytes.
 */
void gps_format_maps_url(const GpsData_t *gps, char *url_buf, uint16_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
