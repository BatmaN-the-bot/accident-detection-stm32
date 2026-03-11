/**
 * gsm.h — GSM / SIM800L Module Public Interface
 * ─────────────────────────────────────────────────────────────
 * Manages AT-command communication with a SIM800L (or compatible)
 * GSM module for emergency SMS dispatch and voice calls.
 * ─────────────────────────────────────────────────────────────
 */

#ifndef GSM_H
#define GSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "gps.h"    /* GpsData_t used in SMS payload */

/* ═══════════════════════════════════════════════════════════
   PUBLIC API
   ═══════════════════════════════════════════════════════════ */

/**
 * @brief  Initialize the SIM800L and wait for network registration.
 *         Sends AT handshake, disables echo, sets SMS text mode,
 *         and polls CREG until registered or timeout.
 *
 * @return true  — module alive and network registered.
 * @return false — no response or registration timed out.
 */
bool gsm_init(void);

/**
 * @brief  Query current GSM signal strength (RSSI).
 *
 * @return 0–31 signal level (0 = -113 dBm, 31 = -51 dBm),
 *         99  = unknown / no signal,
 *         -1  = AT command failed.
 */
int8_t gsm_get_signal_strength(void);

/**
 * @brief  Send a plain SMS to a single phone number.
 *
 * @param  number   Destination in international format (e.g. "+919876543210").
 * @param  message  Message body — max 160 chars for a single SMS.
 * @return true  — +CMGS confirmation received.
 * @return false — module error or no confirmation.
 */
bool gsm_send_sms(const char *number, const char *message);

/**
 * @brief  Send an emergency SMS to all configured contacts.
 *         Message includes GPS coordinates / Maps URL, peak G-force,
 *         speed, and rollover flag.
 *
 * @param  gps       Current GPS fix data.
 * @param  peak_g    Highest recorded G-force during the event.
 * @param  rollover  true if rollover was detected.
 * @return Number of contacts successfully messaged.
 */
uint8_t gsm_send_emergency_sms(const GpsData_t *gps,
                                float            peak_g,
                                bool             rollover);

/**
 * @brief  Dial EMERGENCY_CALL_NUMBER, hold for CALL_DURATION_MS, then hang up.
 *
 * @return true  — ATD command accepted (call was dialled).
 * @return false — module did not respond to dial command.
 */
bool gsm_make_emergency_call(void);

#ifdef __cplusplus
}
#endif

#endif /* GSM_H */
