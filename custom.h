/**
 * @file custom.h
 * @brief RDM responder support for manufacturer-specific PIDs.
 *
 * Modelled on the Weisbrod esp_dmx responder PID pattern.
 * All PIDs are in the E1.20 manufacturer-specific range (0x8000-0xFFDF).
 *
 * PIDs defined here:
 *   0x8000  RDM_PID_MFR_CURVE  — per-channel dimming curve (6 x uint8_t)
 *   0x8001  RDM_PID_MFR_LABEL  — rig/circuit label string (≤32 ASCII chars)
 *   0x8002  RDM_PID_MFR_MODE   — operating mode enum (uint8_t)
 *
 * Implementation pattern:
 *   custom.c contains a static generic core (register_mfr_pid, mfr_pid_get,
 *   mfr_pid_set) and thin typed public wrappers for each PID.  Adding a new
 *   PID requires one static rdm_parameter_definition_t and three wrapper
 *   functions; the generic core does not change.
 *
 * Place custom.c and custom.h in the sketch folder alongside the .ino.
 */

#pragma once

/* utils.h pulls in rdm/responder.h (rdm_callback_t), rdm/include/types.h,
 * and dmx/include/types.h (dmx_port_t). */
#include "rdm/responder/include/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Manufacturer-specific PID constants.
 * Increment sequentially for each new PID.
 * ------------------------------------------------------------------------- */
#define RDM_PID_MFR_CURVE  ((rdm_pid_t)0x8000)  /* 6 x uint8_t curve array  */
#define RDM_PID_MFR_LABEL  ((rdm_pid_t)0x8001)  /* ASCII label string        */
#define RDM_PID_MFR_MODE   ((rdm_pid_t)0x8002)  /* uint8_t mode enum         */

/* ---------------------------------------------------------------------------
 * Shared constants.
 * ------------------------------------------------------------------------- */

/* Number of dimmer channels.  Guard allows custom.h to be included without
 * the sketch defining it first. */
#ifndef NUM_CHANNELS
#define NUM_CHANNELS 6
#endif

/* Maximum label length (wire bytes, no null terminator). */
#define RDM_MFR_LABEL_SIZE  32

/* ---------------------------------------------------------------------------
 * Type aliases.
 * ------------------------------------------------------------------------- */

/* Curve ID — one per channel, valid values 0-3. */
typedef uint8_t rdm_curve_id_t;

/* Operating mode — application-defined enum encoded as uint8_t. */
typedef uint8_t rdm_mfr_mode_t;

/* ---------------------------------------------------------------------------
 * RDM_PID_MFR_CURVE  (0x8000)
 * PD: NUM_CHANNELS bytes, one curve ID per channel, format "bbbbbb$".
 * ------------------------------------------------------------------------- */
bool rdm_register_current_curve(dmx_port_t dmx_num,
                                 const rdm_curve_id_t initial_curves[NUM_CHANNELS],
                                 rdm_callback_t cb,
                                 void *context);

size_t rdm_get_current_curve(dmx_port_t dmx_num,
                              rdm_curve_id_t curves[NUM_CHANNELS]);

bool rdm_set_current_curve(dmx_port_t dmx_num,
                            const rdm_curve_id_t curves[NUM_CHANNELS]);

/* ---------------------------------------------------------------------------
 * RDM_PID_MFR_LABEL  (0x8001)
 * PD: ASCII string up to RDM_MFR_LABEL_SIZE bytes, format "a$".
 * Storage is RDM_MFR_LABEL_SIZE + 1 bytes (null-terminated internally).
 * ------------------------------------------------------------------------- */
bool rdm_register_mfr_label(dmx_port_t dmx_num,
                              const char *initial_label,
                              rdm_callback_t cb,
                              void *context);

/* Copies label into buf; caller must provide buf of at least
 * RDM_MFR_LABEL_SIZE+1 bytes to allow null termination. */
size_t rdm_get_mfr_label(dmx_port_t dmx_num, char *buf, size_t buf_size);

bool rdm_set_mfr_label(dmx_port_t dmx_num, const char *label);

/* ---------------------------------------------------------------------------
 * RDM_PID_MFR_MODE  (0x8002)
 * PD: single uint8_t, format "b$".
 * Application-defined values; suggest 0=normal, 1=test, 2=safe.
 * ------------------------------------------------------------------------- */
bool rdm_register_mfr_mode(dmx_port_t dmx_num,
                             rdm_mfr_mode_t initial_mode,
                             rdm_callback_t cb,
                             void *context);

size_t rdm_get_mfr_mode(dmx_port_t dmx_num, rdm_mfr_mode_t *mode);

bool rdm_set_mfr_mode(dmx_port_t dmx_num, rdm_mfr_mode_t mode);

#ifdef __cplusplus
}
#endif
