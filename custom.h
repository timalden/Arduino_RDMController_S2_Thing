/**
 * @file custom.h
 * @brief RDM responder support for manufacturer-specific PID: dimming curves.
 *
 * Modelled on the Weisbrod esp_dmx product_info.c pattern.
 * RDM_PID_MFR_CURVE sits in the E1.20 manufacturer-specific range
 * (0x8000-0xFFDF); 0x8000 == RDM_PID_MANUFACTURER_SPECIFIC_BEGIN.
 *
 * The PD for this PID is an array of NUM_CHANNELS bytes, one per channel.
 * GET returns all channel curves atomically; SET must provide all channels.
 * Format string: "bbbbbb$" (6 x uint8_t, terminated).
 *
 * Place custom.c and custom.h in the sketch folder alongside the .ino.
 */

#pragma once

/* utils.h pulls in rdm/responder.h (rdm_callback_t), rdm/include/types.h
 * (rdm_pid_t etc.), and dmx/include/types.h (dmx_port_t). */
#include "rdm/responder/include/utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Manufacturer-specific PID.
 * 0x8000 == RDM_PID_MANUFACTURER_SPECIFIC_BEGIN (confirmed in types.h).
 * Increment for additional mfr PIDs (0x8001, 0x8002, ...).
 * ------------------------------------------------------------------------- */
#define RDM_PID_MFR_CURVE  ((rdm_pid_t)0x8000)

/* Number of dimmer channels — must match NUM_CHANNELS in the sketch.
 * Defined here with a guard so custom.h can be included standalone. */
#ifndef NUM_CHANNELS
#define NUM_CHANNELS 6
#endif

/* Curve ID type — uint8_t, format token "b" (1 byte per channel).
 * Valid values: 0=square, 1=log, 2=linear, 3=Tempus. */
typedef uint8_t rdm_curve_id_t;

/* ---------------------------------------------------------------------------
 * rdm_register_current_curve()
 *
 * Call once during setup(), after dmx_driver_install().
 * Allocates parameter storage for NUM_CHANNELS bytes and registers GET + SET.
 *
 * @param dmx_num        DMX port (DMX_NUM_1 etc.)
 * @param initial_curves Array of NUM_CHANNELS initial curve IDs (one per ch).
 * @param cb             Optional callback after a successful SET. NULL if unused.
 * @param context        User pointer forwarded to cb.
 * @return true on success.
 * ------------------------------------------------------------------------- */
bool rdm_register_current_curve(dmx_port_t dmx_num,
                                 const rdm_curve_id_t initial_curves[NUM_CHANNELS],
                                 rdm_callback_t cb,
                                 void *context);

/* ---------------------------------------------------------------------------
 * rdm_get_current_curve()
 *
 * Read all NUM_CHANNELS curve IDs from the driver parameter table.
 *
 * @param dmx_num       DMX port.
 * @param[out] curves   Array of NUM_CHANNELS to receive the curve IDs.
 * @return Bytes copied (NUM_CHANNELS on success, 0 on error).
 * ------------------------------------------------------------------------- */
size_t rdm_get_current_curve(dmx_port_t dmx_num,
                              rdm_curve_id_t curves[NUM_CHANNELS]);

/* ---------------------------------------------------------------------------
 * rdm_set_current_curve()
 *
 * Write NUM_CHANNELS curve IDs into the driver parameter table from the sketch.
 * Also queues an RDM status message so a polling controller sees the change.
 *
 * @param dmx_num  DMX port.
 * @param curves   Array of NUM_CHANNELS curve IDs to write.
 * @return true on success.
 * ------------------------------------------------------------------------- */
bool rdm_set_current_curve(dmx_port_t dmx_num,
                            const rdm_curve_id_t curves[NUM_CHANNELS]);

#ifdef __cplusplus
}
#endif
