/**
 * @file custom_controller.h
 * @brief RDM controller-side send functions for manufacturer-specific PIDs.
 *
 * Modelled on the Weisbrod esp_dmx controller pattern (device_control.c/.h).
 * Place custom_controller.c and custom_controller.h in the sketch folder,
 * alongside custom.h.
 *
 * The implementation in custom_controller.c uses a static generic core
 * (rdm_send_mfr_get / rdm_send_mfr_set) so that each PID's GET and SET
 * reduce to a one-line wrapper specifying the format string and buffer.
 *
 * Also declares rdm_send_get_supported_parameters and
 * rdm_send_get_parameter_description which are generic enough to work
 * against any device, not just the S3HW dimmer.
 *
 * NOTE: rdm_send_get_parameter_description always uses "w$" for the outgoing
 * PID field regardless of the curve or other PD types — rdm_pid_t is always
 * uint16_t on the wire.
 */
#pragma once

#include <string.h>          /* strnlen                                       */
#include "custom.h"          /* PID defines, rdm_curve_id_t, rdm_mfr_mode_t  */
#include "rdm/controller.h"  /* rdm_ack_t, rdm_uid_t, rdm_sub_device_t       */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Generic RDM utilities (work against any device)
 * ------------------------------------------------------------------------- */

/**
 * @brief Sends GET RDM_PID_SUPPORTED_PARAMETERS and reads the PID list.
 * Standard minimum-required PIDs are excluded by the responder per E1.20.
 *
 * @param[out] pids  Buffer to receive uint16_t PID values.
 * @param size       Size of pids buffer in bytes.
 * @return           PDL bytes received (nPids * 2 on ACK), 0 on error.
 */
size_t rdm_send_get_supported_parameters(dmx_port_t dmx_num,
                                          const rdm_uid_t *dest_uid,
                                          rdm_sub_device_t sub_device,
                                          uint16_t *pids,
                                          size_t size,
                                          rdm_ack_t *ack);

/**
 * @brief Sends GET RDM_PID_PARAMETER_DESCRIPTION for a manufacturer PID.
 * Only valid for PIDs in 0x8000-0xFFDF.
 *
 * @param pid              Manufacturer PID to query.
 * @param[out] param_desc  Receives the description on ACK.
 * @return                 PDL bytes received on ACK, 0 on error.
 */
size_t rdm_send_get_parameter_description(dmx_port_t dmx_num,
                                           const rdm_uid_t *dest_uid,
                                           rdm_sub_device_t sub_device,
                                           rdm_pid_t pid,
                                           rdm_parameter_description_t *param_desc,
                                           rdm_ack_t *ack);

/* ---------------------------------------------------------------------------
 * RDM_PID_MFR_CURVE  (0x8000)
 * PD: NUM_CHANNELS bytes, one curve ID per channel, format "bbbbbb$".
 * ------------------------------------------------------------------------- */

/** GET — returns NUM_CHANNELS bytes on ACK, 0 on error. */
size_t rdm_send_get_current_curve(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   rdm_curve_id_t curves[NUM_CHANNELS],
                                   rdm_ack_t *ack);

/** SET — all NUM_CHANNELS must be provided. Returns true on ACK. */
bool rdm_send_set_current_curve(dmx_port_t dmx_num,
                                 const rdm_uid_t *dest_uid,
                                 rdm_sub_device_t sub_device,
                                 const rdm_curve_id_t curves[NUM_CHANNELS],
                                 rdm_ack_t *ack);

/* ---------------------------------------------------------------------------
 * RDM_PID_MFR_LABEL  (0x8001)
 * PD: ASCII string ≤ RDM_MFR_LABEL_SIZE bytes, format "a$".
 * ------------------------------------------------------------------------- */

/**
 * GET — copies label into buf (null-terminated).
 * buf must be at least RDM_MFR_LABEL_SIZE+1 bytes.
 * Returns PDL bytes received (string length), 0 on error.
 */
size_t rdm_send_get_mfr_label(dmx_port_t dmx_num,
                               const rdm_uid_t *dest_uid,
                               rdm_sub_device_t sub_device,
                               char *buf,
                               size_t buf_size,
                               rdm_ack_t *ack);

/** SET — label is a null-terminated C string, max RDM_MFR_LABEL_SIZE chars. */
bool rdm_send_set_mfr_label(dmx_port_t dmx_num,
                              const rdm_uid_t *dest_uid,
                              rdm_sub_device_t sub_device,
                              const char *label,
                              rdm_ack_t *ack);

/* ---------------------------------------------------------------------------
 * RDM_PID_MFR_MODE  (0x8002)
 * PD: single uint8_t, format "b$".
 * ------------------------------------------------------------------------- */

/** GET — writes mode on ACK, returns 1 on success, 0 on error. */
size_t rdm_send_get_mfr_mode(dmx_port_t dmx_num,
                              const rdm_uid_t *dest_uid,
                              rdm_sub_device_t sub_device,
                              rdm_mfr_mode_t *mode,
                              rdm_ack_t *ack);

/** SET — returns true on ACK. */
bool rdm_send_set_mfr_mode(dmx_port_t dmx_num,
                            const rdm_uid_t *dest_uid,
                            rdm_sub_device_t sub_device,
                            rdm_mfr_mode_t mode,
                            rdm_ack_t *ack);

/* ---------------------------------------------------------------------------
 * DMX personality helpers (standard PIDs, added here for controller sketch
 * convenience; these are not manufacturer-specific).
 * ------------------------------------------------------------------------- */

bool rdm_send_get_dmx_personality(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   uint8_t *current,
                                   uint8_t *count,
                                   rdm_ack_t *ack);

bool rdm_send_set_dmx_personality(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   uint8_t personality_num,
                                   rdm_ack_t *ack);

size_t rdm_send_get_dmx_personality_description(
    dmx_port_t dmx_num,
    const rdm_uid_t *dest_uid,
    rdm_sub_device_t sub_device,
    uint8_t personality_num,
    rdm_dmx_personality_description_t *desc,
    rdm_ack_t *ack);

#ifdef __cplusplus
}
#endif
