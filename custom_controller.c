/**
 * @file custom_controller.c
 * @brief RDM controller-side send functions for manufacturer-specific PIDs.
 *
 * ARCHITECTURE
 * ============
 * Two static generic helpers cover the common GET/SET patterns:
 *
 *   rdm_send_mfr_get(dmx_num, dest, sub, pid, response_fmt, out, size, ack)
 *     — sends a GET with no outgoing PD; decodes response with response_fmt.
 *
 *   rdm_send_mfr_set(dmx_num, dest, sub, pid, request_fmt, in, size, ack)
 *     — sends a SET with outgoing PD described by request_fmt; no response PD.
 *
 * Each typed public wrapper is one line calling one of these, supplying the
 * correct PID, format string, and buffer.  Adding a new PID is therefore
 * essentially zero boilerplate in the generic machinery.
 *
 * FORMAT STRINGS:
 *   "bbbbbb$" — 6 x uint8_t (MFR_CURVE)
 *   "a$"      — ASCII string up to 32 bytes (MFR_LABEL)
 *   "b$"      — single uint8_t (MFR_MODE)
 *   "w"       — uint16_t repeated (SUPPORTED_PARAMETERS response)
 *   "w$"      — single uint16_t (PARAMETER_DESCRIPTION request PID field —
 *               always uint16_t regardless of any other PD type changes)
 *   "wbbbx00bbddda" — PARAMETER_DESCRIPTION response (fixed E1.20 layout)
 */

#include "custom_controller.h"

#include <string.h>                        /* strnlen                         */
#include "rdm/controller/include/utils.h"  /* rdm_send_request, rdm_request_t */
#include "rdm/include/uid.h"               /* rdm_uid_is_broadcast            */

/* ===========================================================================
 * GENERIC CORE — static, not exposed in custom_controller.h
 * =========================================================================*/

/**
 * @brief Send a manufacturer GET request with no outgoing PD.
 *
 * @param pid              PID to GET.
 * @param response_format  Format string to decode the ACK response PD.
 * @param out              Output buffer for decoded response.
 * @param out_size         Size of output buffer in bytes.
 * @return                 PDL bytes received on ACK, 0 on error.
 */
static size_t rdm_send_mfr_get(dmx_port_t dmx_num,
                                 const rdm_uid_t *dest_uid,
                                 rdm_sub_device_t sub_device,
                                 rdm_pid_t pid,
                                 const char *response_format,
                                 void *out,
                                 size_t out_size,
                                 rdm_ack_t *ack) {
  if (dmx_num >= DMX_NUM_MAX)           return 0;
  if (dest_uid == NULL)                 return 0;
  if (rdm_uid_is_broadcast(dest_uid))   return 0;
  if (sub_device >= RDM_SUB_DEVICE_MAX) return 0;

  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_GET_COMMAND,
    .pid        = pid,
    /* no outgoing PD — .pd/.pdl/.format zero-initialised */
  };

  return rdm_send_request(dmx_num, &request, response_format, out, out_size, ack);
}

/**
 * @brief Send a manufacturer SET request; ACK carries no response PD.
 *
 * @param pid             PID to SET.
 * @param request_format  Format string describing the outgoing PD.
 * @param in              Outgoing PD buffer.
 * @param in_size         Outgoing PD size in bytes.
 * @return                true on ACK (library returns 1 for zero-PDL ACK).
 */
static bool rdm_send_mfr_set(dmx_port_t dmx_num,
                               const rdm_uid_t *dest_uid,
                               rdm_sub_device_t sub_device,
                               rdm_pid_t pid,
                               const char *request_format,
                               const void *in,
                               size_t in_size,
                               rdm_ack_t *ack) {
  if (dmx_num >= DMX_NUM_MAX)   return false;
  if (dest_uid == NULL)         return false;
  if (in == NULL)               return false;
  if (sub_device >= RDM_SUB_DEVICE_MAX &&
      sub_device != RDM_SUB_DEVICE_ALL) return false;

  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_SET_COMMAND,
    .pid        = pid,
    .pd         = in,
    .pdl        = in_size,
    .format     = request_format,
  };

  return rdm_send_request(dmx_num, &request, NULL, NULL, 0, ack) > 0;
}

/* ===========================================================================
 * Generic RDM utilities
 * =========================================================================*/

size_t rdm_send_get_supported_parameters(dmx_port_t dmx_num,
                                          const rdm_uid_t *dest_uid,
                                          rdm_sub_device_t sub_device,
                                          uint16_t *pids,
                                          size_t size,
                                          rdm_ack_t *ack) {
  if (pids == NULL) return 0;
  /* "w" unterminated — repeated for each uint16_t until PDL exhausted */
  return rdm_send_mfr_get(dmx_num, dest_uid, sub_device,
                           RDM_PID_SUPPORTED_PARAMETERS,
                           "w", pids, size, ack);
}

size_t rdm_send_get_parameter_description(dmx_port_t dmx_num,
                                           const rdm_uid_t *dest_uid,
                                           rdm_sub_device_t sub_device,
                                           rdm_pid_t pid,
                                           rdm_parameter_description_t *param_desc,
                                           rdm_ack_t *ack) {
  if (param_desc == NULL) return 0;

  /* Outgoing PD is the queried PID — always uint16_t, format "w$".
   * Do NOT change this to match any other PD type.                        */
  if (dmx_num >= DMX_NUM_MAX)           return 0;
  if (dest_uid == NULL)                 return 0;
  if (rdm_uid_is_broadcast(dest_uid))   return 0;
  if (sub_device >= RDM_SUB_DEVICE_MAX) return 0;

  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_GET_COMMAND,
    .pid        = RDM_PID_PARAMETER_DESCRIPTION,
    .pd         = &pid,
    .pdl        = sizeof(pid),   /* always sizeof(uint16_t) */
    .format     = "w$",         /* always "w$" — pid_t is always uint16_t */
  };

  return rdm_send_request(dmx_num, &request, "wbbbx00bbddda",
                          param_desc, sizeof(*param_desc), ack);
}

/* ===========================================================================
 * RDM_PID_MFR_CURVE  (0x8000)
 * =========================================================================*/

size_t rdm_send_get_current_curve(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   rdm_curve_id_t curves[NUM_CHANNELS],
                                   rdm_ack_t *ack) {
  if (curves == NULL) return 0;
  return rdm_send_mfr_get(dmx_num, dest_uid, sub_device,
                           RDM_PID_MFR_CURVE, "bbbbbb$",
                           curves, NUM_CHANNELS * sizeof(rdm_curve_id_t), ack);
}

bool rdm_send_set_current_curve(dmx_port_t dmx_num,
                                 const rdm_uid_t *dest_uid,
                                 rdm_sub_device_t sub_device,
                                 const rdm_curve_id_t curves[NUM_CHANNELS],
                                 rdm_ack_t *ack) {
  if (curves == NULL) return false;
  return rdm_send_mfr_set(dmx_num, dest_uid, sub_device,
                           RDM_PID_MFR_CURVE, "bbbbbb$",
                           curves, NUM_CHANNELS * sizeof(rdm_curve_id_t), ack);
}

/* ===========================================================================
 * RDM_PID_MFR_LABEL  (0x8001)
 * =========================================================================*/

size_t rdm_send_get_mfr_label(dmx_port_t dmx_num,
                               const rdm_uid_t *dest_uid,
                               rdm_sub_device_t sub_device,
                               char *buf,
                               size_t buf_size,
                               rdm_ack_t *ack) {
  if (buf == NULL || buf_size == 0) return 0;
  size_t n = rdm_send_mfr_get(dmx_num, dest_uid, sub_device,
                               RDM_PID_MFR_LABEL, "a$",
                               buf, buf_size - 1, ack);
  buf[n] = '\0';  /* null-terminate regardless of what the responder sent  */
  return n;
}

bool rdm_send_set_mfr_label(dmx_port_t dmx_num,
                              const rdm_uid_t *dest_uid,
                              rdm_sub_device_t sub_device,
                              const char *label,
                              rdm_ack_t *ack) {
  if (label == NULL) return false;
  size_t len = strnlen(label, RDM_MFR_LABEL_SIZE);
  return rdm_send_mfr_set(dmx_num, dest_uid, sub_device,
                           RDM_PID_MFR_LABEL, "a$",
                           label, len, ack);
}

/* ===========================================================================
 * RDM_PID_MFR_MODE  (0x8002)
 * =========================================================================*/

size_t rdm_send_get_mfr_mode(dmx_port_t dmx_num,
                              const rdm_uid_t *dest_uid,
                              rdm_sub_device_t sub_device,
                              rdm_mfr_mode_t *mode,
                              rdm_ack_t *ack) {
  if (mode == NULL) return 0;
  return rdm_send_mfr_get(dmx_num, dest_uid, sub_device,
                           RDM_PID_MFR_MODE, "b$",
                           mode, sizeof(*mode), ack);
}

bool rdm_send_set_mfr_mode(dmx_port_t dmx_num,
                            const rdm_uid_t *dest_uid,
                            rdm_sub_device_t sub_device,
                            rdm_mfr_mode_t mode,
                            rdm_ack_t *ack) {
  return rdm_send_mfr_set(dmx_num, dest_uid, sub_device,
                           RDM_PID_MFR_MODE, "b$",
                           &mode, sizeof(mode), ack);
}

/* ===========================================================================
 * Standard DMX personality PIDs
 * =========================================================================*/

bool rdm_send_get_dmx_personality(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   uint8_t *current,
                                   uint8_t *count,
                                   rdm_ack_t *ack) {
  if (current == NULL || count == NULL) return false;
  uint8_t buf[2] = {0, 0};
  size_t n = rdm_send_mfr_get(dmx_num, dest_uid, sub_device,
                               RDM_PID_DMX_PERSONALITY,
                               "bb$", buf, sizeof(buf), ack);
  if (n >= 2) { *current = buf[0]; *count = buf[1]; return true; }
  return false;
}

bool rdm_send_set_dmx_personality(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   uint8_t personality_num,
                                   rdm_ack_t *ack) {
  return rdm_send_mfr_set(dmx_num, dest_uid, sub_device,
                           RDM_PID_DMX_PERSONALITY, "b$",
                           &personality_num, sizeof(personality_num), ack);
}

size_t rdm_send_get_dmx_personality_description(
    dmx_port_t dmx_num,
    const rdm_uid_t *dest_uid,
    rdm_sub_device_t sub_device,
    uint8_t personality_num,
    rdm_dmx_personality_description_t *desc,
    rdm_ack_t *ack) {
  if (desc == NULL) return 0;
  if (dmx_num >= DMX_NUM_MAX)           return 0;
  if (dest_uid == NULL)                 return 0;
  if (rdm_uid_is_broadcast(dest_uid))   return 0;
  if (sub_device >= RDM_SUB_DEVICE_MAX) return 0;

  /* Personality description has an outgoing PD (the personality number),
   * so it can't go through rdm_send_mfr_get which assumes no outgoing PD. */
  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_GET_COMMAND,
    .pid        = RDM_PID_DMX_PERSONALITY_DESCRIPTION,
    .pd         = &personality_num,
    .pdl        = sizeof(personality_num),
    .format     = "b$",
  };
  return rdm_send_request(dmx_num, &request, "bwa$", desc, sizeof(*desc), ack);
}
