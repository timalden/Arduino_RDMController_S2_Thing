/**
 * @file custom_controller.c
 * @brief RDM controller-side send functions for RDM_PID_MFR_CURVE.
 *
 * Modelled on the Weisbrod esp_dmx controller pattern (device_control.c).
 *
 * FORMAT STRINGS:
 *   "bbbbbb$"  — 6 x uint8_t, terminated (NUM_CHANNELS curve bytes)
 *   "w"        — uint16_t (used for SUPPORTED_PARAMETERS response, repeated)
 *   "w$"       — uint16_t, terminated (used for PARAMETER_DESCRIPTION request)
 *   "wbbbx00bbddda" — PARAMETER_DESCRIPTION response (fixed E1.20 layout)
 *
 * NOTE on PARAMETER_DESCRIPTION request format:
 *   The outgoing PD is the queried PID — always rdm_pid_t = uint16_t.
 *   This is "w$" regardless of the curve data type. Do NOT change this
 *   when changing the curve format.
 */

#include "custom_controller.h"

#include "rdm/controller/include/utils.h"  /* rdm_send_request, rdm_request_t */
#include "rdm/include/uid.h"               /* rdm_uid_is_broadcast            */

/* ---------------------------------------------------------------------------
 * rdm_send_get_supported_parameters()
 * "w" unterminated — repeated for each uint16_t PID until PDL exhausted.
 * ------------------------------------------------------------------------- */
size_t rdm_send_get_supported_parameters(dmx_port_t dmx_num,
                                          const rdm_uid_t *dest_uid,
                                          rdm_sub_device_t sub_device,
                                          uint16_t *pids,
                                          size_t size,
                                          rdm_ack_t *ack) {
  if (dmx_num >= DMX_NUM_MAX)           return 0;
  if (dest_uid == NULL)                 return 0;
  if (rdm_uid_is_broadcast(dest_uid))   return 0;
  if (sub_device >= RDM_SUB_DEVICE_MAX) return 0;
  if (pids == NULL)                     return 0;

  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_GET_COMMAND,
    .pid        = RDM_PID_SUPPORTED_PARAMETERS,
  };

  return rdm_send_request(dmx_num, &request, "w", pids, size, ack);
}

/* ---------------------------------------------------------------------------
 * rdm_send_get_parameter_description()
 *
 * Request:  "w$"            — PID being queried, always uint16_t
 * Response: "wbbbx00bbddda" — fixed E1.20 PARAMETER_DESCRIPTION layout
 *
 * NOTE: the request .format is "w$" regardless of the curve data type.
 * The PID field is always a uint16_t on the wire.
 * ------------------------------------------------------------------------- */
size_t rdm_send_get_parameter_description(dmx_port_t dmx_num,
                                           const rdm_uid_t *dest_uid,
                                           rdm_sub_device_t sub_device,
                                           rdm_pid_t pid,
                                           rdm_parameter_description_t *param_desc,
                                           rdm_ack_t *ack) {
  if (dmx_num >= DMX_NUM_MAX)           return 0;
  if (dest_uid == NULL)                 return 0;
  if (rdm_uid_is_broadcast(dest_uid))   return 0;
  if (sub_device >= RDM_SUB_DEVICE_MAX) return 0;
  if (param_desc == NULL)               return 0;

  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_GET_COMMAND,
    .pid        = RDM_PID_PARAMETER_DESCRIPTION,
    .pd         = &pid,
    .pdl        = sizeof(pid),        /* sizeof(rdm_pid_t) = sizeof(uint16_t) */
    .format     = "w$",              /* PID being queried — always uint16_t  */
  };

  return rdm_send_request(dmx_num, &request, "wbbbx00bbddda",
                          param_desc, sizeof(*param_desc), ack);
}

/* ---------------------------------------------------------------------------
 * rdm_send_get_current_curve()
 *
 * GET — no outgoing PD. Response is NUM_CHANNELS bytes decoded as "bbbbbb$".
 * ------------------------------------------------------------------------- */
size_t rdm_send_get_current_curve(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   rdm_curve_id_t curves[NUM_CHANNELS],
                                   rdm_ack_t *ack) {
  if (dmx_num >= DMX_NUM_MAX)           return 0;
  if (dest_uid == NULL)                 return 0;
  if (rdm_uid_is_broadcast(dest_uid))   return 0;
  if (sub_device >= RDM_SUB_DEVICE_MAX) return 0;
  if (curves == NULL)                   return 0;

  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_GET_COMMAND,
    .pid        = RDM_PID_MFR_CURVE,
  };

  return rdm_send_request(dmx_num, &request, "bbbbbb$",
                          curves, NUM_CHANNELS * sizeof(rdm_curve_id_t), ack);
}

/* ---------------------------------------------------------------------------
 * rdm_send_set_current_curve()
 *
 * SET — outgoing PD is NUM_CHANNELS bytes, format "bbbbbb$".
 * ACK carries no response PD.
 * ------------------------------------------------------------------------- */
bool rdm_send_set_current_curve(dmx_port_t dmx_num,
                                 const rdm_uid_t *dest_uid,
                                 rdm_sub_device_t sub_device,
                                 const rdm_curve_id_t curves[NUM_CHANNELS],
                                 rdm_ack_t *ack) {
  if (dmx_num >= DMX_NUM_MAX)  return false;
  if (dest_uid == NULL)        return false;
  if (curves == NULL)          return false;
  if (sub_device >= RDM_SUB_DEVICE_MAX &&
      sub_device != RDM_SUB_DEVICE_ALL) return false;

  const rdm_request_t request = {
    .dest_uid   = dest_uid,
    .sub_device = sub_device,
    .cc         = RDM_CC_SET_COMMAND,
    .pid        = RDM_PID_MFR_CURVE,
    .pd         = curves,
    .pdl        = NUM_CHANNELS * sizeof(rdm_curve_id_t),  /* 6 bytes */
    .format     = "bbbbbb$",
  };

  /* Library returns 1 on ACK with zero PDL (see utils.c). */
  return rdm_send_request(dmx_num, &request, NULL, NULL, 0, ack) > 0;
}
