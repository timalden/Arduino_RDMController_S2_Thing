/**
 * @file custom_controller.h
 * @brief RDM controller-side send functions for RDM_PID_MFR_CURVE.
 *
 * Modelled on the Weisbrod esp_dmx controller pattern (device_control.c/.h).
 * Place custom_controller.c and custom_controller.h in the sketch folder,
 * alongside custom.h (which defines RDM_PID_MFR_CURVE, rdm_curve_id_t,
 * and NUM_CHANNELS).
 *
 * The PD for RDM_PID_MFR_CURVE is an array of NUM_CHANNELS bytes.
 * GET returns all channel curves; SET must provide all channels.
 */
#pragma once

#include "custom.h"          /* RDM_PID_MFR_CURVE, rdm_curve_id_t, NUM_CHANNELS */
#include "rdm/controller.h"  /* rdm_ack_t, rdm_uid_t, rdm_sub_device_t          */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sends an RDM GET RDM_PID_SUPPORTED_PARAMETERS request and reads the
 * response into a caller-supplied array of PIDs.
 *
 * The response is a packed array of uint16_t PIDs. Standard minimum-required
 * PIDs are excluded by the responder per E1.20.
 *
 * @param dmx_num       DMX port number.
 * @param dest_uid      UID of the target responder (must not be broadcast).
 * @param sub_device    Sub-device number (use RDM_SUB_DEVICE_ROOT).
 * @param[out] pids     Buffer to receive the PID list.
 * @param size          Size of pids buffer in bytes.
 * @param[out] ack      ACK metadata.
 * @return              PDL bytes received (nPids * 2 on ACK), 0 on error.
 */
size_t rdm_send_get_supported_parameters(dmx_port_t dmx_num,
                                          const rdm_uid_t *dest_uid,
                                          rdm_sub_device_t sub_device,
                                          uint16_t *pids,
                                          size_t size,
                                          rdm_ack_t *ack);

/**
 * @brief Sends an RDM GET RDM_PID_PARAMETER_DESCRIPTION request for the given
 * PID and reads the response into an rdm_parameter_description_t struct.
 *
 * Only valid for manufacturer-specific PIDs (0x8000-0xFFDF).
 *
 * @param dmx_num          DMX port number.
 * @param dest_uid         UID of the target responder (must not be broadcast).
 * @param sub_device       Sub-device (use RDM_SUB_DEVICE_ROOT).
 * @param pid              Manufacturer-specific PID to query.
 * @param[out] param_desc  Receives the parameter description on ACK.
 * @param[out] ack         ACK metadata.
 * @return                 PDL bytes received on ACK, 0 on timeout/error.
 */
size_t rdm_send_get_parameter_description(dmx_port_t dmx_num,
                                           const rdm_uid_t *dest_uid,
                                           rdm_sub_device_t sub_device,
                                           rdm_pid_t pid,
                                           rdm_parameter_description_t *param_desc,
                                           rdm_ack_t *ack);

/**
 * @brief Sends an RDM GET RDM_PID_MFR_CURVE request and reads the response.
 *
 * On ACK the response PD contains NUM_CHANNELS bytes, one curve ID per channel.
 *
 * @param dmx_num        DMX port number.
 * @param dest_uid       UID of the target responder (must not be broadcast).
 * @param sub_device     Sub-device number (use RDM_SUB_DEVICE_ROOT).
 * @param[out] curves    Array of NUM_CHANNELS to receive curve IDs on ACK.
 * @param[out] ack       ACK metadata.
 * @return               PDL bytes received (NUM_CHANNELS on ACK), 0 on error.
 */
size_t rdm_send_get_current_curve(dmx_port_t dmx_num,
                                   const rdm_uid_t *dest_uid,
                                   rdm_sub_device_t sub_device,
                                   rdm_curve_id_t curves[NUM_CHANNELS],
                                   rdm_ack_t *ack);

/**
 * @brief Sends an RDM SET RDM_PID_MFR_CURVE request and reads the response.
 *
 * The request PD is NUM_CHANNELS bytes, one curve ID per channel.
 * A successful ACK carries no response PD.
 *
 * @param dmx_num    DMX port number.
 * @param dest_uid   UID of the target responder.
 * @param sub_device Sub-device number.
 * @param curves     Array of NUM_CHANNELS curve IDs to set.
 * @param[out] ack   ACK metadata.
 * @return           true if RDM_RESPONSE_TYPE_ACK received, false otherwise.
 */
bool rdm_send_set_current_curve(dmx_port_t dmx_num,
                                 const rdm_uid_t *dest_uid,
                                 rdm_sub_device_t sub_device,
                                 const rdm_curve_id_t curves[NUM_CHANNELS],
                                 rdm_ack_t *ack);

#ifdef __cplusplus
}
#endif
