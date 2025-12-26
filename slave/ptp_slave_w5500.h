/**
 * PTP Slave Protocol Handler - W5500 Ethernet
 *
 * Handles PTP slave-side protocol operations:
 * - Receive Sync/Follow_Up from grandmaster
 * - Send Delay_Req to grandmaster
 * - Receive Delay_Resp from grandmaster
 * - Capture timestamps and pass to discipline module
 */

#ifndef PTP_SLAVE_W5500_H
#define PTP_SLAVE_W5500_H

#include <stdint.h>
#include <stdbool.h>
#include "ptp_protocol.h"
#include "../grandmaster/eth_simple.h"

/**
 * Initialize PTP slave protocol
 *
 * @param clock_id Pointer to our clock identity (from MAC)
 * @param domain PTP domain number (usually 0)
 */
void ptp_slave_w5500_init(ptp_clock_identity_t *clock_id, uint8_t domain);

/**
 * Handle received Sync message
 *
 * Captures t2 (slave RX timestamp) and PIO counter value immediately.
 * Stores sync sequence ID for matching with Follow_Up.
 *
 * @param sync Pointer to Sync message
 * @param udp Pointer to UDP packet info
 * @param src_mac Source MAC address
 * @param src_ip Source IP address
 */
void ptp_slave_w5500_handle_sync(const ptp_sync_msg_t *sync, const udp_packet_t *udp,
                                 const uint8_t *src_mac, uint32_t src_ip);

/**
 * Handle received Follow_Up message
 *
 * Extracts t1 (master TX timestamp) from precise_origin_timestamp.
 * Matches with previous Sync by sequence ID.
 * Signals to discipline that Sync+Follow_Up pair is ready.
 *
 * @param followup Pointer to Follow_Up message
 */
void ptp_slave_w5500_handle_followup(const ptp_follow_up_msg_t *followup);

/**
 * Handle received Delay_Resp message
 *
 * Extracts t4 (master RX timestamp) from receive_timestamp.
 * Matches with our Delay_Req by sequence ID.
 * Signals to discipline that complete timestamp set is ready.
 *
 * @param delay_resp Pointer to Delay_Resp message
 */
void ptp_slave_w5500_handle_delay_resp(const ptp_delay_resp_msg_t *delay_resp);

/**
 * Send Delay_Req message to grandmaster
 *
 * Captures t3 (slave TX timestamp) immediately before sending.
 * Should be called periodically (typically 1 Hz).
 *
 * @param dest_mac Grandmaster MAC address
 * @param dest_ip Grandmaster IP address
 * @param frame_buffer Buffer to build ethernet frame
 * @return Length of ethernet frame to send
 */
uint16_t ptp_slave_w5500_send_delay_req(const uint8_t *dest_mac, uint32_t dest_ip,
                                        uint8_t *frame_buffer);

/**
 * Get PTP slave statistics
 *
 * @param sync_count Output: number of Sync messages received
 * @param followup_count Output: number of Follow_Up messages received
 * @param delay_req_count Output: number of Delay_Req messages sent
 * @param delay_resp_count Output: number of Delay_Resp messages received
 * @param gm_known Output: true if grandmaster has been discovered
 */
void ptp_slave_w5500_get_stats(uint32_t *sync_count, uint32_t *followup_count,
                               uint32_t *delay_req_count, uint32_t *delay_resp_count,
                               bool *gm_known);

/**
 * Get grandmaster MAC address (learned from first Sync)
 *
 * @return Pointer to 6-byte MAC address
 */
const uint8_t* ptp_slave_w5500_get_gm_mac(void);

/**
 * Get grandmaster IP address (learned from first Sync)
 *
 * @return IP address in host byte order
 */
uint32_t ptp_slave_w5500_get_gm_ip(void);

/**
 * Get RX averaging statistics
 *
 * @param avg_ns Output: running average RX latency in nanoseconds
 * @param sample_count Output: number of successful HW samples
 * @param fallback_count Output: number of times average was used
 */
void ptp_slave_w5500_get_rx_avg_stats(int64_t *avg_ns, uint32_t *sample_count, uint32_t *fallback_count);

#endif // PTP_SLAVE_W5500_H
