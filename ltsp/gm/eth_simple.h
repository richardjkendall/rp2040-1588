/**
 * Simple Ethernet Frame Handler
 *
 * Minimal implementation for ARP, IP, and UDP handling.
 * No dynamic allocation, simple parsing for PTP grandmaster.
 */

#ifndef ETH_SIMPLE_H
#define ETH_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>

// Network configuration
typedef struct {
    uint8_t mac[6];
    uint32_t ip;         // Host byte order
    uint32_t netmask;    // Host byte order
    uint32_t gateway;    // Host byte order
} net_config_t;

// Ethernet frame (parsed)
typedef struct {
    uint8_t dest_mac[6];
    uint8_t src_mac[6];
    uint16_t ethertype;  // 0x0806 = ARP, 0x0800 = IPv4
    const uint8_t *payload;
    uint16_t payload_len;
} eth_frame_t;

// ARP packet (parsed)
typedef struct {
    uint16_t hw_type;
    uint16_t proto_type;
    uint8_t hw_len;
    uint8_t proto_len;
    uint16_t opcode;     // 1 = Request, 2 = Reply
    uint8_t sender_mac[6];
    uint32_t sender_ip;  // Host byte order
    uint8_t target_mac[6];
    uint32_t target_ip;  // Host byte order
} arp_packet_t;

// IP header (parsed)
typedef struct {
    uint8_t version;
    uint8_t header_len;  // In 32-bit words
    uint16_t total_len;
    uint8_t protocol;    // 17 = UDP
    uint32_t src_ip;     // Host byte order
    uint32_t dest_ip;    // Host byte order
    const uint8_t *payload;
    uint16_t payload_len;
} ip_packet_t;

// UDP packet (parsed)
typedef struct {
    uint16_t src_port;
    uint16_t dest_port;
    uint16_t length;
    const uint8_t *payload;
    uint16_t payload_len;
} udp_packet_t;

// ICMP packet (parsed)
typedef struct {
    uint8_t type;        // 8 = Echo Request, 0 = Echo Reply
    uint8_t code;
    uint16_t checksum;
    uint16_t identifier;
    uint16_t sequence;
    const uint8_t *payload;
    uint16_t payload_len;
} icmp_packet_t;

/**
 * Initialize network configuration
 *
 * @param config Network configuration (MAC, IP, netmask, gateway)
 */
void eth_init(const net_config_t *config);

/**
 * Parse Ethernet frame
 *
 * @param raw_frame Raw frame bytes
 * @param len Frame length
 * @param frame Output parsed frame
 * @return true if valid frame, false otherwise
 */
bool eth_parse_frame(const uint8_t *raw_frame, uint16_t len, eth_frame_t *frame);

/**
 * Parse ARP packet
 *
 * @param payload ARP payload from Ethernet frame
 * @param len Payload length
 * @param arp Output parsed ARP packet
 * @return true if valid ARP, false otherwise
 */
bool eth_parse_arp(const uint8_t *payload, uint16_t len, arp_packet_t *arp);

/**
 * Parse IP packet
 *
 * @param payload IP payload from Ethernet frame
 * @param len Payload length
 * @param ip Output parsed IP packet
 * @return true if valid IP, false otherwise
 */
bool eth_parse_ip(const uint8_t *payload, uint16_t len, ip_packet_t *ip);

/**
 * Parse UDP packet
 *
 * @param payload UDP payload from IP packet
 * @param len Payload length
 * @param udp Output parsed UDP packet
 * @return true if valid UDP, false otherwise
 */
bool eth_parse_udp(const uint8_t *payload, uint16_t len, udp_packet_t *udp);

/**
 * Parse ICMP packet
 *
 * @param payload ICMP payload from IP packet
 * @param len Payload length
 * @param icmp Output parsed ICMP packet
 * @return true if valid ICMP, false otherwise
 */
bool eth_parse_icmp(const uint8_t *payload, uint16_t len, icmp_packet_t *icmp);

/**
 * Build ICMP echo reply
 *
 * @param buffer Output buffer for frame
 * @param dest_mac Destination MAC address
 * @param dest_ip Destination IP address (host byte order)
 * @param identifier ICMP identifier from request
 * @param sequence ICMP sequence number from request
 * @param echo_data Echo data from request
 * @param echo_len Echo data length
 * @return Frame length
 */
uint16_t eth_build_icmp_echo_reply(uint8_t *buffer,
                                    const uint8_t *dest_mac, uint32_t dest_ip,
                                    uint16_t identifier, uint16_t sequence,
                                    const uint8_t *echo_data, uint16_t echo_len);

/**
 * Build ARP reply
 *
 * @param buffer Output buffer for frame
 * @param target_mac Target MAC address
 * @param target_ip Target IP address (host byte order)
 * @return Frame length
 */
uint16_t eth_build_arp_reply(uint8_t *buffer, const uint8_t *target_mac, uint32_t target_ip);

/**
 * Build ARP request (can be used for Gratuitous ARP)
 *
 * @param buffer Output buffer for frame
 * @param target_ip Target IP address to query (host byte order)
 * @return Frame length
 */
uint16_t eth_build_arp_request(uint8_t *buffer, uint32_t target_ip);

/**
 * Build UDP packet
 *
 * @param buffer Output buffer for frame
 * @param dest_mac Destination MAC address
 * @param dest_ip Destination IP address (host byte order)
 * @param src_port Source UDP port
 * @param dest_port Destination UDP port
 * @param payload UDP payload data
 * @param payload_len Payload length
 * @return Frame length
 */
uint16_t eth_build_udp(uint8_t *buffer,
                       const uint8_t *dest_mac, uint32_t dest_ip,
                       uint16_t src_port, uint16_t dest_port,
                       const uint8_t *payload, uint16_t payload_len);

/**
 * Handle received Ethernet frame
 *
 * Processes ARP requests and ICMP echo requests automatically.
 * Returns UDP packets for application processing.
 *
 * @param raw_frame Raw frame bytes
 * @param len Frame length
 * @param udp Output: parsed UDP packet (if any)
 * @param src_mac Output: source MAC address (if UDP packet)
 * @param src_ip Output: source IP address (if UDP packet, host byte order)
 * @return true if UDP packet available, false otherwise
 */
bool eth_handle_frame(const uint8_t *raw_frame, uint16_t len,
                      udp_packet_t *udp, uint8_t *src_mac, uint32_t *src_ip);

/**
 * Build LTSP raw Ethernet frame (EtherType 0x88B5, broadcast)
 *
 * @param buffer Output buffer for frame (must be >= 14 + payload_len)
 * @param payload LTSP PDU payload bytes (already packed)
 * @param payload_len Payload length (typically 56)
 * @return Frame length
 */
uint16_t eth_build_ltsp(uint8_t *buffer, const uint8_t *payload, uint16_t payload_len);

#endif // ETH_SIMPLE_H
