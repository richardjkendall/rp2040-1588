/**
 * Simple Ethernet Frame Handler Implementation
 */

#include "eth_simple.h"
#include "w5500_simple.h"
#include <stdio.h>
#include <string.h>

// Network configuration (static)
static net_config_t net_cfg;

// Helper: Convert host byte order to network byte order (16-bit)
static inline uint16_t htons(uint16_t x) {
    return ((x & 0xFF) << 8) | ((x >> 8) & 0xFF);
}

// Helper: Convert host byte order to network byte order (32-bit)
static inline uint32_t htonl(uint32_t x) {
    return ((x & 0xFF) << 24) | ((x & 0xFF00) << 8) |
           ((x >> 8) & 0xFF00) | ((x >> 24) & 0xFF);
}

// Helper: Convert network byte order to host byte order (16-bit)
static inline uint16_t ntohs(uint16_t x) {
    return htons(x);  // Same operation
}

// Helper: Convert network byte order to host byte order (32-bit)
static inline uint32_t ntohl(uint32_t x) {
    return htonl(x);  // Same operation
}

// Calculate IP checksum
static uint16_t ip_checksum(const uint8_t *data, uint16_t len) {
    uint32_t sum = 0;

    // Sum 16-bit words
    for (uint16_t i = 0; i < len; i += 2) {
        if (i + 1 < len) {
            sum += (data[i] << 8) | data[i + 1];
        } else {
            sum += data[i] << 8;
        }
    }

    // Fold 32-bit sum to 16 bits
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return ~sum;
}

void eth_init(const net_config_t *config) {
    memcpy(&net_cfg, config, sizeof(net_config_t));
    printf("Ethernet config: %u.%u.%u.%u\n",
           (net_cfg.ip >> 24) & 0xFF,
           (net_cfg.ip >> 16) & 0xFF,
           (net_cfg.ip >> 8) & 0xFF,
           net_cfg.ip & 0xFF);
}

bool eth_parse_frame(const uint8_t *raw_frame, uint16_t len, eth_frame_t *frame) {
    if (len < 14) {
        return false;  // Too short for Ethernet header
    }

    // Parse Ethernet header
    memcpy(frame->dest_mac, &raw_frame[0], 6);
    memcpy(frame->src_mac, &raw_frame[6], 6);
    frame->ethertype = (raw_frame[12] << 8) | raw_frame[13];
    frame->payload = &raw_frame[14];
    frame->payload_len = len - 14;

    return true;
}

bool eth_parse_arp(const uint8_t *payload, uint16_t len, arp_packet_t *arp) {
    if (len < 28) {
        return false;  // Too short for ARP
    }

    arp->hw_type = (payload[0] << 8) | payload[1];
    arp->proto_type = (payload[2] << 8) | payload[3];
    arp->hw_len = payload[4];
    arp->proto_len = payload[5];
    arp->opcode = (payload[6] << 8) | payload[7];

    memcpy(arp->sender_mac, &payload[8], 6);
    // Read IP address byte-by-byte to avoid alignment issues
    arp->sender_ip = ((uint32_t)payload[14] << 24) |
                     ((uint32_t)payload[15] << 16) |
                     ((uint32_t)payload[16] << 8) |
                     ((uint32_t)payload[17]);

    memcpy(arp->target_mac, &payload[18], 6);
    // Read IP address byte-by-byte to avoid alignment issues
    arp->target_ip = ((uint32_t)payload[24] << 24) |
                     ((uint32_t)payload[25] << 16) |
                     ((uint32_t)payload[26] << 8) |
                     ((uint32_t)payload[27]);

    return true;
}

bool eth_parse_ip(const uint8_t *payload, uint16_t len, ip_packet_t *ip) {
    if (len < 20) {
        return false;  // Too short for IP header
    }

    ip->version = payload[0] >> 4;
    ip->header_len = (payload[0] & 0x0F);  // In 32-bit words
    ip->total_len = (payload[2] << 8) | payload[3];
    ip->protocol = payload[9];

    // Read IP addresses byte-by-byte to avoid alignment issues
    ip->src_ip = ((uint32_t)payload[12] << 24) |
                 ((uint32_t)payload[13] << 16) |
                 ((uint32_t)payload[14] << 8) |
                 ((uint32_t)payload[15]);
    ip->dest_ip = ((uint32_t)payload[16] << 24) |
                  ((uint32_t)payload[17] << 16) |
                  ((uint32_t)payload[18] << 8) |
                  ((uint32_t)payload[19]);

    uint16_t header_bytes = ip->header_len * 4;
    if (len < header_bytes) {
        return false;
    }

    ip->payload = &payload[header_bytes];
    ip->payload_len = len - header_bytes;

    return true;
}

bool eth_parse_udp(const uint8_t *payload, uint16_t len, udp_packet_t *udp) {
    if (len < 8) {
        return false;  // Too short for UDP header
    }

    udp->src_port = (payload[0] << 8) | payload[1];
    udp->dest_port = (payload[2] << 8) | payload[3];
    udp->length = (payload[4] << 8) | payload[5];
    // Skip checksum (bytes 6-7)

    udp->payload = &payload[8];
    udp->payload_len = len - 8;

    return true;
}

bool eth_parse_icmp(const uint8_t *payload, uint16_t len, icmp_packet_t *icmp) {
    if (len < 8) {
        return false;  // Too short for ICMP header
    }

    icmp->type = payload[0];
    icmp->code = payload[1];
    icmp->checksum = (payload[2] << 8) | payload[3];
    icmp->identifier = (payload[4] << 8) | payload[5];
    icmp->sequence = (payload[6] << 8) | payload[7];

    icmp->payload = &payload[8];
    icmp->payload_len = len - 8;

    return true;
}

uint16_t eth_build_arp_reply(uint8_t *buffer, const uint8_t *target_mac, uint32_t target_ip) {
    uint16_t pos = 0;

    // Ethernet header (14 bytes)
    memcpy(&buffer[pos], target_mac, 6); pos += 6;        // Dest MAC
    memcpy(&buffer[pos], net_cfg.mac, 6); pos += 6;       // Src MAC
    buffer[pos++] = 0x08; buffer[pos++] = 0x06;           // EtherType: ARP

    // ARP packet (28 bytes)
    buffer[pos++] = 0x00; buffer[pos++] = 0x01;           // HW type: Ethernet
    buffer[pos++] = 0x08; buffer[pos++] = 0x00;           // Protocol: IPv4
    buffer[pos++] = 0x06;                                  // HW addr len: 6
    buffer[pos++] = 0x04;                                  // Protocol addr len: 4
    buffer[pos++] = 0x00; buffer[pos++] = 0x02;           // Opcode: Reply

    memcpy(&buffer[pos], net_cfg.mac, 6); pos += 6;       // Sender MAC (us)
    uint32_t ip_be = htonl(net_cfg.ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Sender IP (us)

    memcpy(&buffer[pos], target_mac, 6); pos += 6;        // Target MAC
    ip_be = htonl(target_ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Target IP

    // Pad to minimum frame size (60 bytes payload)
    while (pos < 60) {
        buffer[pos++] = 0;
    }

    return pos;
}

uint16_t eth_build_arp_request(uint8_t *buffer, uint32_t target_ip) {
    uint16_t pos = 0;

    // Ethernet header (14 bytes) - broadcast
    uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(&buffer[pos], broadcast, 6); pos += 6;         // Dest MAC (broadcast)
    memcpy(&buffer[pos], net_cfg.mac, 6); pos += 6;       // Src MAC (us)
    buffer[pos++] = 0x08; buffer[pos++] = 0x06;           // EtherType: ARP

    // ARP packet (28 bytes)
    buffer[pos++] = 0x00; buffer[pos++] = 0x01;           // HW type: Ethernet
    buffer[pos++] = 0x08; buffer[pos++] = 0x00;           // Protocol: IPv4
    buffer[pos++] = 0x06;                                  // HW addr len: 6
    buffer[pos++] = 0x04;                                  // Protocol addr len: 4
    buffer[pos++] = 0x00; buffer[pos++] = 0x01;           // Opcode: Request

    memcpy(&buffer[pos], net_cfg.mac, 6); pos += 6;       // Sender MAC (us)
    uint32_t ip_be = htonl(net_cfg.ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Sender IP (us)

    // Target (who we're asking about)
    uint8_t zero_mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(&buffer[pos], zero_mac, 6); pos += 6;          // Target MAC (unknown, use zeros)
    ip_be = htonl(target_ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Target IP

    // Pad to minimum frame size (60 bytes)
    while (pos < 60) {
        buffer[pos++] = 0;
    }

    return pos;
}

uint16_t eth_build_icmp_echo_reply(uint8_t *buffer,
                                    const uint8_t *dest_mac, uint32_t dest_ip,
                                    uint16_t identifier, uint16_t sequence,
                                    const uint8_t *echo_data, uint16_t echo_len) {
    uint16_t pos = 0;

    // Ethernet header (14 bytes)
    memcpy(&buffer[pos], dest_mac, 6); pos += 6;
    memcpy(&buffer[pos], net_cfg.mac, 6); pos += 6;
    buffer[pos++] = 0x08; buffer[pos++] = 0x00;           // EtherType: IPv4

    // IP header (20 bytes, no options)
    uint16_t ip_total_len = 20 + 8 + echo_len;  // IP header + ICMP header + data
    buffer[pos++] = 0x45;                                  // Version 4, IHL 5
    buffer[pos++] = 0x00;                                  // DSCP/ECN
    buffer[pos++] = ip_total_len >> 8;
    buffer[pos++] = ip_total_len & 0xFF;
    buffer[pos++] = 0x00; buffer[pos++] = 0x00;           // ID
    buffer[pos++] = 0x40; buffer[pos++] = 0x00;           // Flags: Don't fragment
    buffer[pos++] = 0x40;                                  // TTL: 64
    buffer[pos++] = 1;                                     // Protocol: ICMP
    buffer[pos++] = 0x00; buffer[pos++] = 0x00;           // Checksum (fill later)
    uint32_t ip_be = htonl(net_cfg.ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Src IP
    ip_be = htonl(dest_ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Dest IP

    // Calculate IP checksum
    uint16_t ip_csum = ip_checksum(&buffer[14], 20);
    buffer[24] = ip_csum >> 8;
    buffer[25] = ip_csum & 0xFF;

    // ICMP header (8 bytes)
    uint16_t icmp_start = pos;
    buffer[pos++] = 0;                                     // Type: Echo Reply
    buffer[pos++] = 0;                                     // Code: 0
    buffer[pos++] = 0x00; buffer[pos++] = 0x00;           // Checksum (fill later)
    buffer[pos++] = identifier >> 8;
    buffer[pos++] = identifier & 0xFF;
    buffer[pos++] = sequence >> 8;
    buffer[pos++] = sequence & 0xFF;

    // Echo data
    memcpy(&buffer[pos], echo_data, echo_len);
    pos += echo_len;

    // Calculate ICMP checksum
    uint16_t icmp_len = 8 + echo_len;
    uint16_t icmp_csum = ip_checksum(&buffer[icmp_start], icmp_len);
    buffer[icmp_start + 2] = icmp_csum >> 8;
    buffer[icmp_start + 3] = icmp_csum & 0xFF;

    // Pad to minimum frame size if needed
    while (pos < 60) {
        buffer[pos++] = 0;
    }

    return pos;
}

uint16_t eth_build_udp(uint8_t *buffer,
                       const uint8_t *dest_mac, uint32_t dest_ip,
                       uint16_t src_port, uint16_t dest_port,
                       const uint8_t *payload, uint16_t payload_len) {
    uint16_t pos = 0;

    // Ethernet header (14 bytes)
    memcpy(&buffer[pos], dest_mac, 6); pos += 6;
    memcpy(&buffer[pos], net_cfg.mac, 6); pos += 6;
    buffer[pos++] = 0x08; buffer[pos++] = 0x00;           // EtherType: IPv4

    // IP header (20 bytes, no options)
    uint16_t ip_total_len = 20 + 8 + payload_len;
    buffer[pos++] = 0x45;                                  // Version 4, IHL 5
    buffer[pos++] = 0x00;                                  // DSCP/ECN
    buffer[pos++] = ip_total_len >> 8;
    buffer[pos++] = ip_total_len & 0xFF;
    buffer[pos++] = 0x00; buffer[pos++] = 0x00;           // ID
    buffer[pos++] = 0x40; buffer[pos++] = 0x00;           // Flags: Don't fragment
    buffer[pos++] = 0x40;                                  // TTL: 64
    buffer[pos++] = 17;                                    // Protocol: UDP
    buffer[pos++] = 0x00; buffer[pos++] = 0x00;           // Checksum (fill later)
    uint32_t ip_be = htonl(net_cfg.ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Src IP
    ip_be = htonl(dest_ip);
    memcpy(&buffer[pos], &ip_be, 4); pos += 4;            // Dest IP

    // Calculate IP checksum
    uint16_t ip_csum = ip_checksum(&buffer[14], 20);
    buffer[24] = ip_csum >> 8;
    buffer[25] = ip_csum & 0xFF;

    // UDP header (8 bytes)
    uint16_t udp_len = 8 + payload_len;
    buffer[pos++] = src_port >> 8;
    buffer[pos++] = src_port & 0xFF;
    buffer[pos++] = dest_port >> 8;
    buffer[pos++] = dest_port & 0xFF;
    buffer[pos++] = udp_len >> 8;
    buffer[pos++] = udp_len & 0xFF;
    buffer[pos++] = 0x00; buffer[pos++] = 0x00;           // Checksum: Optional (0)

    // UDP payload
    memcpy(&buffer[pos], payload, payload_len);
    pos += payload_len;

    // Pad to minimum frame size if needed
    while (pos < 60) {
        buffer[pos++] = 0;
    }

    return pos;
}

bool eth_handle_frame(const uint8_t *raw_frame, uint16_t len,
                      udp_packet_t *udp, uint8_t *src_mac, uint32_t *src_ip) {
    eth_frame_t frame;
    if (!eth_parse_frame(raw_frame, len, &frame)) {
        return false;
    }

    // Handle ARP
    if (frame.ethertype == 0x0806) {
        arp_packet_t arp;
        if (!eth_parse_arp(frame.payload, frame.payload_len, &arp)) {
            return false;
        }

        // ARP request for our IP?
        if (arp.opcode == 1 && arp.target_ip == net_cfg.ip) {
            printf("ARP request from %u.%u.%u.%u - sending reply\n",
                   (arp.sender_ip >> 24) & 0xFF,
                   (arp.sender_ip >> 16) & 0xFF,
                   (arp.sender_ip >> 8) & 0xFF,
                   arp.sender_ip & 0xFF);

            // Send ARP reply
            uint8_t reply_buffer[60];
            uint16_t reply_len = eth_build_arp_reply(reply_buffer, arp.sender_mac, arp.sender_ip);
            w5500_send_frame(reply_buffer, reply_len);
        }

        return false;  // No UDP packet
    }

    // Handle IPv4
    if (frame.ethertype == 0x0800) {
        ip_packet_t ip;
        if (!eth_parse_ip(frame.payload, frame.payload_len, &ip)) {
            return false;
        }

        // IP packet for us or broadcast?
        if (ip.dest_ip != net_cfg.ip && ip.dest_ip != 0xFFFFFFFF) {
            return false;  // Not for us
        }

        // Handle ICMP
        if (ip.protocol == 1) {
            icmp_packet_t icmp;
            if (!eth_parse_icmp(ip.payload, ip.payload_len, &icmp)) {
                return false;
            }

            // ICMP Echo Request?
            if (icmp.type == 8 && icmp.code == 0) {
                printf("Ping from %u.%u.%u.%u - sending reply\n",
                       (ip.src_ip >> 24) & 0xFF,
                       (ip.src_ip >> 16) & 0xFF,
                       (ip.src_ip >> 8) & 0xFF,
                       ip.src_ip & 0xFF);

                // Send ICMP Echo Reply
                uint8_t reply_buffer[512];  // Enough for most ping packets
                uint16_t reply_len = eth_build_icmp_echo_reply(reply_buffer,
                                                                frame.src_mac, ip.src_ip,
                                                                icmp.identifier, icmp.sequence,
                                                                icmp.payload, icmp.payload_len);
                w5500_send_frame(reply_buffer, reply_len);
            }

            return false;  // No UDP packet
        }

        // Handle UDP
        if (ip.protocol == 17) {
            if (!eth_parse_udp(ip.payload, ip.payload_len, udp)) {
                return false;
            }

            // Return source info
            memcpy(src_mac, frame.src_mac, 6);
            *src_ip = ip.src_ip;

            return true;  // UDP packet available
        }
    }

    return false;
}
