/**
 * Network Interface Abstraction
 *
 * Provides a common interface for WiFi (CYW43) and Ethernet (W5500) backends.
 * The backend is selected at compile time via USE_ETHERNET define.
 */

#ifndef NETWORK_INTERFACE_H
#define NETWORK_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "lwip/ip_addr.h"

/**
 * Initialize network interface and connect
 *
 * For WiFi: Initializes CYW43 and connects to configured SSID
 * For Ethernet: Initializes W5500 and waits for link
 *
 * @return true if successful, false otherwise
 */
bool network_init(void);

/**
 * Poll network stack (must be called regularly in main loop)
 *
 * Call this frequently (e.g., every 1-10ms) to handle network events
 */
void network_poll(void);

/**
 * Check if network is connected
 *
 * @return true if connected, false otherwise
 */
bool network_is_connected(void);

/**
 * Get current IP address
 *
 * @return IP address (0.0.0.0 if not connected)
 */
ip_addr_t network_get_ip(void);

/**
 * Get IP address as string
 *
 * @param buffer Output buffer for IP address string
 * @param buffer_len Length of output buffer
 */
void network_get_ip_str(char *buffer, size_t buffer_len);

/**
 * Get MAC address of network interface
 *
 * @param mac Output buffer for 6-byte MAC address
 */
void network_get_mac(uint8_t mac[6]);

#endif // NETWORK_INTERFACE_H
