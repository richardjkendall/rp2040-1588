/**
 * Network Configuration
 *
 * Common configuration for both WiFi and Ethernet backends.
 * Device-specific IPs are selected based on build target.
 */

#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

// =============================================================================
// Common Configuration
// =============================================================================

// Connection timeout (applies to WiFi connect or Ethernet link wait)
#define NETWORK_CONNECT_TIMEOUT_MS 30000  // 30 seconds

// Use DHCP for IP address assignment
// Set to 0 to use static IP configuration below
#define NETWORK_USE_DHCP 0

// =============================================================================
// Device IP Configuration
// =============================================================================
// Grandmaster: 192.168.1.100
// Slave:       192.168.1.10

#ifdef BUILD_GRANDMASTER
    #define NETWORK_STATIC_IP      "192.168.1.100"
    #define PTP_PEER_IP            "192.168.1.10"   // Slave's IP (where to send Sync)
#else
    // Slave configuration (default)
    #define NETWORK_STATIC_IP      "192.168.1.10"
    #define PTP_PEER_IP            "192.168.1.100"  // Grandmaster's IP
#endif

// Common network settings
#define NETWORK_STATIC_NETMASK "255.255.255.0"
#define NETWORK_STATIC_GATEWAY "192.168.1.1"

// Legacy defines for compatibility
#define PTP_DEST_IP         PTP_PEER_IP   // Used by grandmaster
#define PTP_GRANDMASTER_IP  PTP_PEER_IP   // Used by slave

// =============================================================================
// WiFi Configuration (only used when USE_ETHERNET is not defined)
// =============================================================================

#ifndef USE_ETHERNET

#define WIFI_SSID     "synctest"
#define WIFI_PASSWORD "ieee1588v2ptp"

#endif // !USE_ETHERNET

// =============================================================================
// Ethernet Configuration (only used when USE_ETHERNET is defined)
// =============================================================================

#ifdef USE_ETHERNET

// MAC address for W5500 (must be unique on network)
// WIZnet OUI is 00:08:DC
#ifdef BUILD_GRANDMASTER
    #define ETH_MAC_ADDR { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x01 }  // GM MAC
#else
    #define ETH_MAC_ADDR { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x02 }  // Slave MAC
#endif

#endif // USE_ETHERNET

#endif // NETWORK_CONFIG_H
