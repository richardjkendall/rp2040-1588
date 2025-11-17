/**
 * WiFi Configuration for PTP Grandmaster
 *
 * IMPORTANT: Update these values to match your network!
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// WiFi Credentials - UPDATE THESE!
#define WIFI_SSID "synctest"
#define WIFI_PASSWORD "ieee1588v2ptp"

// Static IP Configuration - UPDATE TO MATCH YOUR NETWORK!
// Make sure this IP doesn't conflict with other devices
#define STATIC_IP_ADDR "192.168.1.100"
#define STATIC_NETMASK "255.255.255.0"
#define STATIC_GATEWAY "192.168.1.1"

// WiFi connection timeout
#define WIFI_CONNECT_TIMEOUT_MS 30000  // 30 seconds

// PTP Destination - UPDATE THIS!
// Set to "0.0.0.0" for multicast (224.0.1.129)
// Set to specific IP (e.g. "192.168.1.50") for unicast to that host
#define PTP_DEST_IP "192.168.1.10"  // Change to your receiver's IP address

#endif // WIFI_CONFIG_H
