/**
 * WiFi Configuration for PTP Slave
 *
 * IMPORTANT: Update these values to match your network!
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// WiFi Credentials - UPDATE THESE!
#define WIFI_SSID "synctest"
#define WIFI_PASSWORD "ieee1588v2ptp"

// Static IP Configuration - UPDATE TO MATCH YOUR NETWORK!
// Make sure this IP doesn't conflict with grandmaster or other devices
#define STATIC_IP_ADDR "192.168.1.10"
#define STATIC_NETMASK "255.255.255.0"
#define STATIC_GATEWAY "192.168.1.1"

// WiFi connection timeout
#define WIFI_CONNECT_TIMEOUT_MS 30000  // 30 seconds

#endif // WIFI_CONFIG_H
