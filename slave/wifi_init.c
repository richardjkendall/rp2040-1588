/**
 * WiFi Initialization Module for PTP Slave
 */

#include "wifi_init.h"
#include "wifi_config.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

static bool wifi_connected = false;

bool wifi_init_and_connect(void) {
    printf("Initializing WiFi...\n");

    // Initialize CYW43 with lwIP in polling mode
    if (cyw43_arch_init()) {
        printf("ERROR: CYW43 init failed\n");
        return false;
    }

    // Enable station mode (client)
    cyw43_arch_enable_sta_mode();

    printf("Connecting to %s...\n", WIFI_SSID);

    // Connect to WiFi with timeout
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    int connect_result = -1;

    while (to_ms_since_boot(get_absolute_time()) - start_ms < WIFI_CONNECT_TIMEOUT_MS) {
        connect_result = cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            5000  // 5 second timeout per attempt
        );

        if (connect_result == 0) {
            break;  // Connected successfully
        }

        // Retry silently
        sleep_ms(1000);
    }

    if (connect_result != 0) {
        printf("ERROR: Connection timeout\n");
        cyw43_arch_deinit();
        return false;
    }

    printf("WiFi connected\n");

    // Configure static IP address
    printf("Configuring static IP: %s\n", STATIC_IP_ADDR);

    ip4_addr_t ip, netmask, gateway;

    // Parse IP addresses from config
    if (!ip4addr_aton(STATIC_IP_ADDR, &ip)) {
        printf("ERROR: Invalid static IP address\n");
        return false;
    }
    if (!ip4addr_aton(STATIC_NETMASK, &netmask)) {
        printf("ERROR: Invalid netmask\n");
        return false;
    }
    if (!ip4addr_aton(STATIC_GATEWAY, &gateway)) {
        printf("ERROR: Invalid gateway\n");
        return false;
    }

    // Get the network interface
    struct netif *netif = netif_default;
    if (netif == NULL) {
        printf("ERROR: No default network interface\n");
        cyw43_arch_deinit();
        return false;
    }

    // Set static IP (disables DHCP)
    netif_set_addr(netif, &ip, &netmask, &gateway);

    printf("Static IP configured\n");
    printf("  IP: %s\n", ip4addr_ntoa(&ip));
    printf("  Netmask: %s\n", ip4addr_ntoa(&netmask));
    printf("  Gateway: %s\n", ip4addr_ntoa(&gateway));

    wifi_connected = true;
    return true;
}

bool wifi_is_connected(void) {
    if (!wifi_connected) {
        return false;  // Never initialized
    }

    int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);

    // Accept both LINK_JOIN (1) and LINK_UP (3) as "connected"
    return (link_status == CYW43_LINK_JOIN || link_status == CYW43_LINK_UP);
}

void wifi_get_ip_address(char *buffer, size_t buffer_len) {
    if (!wifi_connected || netif_default == NULL) {
        snprintf(buffer, buffer_len, "0.0.0.0");
        return;
    }

    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    snprintf(buffer, buffer_len, "%s", ip4addr_ntoa(ip));
}

void wifi_poll(void) {
    // Poll WiFi and lwIP stack
    cyw43_arch_poll();
}
