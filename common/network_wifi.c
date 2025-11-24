/**
 * WiFi Network Backend (CYW43)
 *
 * Implementation of network_interface.h for Raspberry Pi Pico W WiFi.
 */

#ifndef USE_ETHERNET  // Only compile for WiFi builds

#include "network_interface.h"
#include "network_config.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

static bool network_initialized = false;

bool network_init(void) {
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

    while (to_ms_since_boot(get_absolute_time()) - start_ms < NETWORK_CONNECT_TIMEOUT_MS) {
        connect_result = cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            5000  // 5 second timeout per attempt
        );

        if (connect_result == 0) {
            break;
        }

        sleep_ms(1000);
    }

    if (connect_result != 0) {
        printf("ERROR: WiFi connection timeout\n");
        cyw43_arch_deinit();
        return false;
    }

    printf("WiFi connected\n");

#if NETWORK_USE_DHCP
    // Wait for DHCP to complete
    uint32_t dhcp_start = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - dhcp_start < 5000) {
        cyw43_arch_poll();
        if (netif_default && ip4_addr_get_u32(netif_ip4_addr(netif_default)) != 0) {
            break;
        }
        sleep_ms(100);
    }

    if (netif_default) {
        const ip4_addr_t *ip = netif_ip4_addr(netif_default);
        printf("DHCP IP: %s\n", ip4addr_ntoa(ip));
    }
#else
    // Configure static IP
    printf("Configuring static IP: %s\n", NETWORK_STATIC_IP);

    ip4_addr_t ip, netmask, gateway;
    if (!ip4addr_aton(NETWORK_STATIC_IP, &ip) ||
        !ip4addr_aton(NETWORK_STATIC_NETMASK, &netmask) ||
        !ip4addr_aton(NETWORK_STATIC_GATEWAY, &gateway)) {
        printf("ERROR: Invalid IP configuration\n");
        cyw43_arch_deinit();
        return false;
    }

    struct netif *netif = netif_default;
    if (netif == NULL) {
        printf("ERROR: No default network interface\n");
        cyw43_arch_deinit();
        return false;
    }

    netif_set_addr(netif, &ip, &netmask, &gateway);
    printf("Static IP configured: %s\n", ip4addr_ntoa(&ip));
#endif

    network_initialized = true;
    return true;
}

void network_poll(void) {
    cyw43_arch_poll();
}

bool network_is_connected(void) {
    if (!network_initialized) {
        return false;
    }

    int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
    // Accept both LINK_JOIN (1) and LINK_UP (3) as "connected"
    return (link_status == CYW43_LINK_JOIN || link_status == CYW43_LINK_UP);
}

ip_addr_t network_get_ip(void) {
    ip_addr_t addr;
    ip_addr_set_zero(&addr);

    if (network_initialized && netif_default) {
        ip_addr_copy(addr, *netif_ip4_addr(netif_default));
    }
    return addr;
}

void network_get_ip_str(char *buffer, size_t buffer_len) {
    if (!network_initialized || netif_default == NULL) {
        snprintf(buffer, buffer_len, "0.0.0.0");
        return;
    }

    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    snprintf(buffer, buffer_len, "%s", ip4addr_ntoa(ip));
}

void network_get_mac(uint8_t mac[6]) {
    if (network_initialized) {
        cyw43_hal_get_mac(CYW43_ITF_STA, mac);
    } else {
        memset(mac, 0, 6);
    }
}

#endif // !USE_ETHERNET
