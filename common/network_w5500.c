/**
 * W5500 Ethernet Network Backend
 *
 * Implementation of network_interface.h for W55RP20 / W5500 Ethernet.
 * Uses WIZnet-PICO-LWIP-C library for lwIP integration.
 */

#ifdef USE_ETHERNET

#include "network_interface.h"
#include "network_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

// WIZnet driver includes
#include "wizchip_conf.h"
#include "socket.h"
#include "w5x00_spi.h"
#include "w5x00_lwip.h"

// lwIP includes
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/ip4_addr.h"

// Socket for MACRAW mode
#define SOCKET_MACRAW 0

// Ethernet MTU
#define ETHERNET_MTU 1500

// MAC address from config
static uint8_t eth_mac[6] = ETH_MAC_ADDR;

// Network interface
static struct netif g_netif;

// Receive buffer
static uint8_t *rx_buffer = NULL;

// State
static bool network_initialized = false;

// External MAC variable used by w5x00_lwip.c
extern uint8_t mac[6];

bool network_init(void) {
    printf("Initializing W5500 Ethernet...\n");

    // Copy our MAC to the global used by w5x00_lwip.c
    memcpy(mac, eth_mac, 6);

    // Allocate receive buffer
    rx_buffer = malloc(ETHERNET_MTU);
    if (!rx_buffer) {
        printf("ERROR: Failed to allocate rx buffer\n");
        return false;
    }

    // Initialize SPI for W5500
    wizchip_spi_initialize();
    wizchip_cris_initialize();

    // Reset and initialize W5500
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    // Set MAC address in W5500
    setSHAR(eth_mac);

    // Reset PHY
    ctlwizchip(CW_RESET_PHY, 0);

    printf("W5500 initialized, MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);

    // Initialize lwIP
    lwip_init();

    // Add network interface
    netif_add(&g_netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, netif_initialize, netif_input);
    g_netif.name[0] = 'e';
    g_netif.name[1] = '0';

    // Set callbacks
    netif_set_link_callback(&g_netif, netif_link_callback);
    netif_set_status_callback(&g_netif, netif_status_callback);

    // Open MACRAW socket
    int8_t ret = socket(SOCKET_MACRAW, Sn_MR_MACRAW, 0, 0x00);
    if (ret != SOCKET_MACRAW) {
        printf("ERROR: MACRAW socket open failed (%d)\n", ret);
        return false;
    }

    // Set as default interface and bring up
    netif_set_default(&g_netif);
    netif_set_link_up(&g_netif);
    netif_set_up(&g_netif);

#if NETWORK_USE_DHCP
    // Start DHCP
    printf("Starting DHCP...\n");
    dhcp_start(&g_netif);

    // Wait for DHCP to complete (with timeout)
    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    while (to_ms_since_boot(get_absolute_time()) - start_ms < NETWORK_CONNECT_TIMEOUT_MS) {
        network_poll();

        if (dhcp_supplied_address(&g_netif)) {
            printf("DHCP complete, IP: %s\n", ip4addr_ntoa(netif_ip4_addr(&g_netif)));
            break;
        }
        sleep_ms(100);
    }

    if (!dhcp_supplied_address(&g_netif)) {
        printf("WARNING: DHCP timeout, continuing without IP\n");
    }
#else
    // Static IP configuration
    printf("Configuring static IP: %s\n", NETWORK_STATIC_IP);

    ip4_addr_t ip, netmask, gateway;
    if (!ip4addr_aton(NETWORK_STATIC_IP, &ip) ||
        !ip4addr_aton(NETWORK_STATIC_NETMASK, &netmask) ||
        !ip4addr_aton(NETWORK_STATIC_GATEWAY, &gateway)) {
        printf("ERROR: Invalid IP configuration\n");
        return false;
    }

    netif_set_addr(&g_netif, &ip, &netmask, &gateway);
    printf("Static IP configured: %s\n", ip4addr_ntoa(&ip));
#endif

    network_initialized = true;
    printf("W5500 network ready\n");
    return true;
}

void network_poll(void) {
    if (!network_initialized && !rx_buffer) {
        return;
    }

    // Check for received packets
    uint16_t pack_len = 0;
    getsockopt(SOCKET_MACRAW, SO_RECVBUF, &pack_len);

    if (pack_len > 0) {
        pack_len = recv_lwip(SOCKET_MACRAW, rx_buffer, pack_len);

        if (pack_len > 0) {
            // Allocate pbuf and pass to lwIP
            struct pbuf *p = pbuf_alloc(PBUF_RAW, pack_len, PBUF_POOL);
            if (p != NULL) {
                pbuf_take(p, rx_buffer, pack_len);

                if (g_netif.input(p, &g_netif) != ERR_OK) {
                    pbuf_free(p);
                }
            }
        }
    }

    // Process lwIP timers
    sys_check_timeouts();
}

bool network_is_connected(void) {
    if (!network_initialized) {
        return false;
    }

    // Check PHY link status
    uint8_t phylink = 0;
    ctlwizchip(CW_GET_PHYLINK, &phylink);

    return (phylink == PHY_LINK_ON);
}

ip_addr_t network_get_ip(void) {
    ip_addr_t addr;
    ip_addr_set_zero(&addr);

    if (network_initialized) {
        ip_addr_copy(addr, *netif_ip4_addr(&g_netif));
    }
    return addr;
}

void network_get_ip_str(char *buffer, size_t buffer_len) {
    if (!network_initialized) {
        snprintf(buffer, buffer_len, "0.0.0.0");
        return;
    }

    const ip4_addr_t *ip = netif_ip4_addr(&g_netif);
    snprintf(buffer, buffer_len, "%s", ip4addr_ntoa(ip));
}

void network_get_mac(uint8_t mac_out[6]) {
    memcpy(mac_out, eth_mac, 6);
}

#endif // USE_ETHERNET
