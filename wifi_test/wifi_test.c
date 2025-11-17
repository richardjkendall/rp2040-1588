/**
 * WiFi Test Program
 *
 * Minimal test to prove CYW43 and lwIP stack work.
 * Tests:
 * - CYW43 initialization
 * - WiFi connection
 * - Static IP configuration
 * - Onboard LED control (via CYW43)
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

// WiFi credentials (from wifi_config.h values)
#define WIFI_SSID "synctest"
#define WIFI_PASSWORD "ieee1588v2ptp"

// Static IP
#define STATIC_IP_ADDR "192.168.1.100"
#define STATIC_NETMASK "255.255.255.0"
#define STATIC_GATEWAY "192.168.1.1"

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== WiFi Test Program ===\n\n");

    // Step 1: Initialize CYW43 driver
    printf("Step 1: Initializing CYW43 WiFi driver...\n");
    printf("  - Checking board type...\n");
    printf("  - Initializing PIO and DMA...\n");
    printf("  - Loading CYW43 firmware...\n");

    int init_result = cyw43_arch_init();
    if (init_result != 0) {
        printf("FATAL: CYW43 init failed with error code: %d\n", init_result);
        printf("\nPossible causes:\n");
        printf("  - Hardware issue (unlikely)\n");
        printf("  - lwIP memory allocation failed\n");
        printf("  - PIO/DMA resource conflict\n");
        printf("  - Wrong board configuration\n");
        printf("\nThe board appears to be functional (USB serial works)\n");
        printf("but the CYW43 WiFi chip failed to initialize.\n");
        while (1) {
            sleep_ms(1000);
        }
    }
    printf("SUCCESS: CYW43 initialized\n\n");

    // Step 2: Blink onboard LED to prove CYW43 works
    printf("Step 2: Blinking onboard LED (5 times)...\n");
    for (int i = 0; i < 5; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(250);
    }
    printf("SUCCESS: LED blink complete\n\n");

    // Step 3: Enable WiFi station mode
    printf("Step 3: Enabling WiFi station mode...\n");
    cyw43_arch_enable_sta_mode();
    printf("SUCCESS: Station mode enabled\n\n");

    // Step 4: Connect to WiFi
    printf("Step 4: Connecting to WiFi SSID: %s\n", WIFI_SSID);
    printf("(This may take up to 30 seconds...)\n");

    int connect_result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID,
        WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK,
        30000
    );

    if (connect_result != 0) {
        printf("FATAL: WiFi connection failed (error %d)\n", connect_result);
        printf("Check:\n");
        printf("  - SSID is correct\n");
        printf("  - Password is correct\n");
        printf("  - Router is 2.4 GHz (not 5 GHz)\n");
        printf("  - Router is in range\n");
        while (1) {
            sleep_ms(1000);
        }
    }
    printf("SUCCESS: WiFi connected\n\n");

    // Step 5: Configure static IP
    printf("Step 5: Configuring static IP: %s\n", STATIC_IP_ADDR);

    ip4_addr_t ip, netmask, gateway;
    IP4_ADDR(&ip, 192, 168, 1, 100);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gateway, 192, 168, 1, 1);

    netif_set_addr(netif_default, &ip, &netmask, &gateway);

    printf("SUCCESS: Static IP configured\n");
    printf("  IP:      %s\n", ip4addr_ntoa(&ip));
    printf("  Netmask: %s\n", ip4addr_ntoa(&netmask));
    printf("  Gateway: %s\n", ip4addr_ntoa(&gateway));
    printf("\n");

    // Step 6: Main loop - blink LED and print status
    printf("=== WiFi Test PASSED ===\n");
    printf("Entering status monitoring loop...\n\n");

    uint32_t last_status_ms = 0;
    bool led_state = false;

    while (true) {
        // Poll WiFi stack (required for poll mode)
        cyw43_arch_poll();

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // Print status every 5 seconds
        if (now_ms - last_status_ms >= 5000) {
            last_status_ms = now_ms;

            int link_status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
            const char *status_str;

            switch (link_status) {
                case CYW43_LINK_UP:
                    status_str = "LINK UP";
                    break;
                case CYW43_LINK_DOWN:
                    status_str = "LINK DOWN";
                    break;
                case CYW43_LINK_JOIN:
                    status_str = "JOINING";
                    break;
                case CYW43_LINK_FAIL:
                    status_str = "LINK FAIL";
                    break;
                case CYW43_LINK_NONET:
                    status_str = "NO NETWORK";
                    break;
                case CYW43_LINK_BADAUTH:
                    status_str = "BAD AUTH";
                    break;
                default:
                    status_str = "UNKNOWN";
            }

            printf("WiFi Status: %s, IP: %s\n",
                   status_str,
                   ip4addr_ntoa(netif_ip4_addr(netif_default)));

            // Blink LED to show we're alive
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
        }

        sleep_ms(10);
    }

    return 0;
}
