/**
 * WiFi Scanner
 * Scans for available networks to verify SSID visibility and security settings
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

static int scan_result(void *env, const cyw43_ev_scan_result_t *result) {
    if (result) {
        printf("  SSID: %-32s  RSSI: %4d  Chan: %3d  MAC: %02x:%02x:%02x:%02x:%02x:%02x  Sec: ",
               result->ssid, result->rssi, result->channel,
               result->bssid[0], result->bssid[1], result->bssid[2],
               result->bssid[3], result->bssid[4], result->bssid[5]);

        if (result->auth_mode == CYW43_AUTH_OPEN) {
            printf("Open\n");
        } else if (result->auth_mode == CYW43_AUTH_WPA_TKIP_PSK) {
            printf("WPA-TKIP\n");
        } else if (result->auth_mode == CYW43_AUTH_WPA2_AES_PSK) {
            printf("WPA2-AES\n");
        } else if (result->auth_mode == CYW43_AUTH_WPA2_MIXED_PSK) {
            printf("WPA2-Mixed\n");
        } else {
            printf("Other (%d)\n", result->auth_mode);
        }
    }
    return 0;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== WiFi Scanner ===\n\n");

    // Initialize CYW43
    printf("Initializing WiFi...\n");
    if (cyw43_arch_init()) {
        printf("FATAL: WiFi init failed\n");
        return 1;
    }
    printf("WiFi initialized\n\n");

    // Enable station mode
    cyw43_arch_enable_sta_mode();

    printf("Scanning for WiFi networks...\n");
    printf("(This takes about 3 seconds)\n\n");

    // Perform scan
    cyw43_wifi_scan_options_t scan_options = {0};
    int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result);
    if (err == 0) {
        printf("Scan started...\n\n");
    } else {
        printf("Failed to start scan: %d\n", err);
        return 1;
    }

    // Wait for scan to complete
    absolute_time_t scan_timeout = make_timeout_time_ms(10000);
    while (cyw43_wifi_scan_active(&cyw43_state) && absolute_time_diff_us(get_absolute_time(), scan_timeout) > 0) {
        cyw43_arch_poll();
        sleep_ms(100);
    }

    printf("\n=== Scan Complete ===\n");
    printf("\nLook for your SSID 'synctest' in the list above.\n");
    printf("Check:\n");
    printf("  - Is 'synctest' visible?\n");
    printf("  - What security mode does it show? (Should be WPA2-AES)\n");
    printf("  - Is the RSSI (signal) reasonable? (> -80 is good)\n");
    printf("  - Is it on a 2.4 GHz channel? (Channels 1-14)\n");
    printf("    (Pico W does NOT support 5 GHz - channels 36+)\n\n");

    cyw43_arch_deinit();
    return 0;
}
