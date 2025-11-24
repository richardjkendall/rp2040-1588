/**
 * Minimal USB Serial Test
 * Tests if USB serial works at 250 MHz
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"

int main() {
    // Overclock first
    set_sys_clock_khz(250000, true);

    // Initialize USB serial
    stdio_init_all();

    // Wait for USB to enumerate
    sleep_ms(2000);

    printf("\n=== USB Serial Test ===\n");
    printf("System clock: %lu Hz\n", clock_get_hz(clk_sys));
    printf("USB should be working!\n\n");

    // Initialize LED on GPIO 25 (standard Pico LED)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    uint32_t count = 0;
    while (true) {
        printf("Count: %lu\n", count++);
        gpio_put(25, 1);
        sleep_ms(500);
        gpio_put(25, 0);
        sleep_ms(500);
    }

    return 0;
}
