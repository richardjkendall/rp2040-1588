/**
 * Absolute Minimal Test
 * Just stdio - no WiFi at all
 */

#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== SIMPLE TEST ===\n");
    printf("If you see this, USB serial works!\n\n");

    int count = 0;
    while (true) {
        printf("Count: %d\n", count++);
        sleep_ms(1000);
    }

    return 0;
}
