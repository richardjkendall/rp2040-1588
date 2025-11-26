/**
 * Simple W5500 Driver - MACRAW Mode Only
 *
 * No library dependencies, minimal implementation for PTP grandmaster.
 * Uses MACRAW socket 0 for direct Ethernet frame access.
 */

#ifndef W5500_SIMPLE_H
#define W5500_SIMPLE_H

#include <stdint.h>
#include <stdbool.h>

// W5500 SPI configuration
#define W5500_SPI_PORT spi0
#define W5500_PIN_SCK  18
#define W5500_PIN_MOSI 19
#define W5500_PIN_MISO 16
#define W5500_PIN_CS   17
#define W5500_PIN_RST  20

// Maximum Ethernet frame size
#define W5500_MAX_FRAME_SIZE 1518

/**
 * Initialize W5500 in MACRAW mode
 *
 * @param mac MAC address for this device (6 bytes)
 * @return true if successful, false otherwise
 */
bool w5500_simple_init(const uint8_t mac[6]);

/**
 * Check if Ethernet link is up
 *
 * @return true if link is up, false otherwise
 */
bool w5500_link_up(void);

/**
 * Check if a frame is available to receive
 *
 * @return Number of bytes available, 0 if none
 */
uint16_t w5500_rx_available(void);

/**
 * Receive an Ethernet frame
 *
 * @param buffer Output buffer for frame data
 * @param max_len Maximum size of buffer
 * @param out_len Output: actual frame length received
 * @return true if frame received, false if no frame or error
 */
bool w5500_recv_frame(uint8_t *buffer, uint16_t max_len, uint16_t *out_len);

/**
 * Send an Ethernet frame
 *
 * @param buffer Frame data to send
 * @param len Frame length (must be >= 60 bytes for valid Ethernet)
 * @return true if sent successfully, false otherwise
 */
bool w5500_send_frame(const uint8_t *buffer, uint16_t len);

/**
 * Get W5500 chip version (should return 0x04 for W5500)
 *
 * @return Chip version register value
 */
uint8_t w5500_get_version(void);

/**
 * Dump W5500 status registers for debugging
 */
void w5500_dump_status(void);

#endif // W5500_SIMPLE_H
