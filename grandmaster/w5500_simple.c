/**
 * Simple W5500 Driver Implementation
 */

#include "w5500_simple.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

// W5500 Common Registers (Block Select Bits = 0x00)
#define W5500_REG_MR       0x0000  // Mode Register
#define W5500_REG_GAR      0x0001  // Gateway Address (4 bytes)
#define W5500_REG_SUBR     0x0005  // Subnet Mask (4 bytes)
#define W5500_REG_SHAR     0x0009  // Source Hardware Address (6 bytes)
#define W5500_REG_SIPR     0x000F  // Source IP Address (4 bytes)
#define W5500_REG_PHYCFGR  0x002E  // PHY Configuration
#define W5500_REG_VERSIONR 0x0039  // Chip Version

// Socket 0 Registers (Block Select Bits = 0x08)
#define W5500_S0_MR        0x0000  // Socket 0 Mode Register
#define W5500_S0_CR        0x0001  // Socket 0 Command Register
#define W5500_S0_IR        0x0002  // Socket 0 Interrupt Register
#define W5500_S0_SR        0x0003  // Socket 0 Status Register
#define W5500_S0_PORT      0x0004  // Socket 0 Source Port (2 bytes)
#define W5500_S0_TX_FSR    0x0020  // Socket 0 TX Free Size (2 bytes)
#define W5500_S0_TX_RD     0x0022  // Socket 0 TX Read Pointer (2 bytes)
#define W5500_S0_TX_WR     0x0024  // Socket 0 TX Write Pointer (2 bytes)
#define W5500_S0_RX_RSR    0x0026  // Socket 0 RX Received Size (2 bytes)
#define W5500_S0_RX_RD     0x0028  // Socket 0 RX Read Pointer (2 bytes)
#define W5500_S0_RX_WR     0x002A  // Socket 0 RX Write Pointer (2 bytes)

// Block Select Bits (BSB)
#define W5500_BSB_COMMON   0x00  // Common register block
#define W5500_BSB_S0_REG   0x08  // Socket 0 register block
#define W5500_BSB_S0_TX    0x10  // Socket 0 TX buffer block
#define W5500_BSB_S0_RX    0x18  // Socket 0 RX buffer block

// Control Phase
#define W5500_CTRL_READ    (0x00 << 2)
#define W5500_CTRL_WRITE   (0x01 << 2)

// Socket Commands
#define W5500_CMD_OPEN     0x01
#define W5500_CMD_SEND     0x20
#define W5500_CMD_RECV     0x40

// Socket Mode
#define W5500_MODE_MACRAW  0x04

// Socket Status
#define W5500_STATUS_MACRAW 0x42

// Socket Interrupt Bits (W5500_S0_IR)
#define W5500_IR_RECV      0x04  // Receive interrupt
#define W5500_IR_SENDOK    0x10  // Send OK interrupt

// Socket Interrupt Mask Register
#define W5500_S0_IMR       0x002C  // Socket 0 Interrupt Mask Register

// Common Interrupt Registers
#define W5500_REG_SIMR     0x0018  // Socket Interrupt Mask Register (which sockets cause INT)

// SPI transfer with CS control
static void w5500_cs_select(void) {
    gpio_put(W5500_PIN_CS, 0);
    sleep_us(1);
}

static void w5500_cs_deselect(void) {
    sleep_us(1);
    gpio_put(W5500_PIN_CS, 1);
}

/**
 * Write to W5500 register
 *
 * @param addr Register address (16-bit)
 * @param bsb Block Select Bits
 * @param data Data to write
 */
static void w5500_write_byte(uint16_t addr, uint8_t bsb, uint8_t data) {
    uint8_t control = bsb | W5500_CTRL_WRITE;

    w5500_cs_select();

    // Send address (MSB first)
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr >> 8}, 1);
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr & 0xFF}, 1);

    // Send control byte
    spi_write_blocking(W5500_SPI_PORT, &control, 1);

    // Send data
    spi_write_blocking(W5500_SPI_PORT, &data, 1);

    w5500_cs_deselect();
}

/**
 * Read from W5500 register
 *
 * @param addr Register address (16-bit)
 * @param bsb Block Select Bits
 * @return Data read
 */
static uint8_t w5500_read_byte(uint16_t addr, uint8_t bsb) {
    uint8_t control = bsb | W5500_CTRL_READ;
    uint8_t data;

    w5500_cs_select();

    // Send address (MSB first)
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr >> 8}, 1);
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr & 0xFF}, 1);

    // Send control byte
    spi_write_blocking(W5500_SPI_PORT, &control, 1);

    // Read data
    spi_read_blocking(W5500_SPI_PORT, 0, &data, 1);

    w5500_cs_deselect();

    return data;
}

/**
 * Write 16-bit value (big-endian)
 */
static void w5500_write_word(uint16_t addr, uint8_t bsb, uint16_t value) {
    w5500_write_byte(addr, bsb, value >> 8);
    w5500_write_byte(addr + 1, bsb, value & 0xFF);
}

/**
 * Read 16-bit value (big-endian)
 */
static uint16_t w5500_read_word(uint16_t addr, uint8_t bsb) {
    uint16_t msb = w5500_read_byte(addr, bsb);
    uint16_t lsb = w5500_read_byte(addr + 1, bsb);
    return (msb << 8) | lsb;
}

/**
 * Write buffer to W5500 memory
 */
static void w5500_write_buffer(uint16_t addr, uint8_t bsb, const uint8_t *buffer, uint16_t len) {
    uint8_t control = bsb | W5500_CTRL_WRITE;

    w5500_cs_select();

    // Send address
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr >> 8}, 1);
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr & 0xFF}, 1);

    // Send control byte
    spi_write_blocking(W5500_SPI_PORT, &control, 1);

    // Send data
    spi_write_blocking(W5500_SPI_PORT, buffer, len);

    w5500_cs_deselect();
}

/**
 * Read buffer from W5500 memory
 */
static void w5500_read_buffer(uint16_t addr, uint8_t bsb, uint8_t *buffer, uint16_t len) {
    uint8_t control = bsb | W5500_CTRL_READ;

    w5500_cs_select();

    // Send address
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr >> 8}, 1);
    spi_write_blocking(W5500_SPI_PORT, &(uint8_t){addr & 0xFF}, 1);

    // Send control byte
    spi_write_blocking(W5500_SPI_PORT, &control, 1);

    // Read data
    spi_read_blocking(W5500_SPI_PORT, 0, buffer, len);

    w5500_cs_deselect();
}

bool w5500_simple_init(const uint8_t mac[6]) {
    printf("Initializing W5500 (simple driver)...\n");

    // Initialize SPI
    spi_init(W5500_SPI_PORT, 5 * 1000 * 1000);  // 5 MHz (safer for initial testing)
    gpio_set_function(W5500_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(W5500_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(W5500_PIN_MISO, GPIO_FUNC_SPI);

    // Initialize CS pin
    gpio_init(W5500_PIN_CS);
    gpio_set_dir(W5500_PIN_CS, GPIO_OUT);
    gpio_put(W5500_PIN_CS, 1);  // Deselect

    // Initialize RST pin
    gpio_init(W5500_PIN_RST);
    gpio_set_dir(W5500_PIN_RST, GPIO_OUT);

    // Hardware reset
    printf("  Resetting W5500...\n");
    gpio_put(W5500_PIN_RST, 0);
    sleep_ms(50);
    gpio_put(W5500_PIN_RST, 1);
    sleep_ms(200);  // Wait longer for chip to stabilize

    // Check chip version
    uint8_t version = w5500_get_version();
    printf("  W5500 version: 0x%02X\n", version);
    if (version != 0x04) {
        printf("  ERROR: Expected version 0x04, got 0x%02X\n", version);
        return false;
    }

    // Configure Mode Register (ensure clean state)
    // MR = 0x00: Normal operation, no special modes
    w5500_write_byte(W5500_REG_MR, W5500_BSB_COMMON, 0x00);

    // Set MAC address
    printf("  Setting MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    for (int i = 0; i < 6; i++) {
        w5500_write_byte(W5500_REG_SHAR + i, W5500_BSB_COMMON, mac[i]);
    }

    // Configure PHY (full duplex, 100Mbps, auto-negotiation)
    w5500_write_byte(W5500_REG_PHYCFGR, W5500_BSB_COMMON, 0b11111000);
    sleep_ms(50);

    // Open Socket 0 in MACRAW mode
    printf("  Opening MACRAW socket...\n");

    // Clear socket interrupt flags
    w5500_write_byte(W5500_S0_IR, W5500_BSB_S0_REG, 0xFF);

    // Initialize RX/TX buffer pointers to 0
    w5500_write_word(W5500_S0_TX_WR, W5500_BSB_S0_REG, 0);
    w5500_write_word(W5500_S0_TX_RD, W5500_BSB_S0_REG, 0);
    w5500_write_word(W5500_S0_RX_WR, W5500_BSB_S0_REG, 0);
    w5500_write_word(W5500_S0_RX_RD, W5500_BSB_S0_REG, 0);

    // Set socket mode to MACRAW
    w5500_write_byte(W5500_S0_MR, W5500_BSB_S0_REG, W5500_MODE_MACRAW);

    // Issue OPEN command
    w5500_write_byte(W5500_S0_CR, W5500_BSB_S0_REG, W5500_CMD_OPEN);

    // Wait for socket to open
    sleep_ms(10);
    uint8_t status = w5500_read_byte(W5500_S0_SR, W5500_BSB_S0_REG);
    if (status != W5500_STATUS_MACRAW) {
        printf("  ERROR: Socket status 0x%02X (expected 0x%02X)\n", status, W5500_STATUS_MACRAW);
        return false;
    }

    // Verify RX buffer is empty after init
    uint16_t rx_size = w5500_read_word(W5500_S0_RX_RSR, W5500_BSB_S0_REG);
    printf("  Initial RX buffer size: %u bytes\n", rx_size);

    printf("  W5500 ready (MACRAW mode)\n");
    return true;
}

bool w5500_link_up(void) {
    uint8_t phycfgr = w5500_read_byte(W5500_REG_PHYCFGR, W5500_BSB_COMMON);
    return (phycfgr & 0x01) != 0;  // Bit 0 = Link status
}

uint16_t w5500_rx_available(void) {
    return w5500_read_word(W5500_S0_RX_RSR, W5500_BSB_S0_REG);
}

bool w5500_recv_frame(uint8_t *buffer, uint16_t max_len, uint16_t *out_len) {
    // Check if data available
    uint16_t rx_size = w5500_rx_available();
    if (rx_size == 0 || rx_size < 2) {
        return false;
    }

    // Minimal logging - just RX activity
    // printf("RX: Processing %u bytes\n", rx_size);

    // In MACRAW mode, first 2 bytes are frame size
    uint16_t rd_ptr = w5500_read_word(W5500_S0_RX_RD, W5500_BSB_S0_REG);
    uint16_t wr_ptr = w5500_read_word(W5500_S0_RX_WR, W5500_BSB_S0_REG);

    // Read frame size (2 bytes, big-endian)
    // IMPORTANT: In MACRAW mode, this length INCLUDES the 2-byte header itself!
    uint8_t size_bytes[2];
    w5500_read_buffer(rd_ptr, W5500_BSB_S0_RX, size_bytes, 2);
    uint16_t total_len = (size_bytes[0] << 8) | size_bytes[1];
    rd_ptr += 2;

    // Actual frame length is total minus the 2-byte header
    if (total_len < 2) {
        printf("RX DROPPED: Invalid total_len=%u (too small)\n", total_len);
        return false;
    }
    uint16_t frame_len = total_len - 2;

    // Sanity check - only accept valid Ethernet frames
    if (frame_len > max_len || frame_len < 60 || frame_len > 1518) {
        // Invalid frame - print for debugging then discard
        printf("RX DROPPED: Invalid frame_len=%u (rx_size=%u, rd_ptr=0x%04X)\n",
               frame_len, rx_size, rd_ptr - 2);

        // Try to recover by clearing the entire RX buffer
        uint16_t wr_ptr_recovery = w5500_read_word(W5500_S0_RX_WR, W5500_BSB_S0_REG);
        printf("RX RECOVERY: Clearing buffer\n");
        w5500_write_word(W5500_S0_RX_RD, W5500_BSB_S0_REG, wr_ptr_recovery);
        w5500_write_byte(W5500_S0_CR, W5500_BSB_S0_REG, W5500_CMD_RECV);

        // Wait for RECV command to complete
        uint32_t recovery_timeout = 1000;
        while (recovery_timeout--) {
            uint8_t cr = w5500_read_byte(W5500_S0_CR, W5500_BSB_S0_REG);
            if (cr == 0) break;
            sleep_us(1);
        }
        if (recovery_timeout == 0xFFFFFFFF) {
            printf("WARN: Recovery RECV timeout\n");
        }

        printf("RX: Recovery complete\n");
        return false;
    }

    // Read frame data
    w5500_read_buffer(rd_ptr, W5500_BSB_S0_RX, buffer, frame_len);
    rd_ptr += frame_len;

    // Update read pointer
    w5500_write_word(W5500_S0_RX_RD, W5500_BSB_S0_REG, rd_ptr);

    // Issue RECV command to process
    w5500_write_byte(W5500_S0_CR, W5500_BSB_S0_REG, W5500_CMD_RECV);

    // CRITICAL: Wait for RECV command to complete
    // The S0_CR register is cleared by the W5500 when command completes
    uint32_t timeout = 1000;  // 1ms timeout should be plenty
    while (timeout--) {
        uint8_t cr = w5500_read_byte(W5500_S0_CR, W5500_BSB_S0_REG);
        if (cr == 0) {
            break;  // Command completed
        }
        sleep_us(1);
    }
    // Note: timeout will be 0xFFFFFFFF if it timed out (post-decrement from 0)
    if (timeout == 0xFFFFFFFF) {
        printf("WARN: RECV command timeout\n");
    }

    *out_len = frame_len;
    return true;
}

bool w5500_send_frame(const uint8_t *buffer, uint16_t len) {
    // Minimum Ethernet frame size is 60 bytes
    if (len < 60) {
        printf("TX ERROR: Frame too short (%u bytes)\n", len);
        return false;
    }

    // Check socket status
    uint8_t status = w5500_read_byte(W5500_S0_SR, W5500_BSB_S0_REG);
    if (status != W5500_STATUS_MACRAW) {
        printf("TX ERROR: Socket not in MACRAW mode (status=0x%02X)\n", status);
        return false;
    }

    // Check TX buffer space
    uint16_t free_size = w5500_read_word(W5500_S0_TX_FSR, W5500_BSB_S0_REG);
    if (free_size < len) {
        printf("TX ERROR: Buffer full (need %u, have %u)\n", len, free_size);
        return false;
    }

    // Get write pointer
    uint16_t wr_ptr = w5500_read_word(W5500_S0_TX_WR, W5500_BSB_S0_REG);

    // Write frame data
    w5500_write_buffer(wr_ptr, W5500_BSB_S0_TX, buffer, len);
    wr_ptr += len;

    // Update write pointer
    w5500_write_word(W5500_S0_TX_WR, W5500_BSB_S0_REG, wr_ptr);

    // Issue SEND command
    w5500_write_byte(W5500_S0_CR, W5500_BSB_S0_REG, W5500_CMD_SEND);

    // Wait for send completion
    uint32_t timeout = 10000;  // 10ms timeout
    while (timeout--) {
        uint8_t ir = w5500_read_byte(W5500_S0_IR, W5500_BSB_S0_REG);
        if (ir & 0x10) {  // SENDOK flag
            w5500_write_byte(W5500_S0_IR, W5500_BSB_S0_REG, 0x10);  // Clear flag
            return true;
        }
        if (ir & 0x08) {  // TIMEOUT flag
            w5500_write_byte(W5500_S0_IR, W5500_BSB_S0_REG, 0x08);  // Clear flag
            printf("TX ERROR: W5500 timeout flag set\n");
            return false;
        }
        sleep_us(1);
    }

    printf("TX ERROR: Send timeout (no SENDOK)\n");
    return false;
}

uint8_t w5500_get_version(void) {
    return w5500_read_byte(W5500_REG_VERSIONR, W5500_BSB_COMMON);
}

void w5500_dump_status(void) {
    printf("\n=== W5500 Status Dump ===\n");

    // Common registers
    uint8_t mr = w5500_read_byte(W5500_REG_MR, W5500_BSB_COMMON);
    uint8_t phycfgr = w5500_read_byte(W5500_REG_PHYCFGR, W5500_BSB_COMMON);
    printf("  MR      = 0x%02X\n", mr);
    printf("  PHYCFGR = 0x%02X (Link=%s)\n", phycfgr, (phycfgr & 0x01) ? "UP" : "DOWN");

    // Socket 0 registers
    uint8_t s0_mr = w5500_read_byte(W5500_S0_MR, W5500_BSB_S0_REG);
    uint8_t s0_sr = w5500_read_byte(W5500_S0_SR, W5500_BSB_S0_REG);
    uint8_t s0_ir = w5500_read_byte(W5500_S0_IR, W5500_BSB_S0_REG);
    printf("  S0_MR   = 0x%02X (Mode=%s)\n", s0_mr,
           s0_mr == 0x04 ? "MACRAW" : "OTHER");
    printf("  S0_SR   = 0x%02X (Status=%s)\n", s0_sr,
           s0_sr == 0x42 ? "MACRAW" : "OTHER");
    printf("  S0_IR   = 0x%02X\n", s0_ir);

    // Buffer status
    uint16_t tx_fsr = w5500_read_word(W5500_S0_TX_FSR, W5500_BSB_S0_REG);
    uint16_t tx_rd = w5500_read_word(W5500_S0_TX_RD, W5500_BSB_S0_REG);
    uint16_t tx_wr = w5500_read_word(W5500_S0_TX_WR, W5500_BSB_S0_REG);
    uint16_t rx_rsr = w5500_read_word(W5500_S0_RX_RSR, W5500_BSB_S0_REG);
    uint16_t rx_rd = w5500_read_word(W5500_S0_RX_RD, W5500_BSB_S0_REG);
    uint16_t rx_wr = w5500_read_word(W5500_S0_RX_WR, W5500_BSB_S0_REG);

    printf("  TX: Free=%u, RD=0x%04X, WR=0x%04X\n", tx_fsr, tx_rd, tx_wr);
    printf("  RX: Avail=%u, RD=0x%04X, WR=0x%04X\n", rx_rsr, rx_rd, rx_wr);

    // MAC address
    printf("  MAC: ");
    for (int i = 0; i < 6; i++) {
        uint8_t mac_byte = w5500_read_byte(W5500_REG_SHAR + i, W5500_BSB_COMMON);
        printf("%02X%s", mac_byte, i < 5 ? ":" : "\n");
    }

    printf("=========================\n\n");
}

/**
 * Enable W5500 socket interrupts for hardware timestamping
 */
void w5500_enable_interrupts(void) {
    // Enable Socket 0 to assert INT pin (SIMR register)
    w5500_write_byte(W5500_REG_SIMR, W5500_BSB_COMMON, 0x01);  // Enable Socket 0

    // Enable RECV and SENDOK interrupts on Socket 0 (Sn_IMR register)
    uint8_t mask = W5500_IR_RECV | W5500_IR_SENDOK;
    w5500_write_byte(W5500_S0_IMR, W5500_BSB_S0_REG, mask);

    printf("W5500 interrupts enabled (Socket 0: RECV | SENDOK)\n");
}

/**
 * Read and clear socket interrupt flags
 */
uint8_t w5500_read_clear_interrupts(void) {
    // Read interrupt register
    uint8_t ir = w5500_read_byte(W5500_S0_IR, W5500_BSB_S0_REG);

    // Clear interrupts by writing 1s back
    if (ir != 0) {
        w5500_write_byte(W5500_S0_IR, W5500_BSB_S0_REG, ir);
    }

    return ir;
}
