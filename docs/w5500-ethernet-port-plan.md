# Plan: Port Grandmaster & Slave to W55RP20 (Wired Ethernet)

## Target Board

**W55RP20-EVB-PICO** (SparkFun evaluation board)
- Integrated W55RP20 chip (RP2040 + W5500 in single package)
- 10/100 Ethernet PHY with built-in RJ45
- Reserved pins for Ethernet: GPIO 17, 20-25
- Pico-compatible footprint

## Affected Modules

| Module | Uses Networking | Action Required |
|--------|-----------------|-----------------|
| `grandmaster/` | Yes (WiFi) | Update to use abstraction |
| `slave/` | Yes (WiFi) | Update to use abstraction |
| `wifi_test/` | Yes (WiFi) | Leave as-is (test utility) |
| `final_phase_meter/` | No | No changes |
| `logger/` | No | No changes |

## Current State

Both grandmaster and slave use:
- CYW43 WiFi + lwIP in polling mode
- Raw lwIP UDP API (`udp_pcb`, `udp_bind`, `udp_sendto`)
- `cyw43_hal_get_mac()` for PTP clock identity
- Board type `pico_w`
- Separate `wifi_init.c/h` files (duplicated code)

**Good news**: Both WiFi and W5500 use lwIP, so PTP protocol code remains unchanged.

## Approach

Create a shared network abstraction layer in `common/` with compile-time backend selection.

```
┌─────────────────┐     ┌─────────────────┐
│ ptp_grandmaster │     │   ptp_slave     │
└────────┬────────┘     └────────┬────────┘
         │                       │
         └───────────┬───────────┘
                     ▼
┌─────────────────────────────────────────┐
│      common/network_interface.h         │
│   network_init() / poll() / get_mac()   │
└─────────────────┬───────────────────────┘
                  │
        ┌─────────┴─────────┐
        ▼                   ▼
┌───────────────┐   ┌───────────────┐
│ common/       │   │ common/       │
│ network_wifi.c│   │ network_w5500.c│
│   (CYW43)     │   │  (W55RP20)    │
└───────────────┘   └───────────────┘
```

## Implementation Steps

### Phase 1: Create Network Abstraction in common/

1. **Create `common/network_interface.h`**
   ```c
   #ifndef NETWORK_INTERFACE_H
   #define NETWORK_INTERFACE_H

   #include <stdbool.h>
   #include "lwip/ip_addr.h"

   bool network_init(void);
   void network_poll(void);
   bool network_is_connected(void);
   ip_addr_t network_get_ip(void);
   void network_get_mac(uint8_t mac[6]);

   #endif
   ```

2. **Create `common/network_wifi.c`**
   - Consolidate from `grandmaster/wifi_init.c` and `slave/wifi_init.c`
   - Implement the new interface
   - Keep existing CYW43 logic

3. **Create `common/network_config.h`**
   ```c
   // Common
   #define NETWORK_CONNECT_TIMEOUT_MS 30000

   // WiFi-specific (only used when USE_ETHERNET is not defined)
   #ifndef USE_ETHERNET
   #define WIFI_SSID "synctest"
   #define WIFI_PASSWORD "ieee1588v2ptp"
   #endif

   // Ethernet-specific
   #ifdef USE_ETHERNET
   #define ETH_MAC {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}
   #define USE_DHCP 1
   #endif
   ```

4. **Update `ptp_grandmaster.c`**
   - Replace `cyw43_hal_get_mac()` with `network_get_mac()`

5. **Update `ptp_slave.c`**
   - Replace `cyw43_hal_get_mac()` with `network_get_mac()`

6. **Update `grandmaster/main.c`**
   - Replace `wifi_*` calls with `network_*` calls
   - Remove `#include "wifi_init.h"`

7. **Update `slave/main.c`**
   - Replace `wifi_*` calls with `network_*` calls
   - Remove `#include "wifi_init.h"`

8. **Delete old WiFi files**
   - `grandmaster/wifi_init.c`, `grandmaster/wifi_init.h`
   - `slave/wifi_init.c`, `slave/wifi_init.h`
   - `grandmaster/wifi_config.h`, `slave/wifi_config.h`

### Phase 2: W5500 Backend

9. **Create `common/network_w5500.c`**
    - Initialize W5500 via SPI (internal to W55RP20)
    - Configure lwIP netif for W5500
    - Implement polling for packet reception
    - MAC address: read from W5500 or configure static

10. **Add W5500 driver as submodule or external dependency**
    - Use WIZnet's official SDK: https://github.com/WIZnet-ioNIC/WIZnet-PICO-C
    - Or ioLibrary_Driver with lwIP bindings

### Phase 3: CMake Build System

11. **Modify root `CMakeLists.txt`**
    ```cmake
    option(USE_ETHERNET "Use W5500 Ethernet instead of WiFi" OFF)

    if(USE_ETHERNET)
        set(PICO_BOARD pico CACHE STRING "Board type")
    else()
        set(PICO_BOARD pico_w CACHE STRING "Board type")
    endif()

    # Update common library
    add_library(common INTERFACE)
    target_sources(common INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/common/discipline.c
        ${CMAKE_CURRENT_LIST_DIR}/common/ptp_protocol.c
    )
    # Network sources added conditionally in subdirectory CMakeLists
    ```

12. **Create `common/CMakeLists.txt`** (new file)
    ```cmake
    # Network abstraction library
    add_library(network INTERFACE)

    if(USE_ETHERNET)
        target_sources(network INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/network_w5500.c
        )
        target_compile_definitions(network INTERFACE USE_ETHERNET=1)
        target_link_libraries(network INTERFACE
            pico_stdlib
            hardware_spi
            # W5500 driver library
            # pico_lwip_nosys or similar
        )
    else()
        target_sources(network INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/network_wifi.c
        )
        target_link_libraries(network INTERFACE
            pico_cyw43_arch_lwip_poll
        )
    endif()

    target_include_directories(network INTERFACE ${CMAKE_CURRENT_LIST_DIR})
    ```

13. **Modify `grandmaster/CMakeLists.txt`**
    ```cmake
    # Remove: wifi_init.c from sources
    # Remove: pico_cyw43_arch_lwip_poll from direct linking
    # Add: network to target_link_libraries
    target_link_libraries(grandmaster
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_uart
        hardware_timer
        hardware_irq
        common
        network  # New: provides WiFi or W5500 based on USE_ETHERNET
    )
    ```

14. **Modify `slave/CMakeLists.txt`**
    - Same changes as grandmaster

### Phase 4: Testing & Validation

15. **Test WiFi build (regression)**
    ```bash
    mkdir build-wifi && cd build-wifi
    cmake .. -DUSE_ETHERNET=OFF
    make grandmaster slave
    ```
    - Deploy to Pico W
    - Verify PTP sync still works

16. **Test Ethernet build**
    ```bash
    mkdir build-eth && cd build-eth
    cmake .. -DUSE_ETHERNET=ON
    make grandmaster slave
    ```
    - Deploy to W55RP20-EVB-PICO
    - Verify link detection, DHCP
    - Verify PTP sync works

17. **Compare sync quality**
    - Use `final_phase_meter` or `logger` to measure phase difference
    - Compare WiFi vs Ethernet jitter

## W55RP20 Specific Notes

### Reserved GPIO Pins
Do NOT use these for GPS/PPS - they're used internally for Ethernet:
- GPIO 17, 20, 21, 22, 23, 24, 25

### Current Pin Usage (grandmaster/slave)
- GPIO 0, 1: UART (GPS data) - **OK**
- GPIO 2: PPS input - **OK**
- GPIO 3: 100 PPS output - **OK**

No conflicts with W55RP20 reserved pins.

### W5500 Internal Connection
The W55RP20 connects W5500 to RP2040 internally via SPI. Typical mapping:
- SPI0 on GPIO 16-19 (check WIZnet examples for exact config)

## Build Commands

```bash
# WiFi build (Pico W)
mkdir build-wifi && cd build-wifi
cmake .. -DUSE_ETHERNET=OFF
make grandmaster slave

# Ethernet build (W55RP20)
mkdir build-eth && cd build-eth
cmake .. -DUSE_ETHERNET=ON
make grandmaster slave
```

## File Changes Summary

### New Files
- `common/network_interface.h`
- `common/network_wifi.c`
- `common/network_w5500.c`
- `common/network_config.h`
- `common/CMakeLists.txt`

### Modified Files
- `CMakeLists.txt` (root)
- `grandmaster/CMakeLists.txt`
- `grandmaster/main.c`
- `grandmaster/ptp_grandmaster.c`
- `slave/CMakeLists.txt`
- `slave/main.c`
- `slave/ptp_slave.c`

### Deleted Files
- `grandmaster/wifi_init.c`
- `grandmaster/wifi_init.h`
- `grandmaster/wifi_config.h`
- `slave/wifi_init.c`
- `slave/wifi_init.h`
- `slave/wifi_config.h`

## Expected Benefits

- Lower and more consistent network latency
- No WiFi interference or contention
- Potentially better sync precision
- Deterministic packet timing
- Consolidated network code (no duplication between grandmaster/slave)

## Resources

- WIZnet W55RP20 examples: https://github.com/WIZnet-ioNIC/WIZnet-PICO-C
- W5500 lwIP driver: https://github.com/Wiznet/ioLibrary_Driver
- W55RP20 datasheet: via WIZnet or SparkFun
