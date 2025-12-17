# PTP Phase Offset Telemetry Server

Simple Go server for receiving and displaying PTP phase offset measurements from Pico W measurement device.

## Features

- TCP server (port 5000) for Pico W connections
- Web UI (port 8080) with real-time statistics
- WebSocket updates to browser
- Optional JSONL file logging
- Single binary, no dependencies

## Quick Start

```bash
# Install dependencies
go mod download

# Run server
go run .

# Or build binary
go build -o telemetry-server
./telemetry-server
```

## Usage

```bash
# Default ports (TCP 5000, HTTP 8080)
./telemetry-server

# Custom ports
./telemetry-server -tcp 5000 -http 8080

# Enable file logging
./telemetry-server -log measurements.jsonl
```

## Web UI

Open http://localhost:8080 in your browser to see real-time statistics:

- Phase offset statistics (mean, stddev, min, max)
- GPS crystal error and scale factor
- Telemetry status (batches received, buffer drops)
- Connection status indicator

## Pico W Configuration

Update your Pico W measurement device WiFi configuration:

```c
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
#define TELEMETRY_SERVER_IP "192.168.1.100"  // IP of machine running this server
```

## Data Format

Measurements are received as newline-delimited JSON batches:

```json
{
  "device": "measurement_device",
  "batch_seq": 42,
  "count": 60,
  "measurements": [
    {
      "seq": 2501,
      "timestamp_us": 150012345678,
      "phase_ns": 245.3,
      "gm_to_slave_ns": 245.3,
      "slave_to_gm_ns": 987654.2,
      "scale_factor": 1.000012,
      "gm_first": true,
      "crystal_error_ns": 145
    }
  ],
  "stats": {
    "buffer_available": 850,
    "buffer_dropped": 0
  }
}
```

## Architecture

```
Pico W (TCP client)
    ↓
TCP Server :5000
    ↓
Statistics Engine (Welford's algorithm)
    ↓
WebSocket Broadcaster
    ↓
Browser (Web UI :8080)
```
