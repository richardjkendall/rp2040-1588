# WiFi Telemetry Improvements Plan

## Problem Analysis

Based on your reported issues:
1. **Fails to send batches** - Connection becomes unstable
2. **Gets into stuck state** - Stops sending entirely
3. **Server restart breaks everything** - Can't reconnect gracefully
4. **Too inflexible** - Server restart requires Pico restart

## Root Causes Identified

### 1. **Timing Conflict (Critical)**

**Issue**: Keepalive mismatch causes connection drops

- **Pico** (telemetry.h:12): Sends keepalive after 60s of inactivity
- **Server** (server.go:66): Has 30s read deadline
- **Result**: Server kills connection after 30s, before Pico sends keepalive!

```
Time 0s:   Last batch sent
Time 30s:  Server timeout → Connection closed by server
Time 60s:  Pico tries to send keepalive → Connection already dead
```

**Fix**: Reduce Pico keepalive to 15-20s OR increase server timeout to 120s

### 2. **Port Mismatch**

- telemetry.h defines `TELEMETRY_SERVER_PORT 5001`
- Server defaults to port 5000 (main.go:12)
- Plan document says 5000

**Fix**: Standardize on port 5000 or 5001 everywhere

### 3. **No TCP Error Retry Logic**

**Issue**: Any `tcp_write()` error immediately breaks connection

```c
// telemetry.c:203-214
err = tcp_write(pcb, json_buffer, json_len, TCP_WRITE_FLAG_COPY);
if (err == ERR_OK) {
    // success
} else {
    printf("[Telemetry] TCP write failed: %d\n", err);
    g_send_failures++;
    g_connected = false;  // ← Immediately gives up!
    break;
}
```

**Problem**: `tcp_write()` can fail with:
- `ERR_MEM` (out of TCP buffers) - **TRANSIENT** - should retry!
- `ERR_WOULDBLOCK` (send queue full) - **TRANSIENT** - should retry!
- `ERR_CONN` (connection closed) - **FATAL** - should disconnect

**Fix**: Distinguish transient from fatal errors, retry transient ones

### 4. **Server Restart Not Detected**

**Issue**: When server restarts:
1. Server closes all connections
2. Pico's TCP stack doesn't immediately know (takes up to 30s)
3. Pico tries to send → gets error → disconnects
4. Takes 5s to reconnect (TELEMETRY_RECONNECT_DELAY_MS)
5. Total downtime: 30-35 seconds

**Fix**: Faster disconnect detection + immediate reconnect attempt

### 5. **No Connection State Recovery**

**Issue**: If connection breaks mid-batch:
- Partial batch is lost
- No way to know what server received
- Batch sequence numbers continue, but gaps appear

**Fix**: Could add batch ACKs or make it acceptable to have gaps

### 6. **Excessive Logging Noise**

**Issue**: Every batch prints, every error prints
- Makes console output unusable during normal operation
- Hard to see actual measurement data

**Fix**: Reduce logging verbosity, add debug levels

## Improvement Plan

### Phase 1: Quick Fixes (High Impact, Low Risk)

#### Fix 1.1: Align Keepalive/Timeout Timing

**Pico Changes** (telemetry.h):
```c
#define TELEMETRY_KEEPALIVE_MS 20000     // Send keepalive every 20s (was 60s)
```

**Server Changes** (server.go:66):
```go
// Set read deadline to 60s (was 30s) - allows for batch interval + margin
conn.SetReadDeadline(time.Now().Add(60 * time.Second))
```

**Rationale**:
- Batches sent every ~60s (at 1Hz measurement rate)
- Keepalive at 20s ensures server sees activity every 20s
- Server timeout at 60s gives plenty of margin

#### Fix 1.2: Standardize Port

**Change**: Use port 5001 consistently

**Pico** - Already correct in telemetry.h:
```c
#define TELEMETRY_SERVER_PORT 5001
```

**Server** (main.go:12):
```go
tcpPort  = flag.Int("tcp", 5001, "TCP port for Pico connections")  // was 5000
```

**README.md**: Update all examples to use 5001

#### Fix 1.3: Add TCP Write Retry Logic

**Pico Changes** (telemetry.c):

```c
// Replace immediate failure with retry logic
#define MAX_SEND_RETRIES 3
#define SEND_RETRY_DELAY_MS 100

static bool tcp_send_with_retry(struct tcp_pcb *pcb, const char *data, int len) {
    for (int retry = 0; retry < MAX_SEND_RETRIES; retry++) {
        err_t err = tcp_write(pcb, data, len, TCP_WRITE_FLAG_COPY);

        if (err == ERR_OK) {
            tcp_output(pcb);  // Flush
            return true;
        }

        // Check if error is transient or fatal
        if (err == ERR_MEM || err == ERR_WOULDBLOCK) {
            // Transient error - retry after delay
            printf("[Telemetry] Transient error %d, retry %d/%d\n",
                   err, retry + 1, MAX_SEND_RETRIES);
            cyw43_arch_poll();  // Process WiFi stack
            sleep_ms(SEND_RETRY_DELAY_MS);
            continue;
        }

        // Fatal error - don't retry
        printf("[Telemetry] Fatal TCP error: %d\n", err);
        return false;
    }

    // All retries exhausted
    printf("[Telemetry] Send failed after %d retries\n", MAX_SEND_RETRIES);
    return false;
}
```

Then replace both `tcp_write()` calls with:
```c
if (tcp_send_with_retry(pcb, json_buffer, json_len)) {
    g_batches_sent++;
    // ... success handling
} else {
    g_send_failures++;
    g_connected = false;
    break;
}
```

#### Fix 1.4: Reduce Logging Noise

**Pico Changes** (telemetry.c):

Add verbosity levels:
```c
#define TELEMETRY_LOG_ERRORS  1  // Always log errors
#define TELEMETRY_LOG_INFO    2  // Log connections/disconnections
#define TELEMETRY_LOG_DEBUG   3  // Log every batch

#define TELEMETRY_LOG_LEVEL TELEMETRY_LOG_INFO  // Default

#define TEL_LOG_ERROR(...)  printf("[Telemetry] " __VA_ARGS__)
#define TEL_LOG_INFO(...)   if (TELEMETRY_LOG_LEVEL >= 2) printf("[Telemetry] " __VA_ARGS__)
#define TEL_LOG_DEBUG(...)  if (TELEMETRY_LOG_LEVEL >= 3) printf("[Telemetry] " __VA_ARGS__)
```

Then replace:
```c
// Was: printf("[Telemetry] Sent batch #%u...\n", ...)
TEL_LOG_DEBUG("Sent batch #%u (%d measurements)\n", ...);

// Keep important ones:
TEL_LOG_INFO("WiFi connected\n");
TEL_LOG_INFO("TCP connected to %s:%u\n", ...);
TEL_LOG_ERROR("TCP write failed: %d\n", err);
```

### Phase 2: Server Improvements (Medium Priority)

#### Fix 2.1: Graceful Connection Handling

**Server Changes** (server.go):

Add TCP keepalive to detect dead connections faster:

```go
func (s *TCPServer) handleConnection(conn net.Conn) {
    defer conn.Close()
    defer log.Printf("Connection closed from %s", conn.RemoteAddr())

    // Enable TCP keepalive for faster disconnect detection
    if tcpConn, ok := conn.(*net.TCPConn); ok {
        tcpConn.SetKeepAlive(true)
        tcpConn.SetKeepAlivePeriod(10 * time.Second)
    }

    reader := bufio.NewReader(conn)

    for {
        // Increased timeout to 60s (was 30s)
        conn.SetReadDeadline(time.Now().Add(60 * time.Second))

        // ... rest of handler
    }
}
```

#### Fix 2.2: Connection State Visibility

**Server Changes** (stats.go):

Add connection tracking:

```go
type Statistics struct {
    // ... existing fields ...

    Connected       bool      `json:"connected"`
    LastConnection  time.Time `json:"last_connection"`
    ConnectionCount uint32    `json:"connection_count"`
}

func (s *Statistics) SetConnected(connected bool) {
    s.mu.Lock()
    defer s.mu.Unlock()

    s.Connected = connected
    if connected {
        s.ConnectionCount++
        s.LastConnection = time.Now()
    }
}
```

Update server.go:
```go
func (s *TCPServer) handleConnection(conn net.Conn) {
    defer conn.Close()
    defer func() {
        log.Printf("Connection closed from %s", conn.RemoteAddr())
        s.stats.SetConnected(false)  // Mark as disconnected
    }()

    log.Printf("New connection from %s", conn.RemoteAddr())
    s.stats.SetConnected(true)  // Mark as connected

    // ... rest
}
```

### Phase 3: Protocol Improvements (Lower Priority, Higher Effort)

#### Fix 3.1: Batch Acknowledgment (Optional)

**Why**: Know if server received data

**Approach**: Server sends ACK after each batch

**Pico Changes**:
```c
// After sending batch, wait for ACK (with timeout)
static bool wait_for_ack(struct tcp_pcb *pcb, uint32_t batch_seq, uint32_t timeout_ms) {
    // Poll for ACK message: {"ack": batch_seq}
    // If timeout, assume ACK lost (acceptable)
    // If wrong seq, server missed something
}
```

**Server Changes**:
```go
// After processing batch, send ACK
ack := fmt.Sprintf("{\"ack\":%d}\n", batch.BatchSeq)
conn.Write([]byte(ack))
```

**Complexity**: Medium - changes protocol, need to handle bidirectional communication

**Value**: Low - not critical for this use case

#### Fix 3.2: Compression (Optional)

**Why**: Reduce bandwidth, faster sends

**Approach**: gzip compress JSON before sending

**Pico Changes**:
```c
// Use pico-sdk's zlib
#include "pico/zlib.h"

static int compress_json(const char *json, int json_len, char *compressed, int compressed_max) {
    // Compress JSON
    // Prepend 4-byte uncompressed length
    // Return compressed length
}
```

**Server Changes**:
```go
// Detect compressed data (check for gzip magic bytes)
// Decompress before JSON parsing
```

**Complexity**: Medium

**Value**: Medium - 70-80% size reduction

### Phase 4: Robustness Improvements

#### Fix 4.1: Connection State Machine

**Pico Changes** (telemetry.c):

Add explicit state tracking:

```c
typedef enum {
    TEL_STATE_DISCONNECTED,
    TEL_STATE_WIFI_CONNECTING,
    TEL_STATE_WIFI_CONNECTED,
    TEL_STATE_TCP_CONNECTING,
    TEL_STATE_TCP_CONNECTED,
    TEL_STATE_SENDING,
    TEL_STATE_ERROR
} telemetry_state_t;

static volatile telemetry_state_t g_state = TEL_STATE_DISCONNECTED;
```

Allows for better state visibility and recovery logic.

#### Fix 4.2: Exponential Backoff

**Current**: Always wait 5s between reconnect attempts

**Problem**: If server is down, spams reconnect attempts

**Fix**:
```c
#define TELEMETRY_RECONNECT_MIN_MS 1000      // Start at 1s
#define TELEMETRY_RECONNECT_MAX_MS 60000     // Cap at 60s

static uint32_t reconnect_delay = TELEMETRY_RECONNECT_MIN_MS;

// On successful connect:
reconnect_delay = TELEMETRY_RECONNECT_MIN_MS;

// On connect failure:
sleep_ms(reconnect_delay);
reconnect_delay = min(reconnect_delay * 2, TELEMETRY_RECONNECT_MAX_MS);
```

#### Fix 4.3: Ring Buffer Overflow Handling

**Current**: Drops measurements if buffer full

**Enhancement**: Log when dropping starts/stops

```c
static bool last_write_success = true;

if (!ring_buffer_try_write(&measurement_buffer, &m)) {
    if (last_write_success) {
        TEL_LOG_ERROR("Ring buffer full! Dropping measurements\n");
        last_write_success = false;
    }
} else {
    if (!last_write_success) {
        TEL_LOG_INFO("Ring buffer recovered\n");
        last_write_success = true;
    }
}
```

## Implementation Priority

### Must Do (Fixes critical bugs):
1. ✅ Fix 1.1: Align Keepalive/Timeout (5 minutes)
2. ✅ Fix 1.2: Standardize Port (2 minutes)
3. ✅ Fix 1.3: TCP Write Retry (30 minutes)
4. ✅ Fix 1.4: Reduce Logging (15 minutes)
5. ✅ Fix 2.1: Server Graceful Handling (15 minutes)

**Total: ~1 hour**

### Should Do (Improves stability):
6. ⚠️ Fix 2.2: Connection State Visibility (20 minutes)
7. ⚠️ Fix 4.2: Exponential Backoff (15 minutes)
8. ⚠️ Fix 4.3: Ring Buffer Logging (10 minutes)

**Total: +45 minutes**

### Nice to Have (Future enhancements):
9. 💡 Fix 3.2: Compression (2 hours)
10. 💡 Fix 3.1: Batch ACKs (3 hours)
11. 💡 Fix 4.1: State Machine (1 hour)

## Testing Plan

### Test 1: Server Restart Resilience
1. Start Pico, start server, verify connection
2. Kill server (Ctrl+C)
3. **Expected**: Pico detects disconnect within 10-15s
4. Restart server
5. **Expected**: Pico reconnects within 5s
6. **Expected**: Measurements continue flowing

### Test 2: Network Interruption
1. Disconnect WiFi AP
2. **Expected**: Pico detects failure, attempts reconnect with backoff
3. Reconnect WiFi AP
4. **Expected**: Pico reconnects and resumes

### Test 3: Long-term Stability
1. Run for 8+ hours
2. **Expected**: Zero dropped measurements (ring buffer)
3. **Expected**: No memory leaks (monitor free RAM)
4. **Expected**: Consistent send success rate

### Test 4: High Load
1. Reduce batch size to 10 (send more frequently)
2. **Expected**: No send failures
3. **Expected**: No ring buffer overflows

## Configuration Recommendations

### Optimal Settings

**For 1Hz measurement rate**:
```c
#define TELEMETRY_BATCH_SIZE 30          // Send every 30s (was 60)
#define TELEMETRY_SERVER_PORT 5001       // Fixed
#define TELEMETRY_RECONNECT_DELAY_MS 2000  // 2s (was 5s)
#define TELEMETRY_KEEPALIVE_MS 15000     // 15s (was 60s)
#define MEASUREMENT_BUFFER_SIZE 1000     // Unchanged (~16 min buffer)
```

**Server**:
```go
conn.SetReadDeadline(time.Now().Add(45 * time.Second))  // Batch@30s + margin
```

**Rationale**:
- Batch every 30s = more frequent updates, faster server restart recovery
- Keepalive every 15s = server sees activity at least 2x per minute
- Server timeout at 45s = covers batch interval + 50% margin
- Reconnect in 2s = faster recovery from transient issues

### For Slower Networks

```c
#define TELEMETRY_BATCH_SIZE 120         // 2 minutes of data
#define TELEMETRY_KEEPALIVE_MS 30000     // 30s keepalive
```

Server timeout: 150s

## Files to Modify

### Pico W Changes:
1. `measurement/telemetry.h` - Update constants
2. `measurement/telemetry.c` - Add retry logic, logging levels
3. `measurement/main_measurement_device_wifi.c` - Minor logging changes

### Server Changes:
1. `tools/telemetry-server/main.go` - Change default port
2. `tools/telemetry-server/server.go` - Increase timeout, add TCP keepalive
3. `tools/telemetry-server/stats.go` - Add connection tracking
4. `tools/telemetry-server/README.md` - Update docs

## Summary

The main issues are:
1. **Timing conflict** between keepalive (60s) and timeout (30s)
2. **No retry logic** for transient TCP errors
3. **Port inconsistency** across code/docs
4. **Too much logging noise**

The Phase 1 fixes should resolve 90% of the stability problems with minimal effort. Phase 2+ are optional enhancements.

**Estimated time to implement Phase 1**: ~1 hour
**Expected improvement**: Should achieve hours of stable operation instead of minutes
