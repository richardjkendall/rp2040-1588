package main

import (
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"
)

var (
	tcpPort  = flag.Int("tcp", 5001, "TCP port for Pico connections")
	httpPort = flag.Int("http", 8080, "HTTP port for web UI")
	logFile  = flag.String("log", "", "Optional JSONL log file (empty = no logging)")
)

func main() {
	flag.Parse()

	fmt.Printf("PTP Phase Offset Telemetry Server\n")
	fmt.Printf("==================================\n\n")

	// Initialize statistics tracker
	stats := NewStatistics()

	// Start TCP server for Pico connections
	tcpServer := NewTCPServer(*tcpPort, stats, *logFile)
	go tcpServer.Start()

	// Start HTTP/WebSocket server for web UI
	webServer := NewWebServer(*httpPort, stats)
	go webServer.Start()

	fmt.Printf("TCP server listening on port %d (Pico connections)\n", *tcpPort)
	fmt.Printf("Web UI available at http://localhost:%d\n", *httpPort)
	if *logFile != "" {
		fmt.Printf("Logging measurements to %s\n", *logFile)
	}
	fmt.Printf("\nPress Ctrl+C to stop\n\n")

	// Wait for interrupt signal
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)
	<-sigChan

	fmt.Printf("\nShutting down...\n")
}
