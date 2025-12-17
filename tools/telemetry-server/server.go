package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net"
	"os"
	"time"
)

type TCPServer struct {
	port  int
	stats *Statistics
	log   *os.File
}

func NewTCPServer(port int, stats *Statistics, logFile string) *TCPServer {
	server := &TCPServer{
		port:  port,
		stats: stats,
	}

	// Open log file if specified
	if logFile != "" {
		f, err := os.OpenFile(logFile, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
		if err != nil {
			log.Printf("Warning: failed to open log file: %v", err)
		} else {
			server.log = f
		}
	}

	return server
}

func (s *TCPServer) Start() {
	listener, err := net.Listen("tcp", fmt.Sprintf(":%d", s.port))
	if err != nil {
		log.Fatalf("Failed to start TCP server: %v", err)
	}
	defer listener.Close()

	for {
		conn, err := listener.Accept()
		if err != nil {
			log.Printf("Failed to accept connection: %v", err)
			continue
		}

		log.Printf("New connection from %s", conn.RemoteAddr())
		go s.handleConnection(conn)
	}
}

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
		// Set read deadline to 90s (allows for batch interval + margin)
		// Pico sends batches every ~60s (60 measurements @ 1Hz)
		// 90s timeout gives 50% margin for network delays
		conn.SetReadDeadline(time.Now().Add(90 * time.Second))

		line, err := reader.ReadBytes('\n')
		if err != nil {
			if err != io.EOF {
				log.Printf("Error reading from %s: %v", conn.RemoteAddr(), err)
			}
			return
		}

		// Parse batch
		var batch Batch
		if err := json.Unmarshal(line, &batch); err != nil {
			log.Printf("Error parsing JSON from %s: %v", conn.RemoteAddr(), err)
			continue
		}

		// Update statistics
		s.stats.Update(&batch)

		// Log to file if enabled
		if s.log != nil {
			if _, err := s.log.Write(line); err != nil {
				log.Printf("Warning: failed to write to log file: %v", err)
			}
		}

		// Print batch info
		log.Printf("Received batch #%d: %d measurements (total samples: %d)",
			batch.BatchSeq, batch.Count, s.stats.Snapshot().SampleCount)
	}
}
