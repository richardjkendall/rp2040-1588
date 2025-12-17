package main

import (
	"embed"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
)

//go:embed static
var staticFiles embed.FS

type WebServer struct {
	port  int
	stats *Statistics
}

func NewWebServer(port int, stats *Statistics) *WebServer {
	return &WebServer{
		port:  port,
		stats: stats,
	}
}

func (s *WebServer) Start() {
	// WebSocket endpoint
	wsHandler := NewWebSocketHandler(s.stats)
	http.Handle("/ws", wsHandler)

	// REST API endpoints
	http.HandleFunc("/api/stats", s.handleStats)
	http.HandleFunc("/api/reset", s.handleReset)
	http.HandleFunc("/api/download", s.handleDownload)

	// Serve static files (web UI)
	http.Handle("/", http.FileServer(http.FS(staticFiles)))

	addr := fmt.Sprintf(":%d", s.port)
	log.Fatal(http.ListenAndServe(addr, nil))
}

func (s *WebServer) handleStats(w http.ResponseWriter, r *http.Request) {
	snapshot := s.stats.Snapshot()

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(snapshot); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
	}
}

func (s *WebServer) handleReset(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	s.stats.Reset()
	log.Println("Statistics reset")

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "reset"})
}

func (s *WebServer) handleDownload(w http.ResponseWriter, r *http.Request) {
	measurements := s.stats.GetRawMeasurements()

	w.Header().Set("Content-Type", "text/csv")
	w.Header().Set("Content-Disposition", "attachment; filename=phase_measurements.csv")

	// Write CSV header
	fmt.Fprintf(w, "seq,timestamp_us,phase_ns,gm_to_slave_ns,slave_to_gm_ns,scale_factor,gm_first,crystal_error_ns,received_at\n")

	// Write measurements
	for _, m := range measurements {
		fmt.Fprintf(w, "%d,%d,%.1f,%.1f,%.1f,%.9f,%t,%d,%s\n",
			m.Seq,
			m.TimestampUs,
			m.PhaseNs,
			m.GmToSlaveNs,
			m.SlaveToGmNs,
			m.ScaleFactor,
			m.GmFirst,
			m.CrystalErrorNs,
			m.ReceivedAt.Format("2006-01-02T15:04:05.000Z"))
	}
}
