package main

import (
	"encoding/json"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true // Allow all origins for local development
	},
}

type WebSocketHandler struct {
	stats   *Statistics
	clients map[*websocket.Conn]bool
	mu      sync.RWMutex
}

func NewWebSocketHandler(stats *Statistics) *WebSocketHandler {
	h := &WebSocketHandler{
		stats:   stats,
		clients: make(map[*websocket.Conn]bool),
	}

	// Start broadcaster goroutine
	go h.broadcaster()

	return h
}

func (h *WebSocketHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("WebSocket upgrade error: %v", err)
		return
	}

	h.mu.Lock()
	h.clients[conn] = true
	h.mu.Unlock()

	log.Printf("WebSocket client connected (total: %d)", len(h.clients))

	// Send initial snapshot
	snapshot := h.stats.Snapshot()
	h.sendSnapshot(conn, snapshot)

	// Keep connection alive and handle disconnect
	defer func() {
		h.mu.Lock()
		delete(h.clients, conn)
		h.mu.Unlock()
		conn.Close()
		log.Printf("WebSocket client disconnected (total: %d)", len(h.clients))
	}()

	// Read messages (just to detect disconnect)
	for {
		_, _, err := conn.ReadMessage()
		if err != nil {
			break
		}
	}
}

func (h *WebSocketHandler) broadcaster() {
	updates := h.stats.Subscribe()

	for snapshot := range updates {
		h.mu.RLock()
		clients := make([]*websocket.Conn, 0, len(h.clients))
		for conn := range h.clients {
			clients = append(clients, conn)
		}
		h.mu.RUnlock()

		// Broadcast to all clients
		for _, conn := range clients {
			h.sendSnapshot(conn, snapshot)
		}
	}
}

func (h *WebSocketHandler) sendSnapshot(conn *websocket.Conn, snapshot *StatsSnapshot) {
	data, err := json.Marshal(snapshot)
	if err != nil {
		log.Printf("Error marshaling snapshot: %v", err)
		return
	}

	conn.SetWriteDeadline(time.Now().Add(5 * time.Second))
	if err := conn.WriteMessage(websocket.TextMessage, data); err != nil {
		log.Printf("Error sending to WebSocket client: %v", err)
	}
}
