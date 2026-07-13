// ======================================================================
// ESP32 WROOM (38-pin) — RPLIDAR A1 UART ↔ WiFi TCP client bridge.
// Host PC is the TCP server (same pattern as micro-ROS AGENT_IP).
//
// Wiring (external 5V OK; share GND; do NOT plug A1 USB into the PC):
//   A1 TX  -> ESP32 GPIO16 (UART2 RX)   [if timeout, try SWAP_UART_PINS]
//   A1 RX  -> ESP32 GPIO17 (UART2 TX)
//   A1 GND -> ESP32 GND
// ======================================================================
#pragma once

static const int PIN_LIDAR_RX = 16;  // ESP32 receives from A1 TX
static const int PIN_LIDAR_TX = 17;  // ESP32 transmits to A1 RX

// If sllidar times out with tcp->uart>0 and uart->tcp==0, flip this and reflash.
#ifndef SWAP_UART_PINS
#define SWAP_UART_PINS 1
#endif

static const uint32_t LIDAR_BAUD = 115200;  // RPLIDAR A1
static const uint32_t USB_BAUD   = 115200;  // status prints

#define WIFI_SSID "NETGEAR71"
#define WIFI_PASS "melodicdaisy353"

// Host PC running rplidar_tcp_relay (same idea as micro-ROS AGENT_IP).
static const char HOST_IP[] = "192.168.1.12";
static const uint16_t HOST_PORT = 20108;

static const uint32_t RECONNECT_MS = 2000;

static const size_t UART_RX_BUF = 8192;
static const size_t BRIDGE_CHUNK = 512;
