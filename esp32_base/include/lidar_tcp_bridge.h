// RPLIDAR A1 UART ↔ WiFi TCP client (host is server on LIDAR_HOST_PORT).
// Passthrough only — do NOT drop UART based on availableForWrite() (often 0 on
// ESP32 WiFiClient and was wiping getDeviceInfo replies).
#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "config_l298n.h"

#if LIDAR_BRIDGE_ENABLE

static WiFiClient g_lidar_client;
static HardwareSerial LidarSerial(2);
static uint8_t g_lidar_buf[LIDAR_BRIDGE_CHUNK];
static uint32_t g_lidar_last_connect_ms = 0;
static bool g_lidar_link_up = false;
static uint32_t g_lidar_link_start_ms = 0;

static void lidar_ensure_wifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(MICROROS_WIFI_SSID, MICROROS_WIFI_PASS);
  Serial.print("Lidar bridge WiFi connecting");
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Lidar bridge WiFi OK IP=%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("Lidar bridge WiFi FAILED");
  }
}

static void lidar_drain_rx() {
  while (LidarSerial.available() > 0) {
    (void)LidarSerial.read();
  }
}

// Only after the SDK handshake: if UART backlog is huge, keep the newest bytes.
static void lidar_trim_uart_backlog() {
  // Protect getDeviceInfo / startScan for the first few seconds.
  if ((millis() - g_lidar_link_start_ms) < 5000) {
    return;
  }
  int avail = LidarSerial.available();
  if (avail <= LIDAR_UART_DROP_ABOVE) {
    return;
  }
  int drop = avail - LIDAR_UART_KEEP_AFTER_DROP;
  while (drop-- > 0 && LidarSerial.available() > 0) {
    (void)LidarSerial.read();
  }
}

static void lidar_reset_link() {
  static const uint8_t kStop[] = {0xA5, 0x25};
  LidarSerial.write(kStop, sizeof(kStop));
  LidarSerial.flush();
  delay(20);
  const uint32_t t0 = millis();
  while (millis() - t0 < 80) {
    lidar_drain_rx();
    delay(1);
  }
  lidar_drain_rx();
}

static size_t lidar_pump_uart_to_tcp() {
  lidar_trim_uart_backlog();

  size_t total = 0;
  while (LidarSerial.available() > 0) {
    int n = 0;
    while (n < (int)LIDAR_BRIDGE_CHUNK && LidarSerial.available() > 0) {
      const int c = LidarSerial.read();
      if (c < 0) {
        break;
      }
      g_lidar_buf[n++] = (uint8_t)c;
    }
    if (n <= 0) {
      break;
    }
    size_t off = 0;
    while (off < (size_t)n) {
      const size_t w = g_lidar_client.write(g_lidar_buf + off, (size_t)n - off);
      if (w == 0) {
        return total;
      }
      off += w;
      total += w;
    }
  }
  return total;
}

static size_t lidar_pump_tcp_to_uart() {
  size_t total = 0;
  while (g_lidar_client.available() > 0) {
    int n = 0;
    while (n < (int)LIDAR_BRIDGE_CHUNK && g_lidar_client.available() > 0) {
      const int c = g_lidar_client.read();
      if (c < 0) {
        break;
      }
      g_lidar_buf[n++] = (uint8_t)c;
    }
    if (n <= 0) {
      break;
    }
    LidarSerial.write(g_lidar_buf, (size_t)n);
    total += (size_t)n;
  }
  if (total > 0) {
    LidarSerial.flush();
  }
  return total;
}

static bool lidar_ensure_host() {
  if (g_lidar_client.connected()) {
    return true;
  }

  if (g_lidar_link_up) {
    g_lidar_link_up = false;
    Serial.println("Lidar TCP disconnected");
  }
  if (g_lidar_client) {
    g_lidar_client.stop();
  }

  const uint32_t now = millis();
  if (now - g_lidar_last_connect_ms < LIDAR_RECONNECT_MS) {
    return false;
  }
  g_lidar_last_connect_ms = now;

  Serial.printf("Lidar TCP connect %s:%u ... ", AGENT_IP, (unsigned)LIDAR_HOST_PORT);
  if (!g_lidar_client.connect(AGENT_IP, LIDAR_HOST_PORT)) {
    Serial.println("fail");
    return false;
  }

  g_lidar_client.setNoDelay(true);
  lidar_reset_link();
  g_lidar_link_up = true;
  g_lidar_link_start_ms = millis();
  Serial.println("OK");
  return true;
}

static void lidarBridgeTask(void *arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(1500));
  lidar_ensure_wifi();

#if LIDAR_SWAP_UART_PINS
  const int rx = PIN_LIDAR_TX;
  const int tx = PIN_LIDAR_RX;
  Serial.println("Lidar UART SWAP=1  (A1 TX->GPIO19, A1 RX->GPIO18)");
#else
  const int rx = PIN_LIDAR_RX;
  const int tx = PIN_LIDAR_TX;
  Serial.println("Lidar UART SWAP=0  (A1 TX->GPIO18, A1 RX->GPIO19)");
#endif
  Serial.printf("Lidar UART2 RX=GPIO%d TX=GPIO%d baud=%lu\n", rx, tx,
                (unsigned long)LIDAR_UART_BAUD);

  LidarSerial.setRxBufferSize(LIDAR_UART_RX_BUF);
  LidarSerial.setTimeout(1);
  LidarSerial.begin(LIDAR_UART_BAUD, SERIAL_8N1, rx, tx);

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (g_lidar_client) {
        g_lidar_client.stop();
      }
      g_lidar_link_up = false;
      lidar_ensure_wifi();
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (!lidar_ensure_host()) {
      lidar_drain_rx();
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    bool busy = false;
    for (int i = 0; i < 16; ++i) {
      const size_t a = lidar_pump_uart_to_tcp();
      const size_t b = lidar_pump_tcp_to_uart();
      if (a == 0 && b == 0) {
        break;
      }
      busy = true;
    }
    // Always yield so micro-ROS (core0) can keep /odom /cmd_vel on WiFi.
    vTaskDelay(pdMS_TO_TICKS(busy ? 2 : 5));
  }
}

static void start_lidar_tcp_bridge() {
  // Priority below motor loop; never starve micro-ROS WiFi.
  xTaskCreatePinnedToCore(lidarBridgeTask, "lidar_tcp", 6144, NULL, 1, NULL, 1);
  Serial.printf("Lidar bridge task started (core1) → %s:%u\n", AGENT_IP,
                (unsigned)LIDAR_HOST_PORT);
}

#else  // !LIDAR_BRIDGE_ENABLE

static void start_lidar_tcp_bridge() {}

#endif
