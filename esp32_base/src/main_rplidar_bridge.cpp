// RPLIDAR A1 UART ↔ WiFi TCP client bridge.
// ESP32 connects to the PC relay (HOST_IP:HOST_PORT); you never need the ESP32 IP.

#include <Arduino.h>
#include <WiFi.h>

#include "config_rplidar_bridge.h"

static WiFiClient g_client;
static HardwareSerial LidarSerial(2);

static uint8_t g_buf[BRIDGE_CHUNK];

static uint32_t g_tcp_to_uart = 0;
static uint32_t g_uart_to_tcp = 0;
static uint32_t g_last_stats_ms = 0;
static uint32_t g_last_connect_attempt_ms = 0;
static bool g_logged_first_rx = false;
static bool g_logged_first_tx = false;

static void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting to ");
  Serial.println(WIFI_SSID);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
    if (millis() - start > 30000) {
      Serial.println("\nWiFi timeout, restarting");
      ESP.restart();
    }
  }

  Serial.println();
  Serial.print("WiFi OK  IP=");
  Serial.println(WiFi.localIP());
  Serial.print("Host target ");
  Serial.print(HOST_IP);
  Serial.print(':');
  Serial.println(HOST_PORT);
}

static void drain_lidar_rx() {
  while (LidarSerial.available() > 0) {
    (void)LidarSerial.read();
  }
}

static void dump_prefix(const char *tag, const uint8_t *data, size_t n) {
  Serial.print(tag);
  const size_t show = n < 16 ? n : 16;
  for (size_t i = 0; i < show; ++i) {
    if (data[i] < 16) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

static size_t pump_uart_to_tcp() {
  size_t total = 0;
  while (LidarSerial.available() > 0) {
    int n = 0;
    while (n < (int)BRIDGE_CHUNK && LidarSerial.available() > 0) {
      const int c = LidarSerial.read();
      if (c < 0) {
        break;
      }
      g_buf[n++] = (uint8_t)c;
    }
    if (n <= 0) {
      break;
    }
    if (!g_logged_first_rx) {
      g_logged_first_rx = true;
      dump_prefix("UART RX: ", g_buf, (size_t)n);
    }
    size_t off = 0;
    while (off < (size_t)n) {
      const size_t w = g_client.write(g_buf + off, (size_t)n - off);
      if (w == 0) {
        return total;
      }
      off += w;
      total += w;
    }
  }
  g_uart_to_tcp += (uint32_t)total;
  return total;
}

static size_t pump_tcp_to_uart() {
  size_t total = 0;
  while (g_client.available() > 0) {
    int n = 0;
    while (n < (int)BRIDGE_CHUNK && g_client.available() > 0) {
      const int c = g_client.read();
      if (c < 0) {
        break;
      }
      g_buf[n++] = (uint8_t)c;
    }
    if (n <= 0) {
      break;
    }
    if (!g_logged_first_tx) {
      g_logged_first_tx = true;
      dump_prefix("TCP->UART: ", g_buf, (size_t)n);
    }
    LidarSerial.write(g_buf, (size_t)n);
    total += (size_t)n;
  }
  if (total > 0) {
    LidarSerial.flush();
  }
  g_tcp_to_uart += (uint32_t)total;
  return total;
}

static void print_stats() {
  const uint32_t now = millis();
  if (now - g_last_stats_ms < 1000) {
    return;
  }
  g_last_stats_ms = now;
  Serial.print("stats tcp->uart=");
  Serial.print(g_tcp_to_uart);
  Serial.print(" uart->tcp=");
  Serial.print(g_uart_to_tcp);
  Serial.print(" host=");
  Serial.println(g_client.connected() ? "yes" : "no");
  if (g_tcp_to_uart > 0 && g_uart_to_tcp == 0) {
    Serial.println("HINT: commands out, no lidar reply — swap TX/RX or check 5V/GND/USB conflict");
  }
  g_tcp_to_uart = 0;
  g_uart_to_tcp = 0;
}

static bool ensure_host_connected() {
  if (g_client.connected()) {
    return true;
  }

  if (g_client) {
    g_client.stop();
  }

  const uint32_t now = millis();
  if (now - g_last_connect_attempt_ms < RECONNECT_MS) {
    return false;
  }
  g_last_connect_attempt_ms = now;

  Serial.print("Connecting to host ");
  Serial.print(HOST_IP);
  Serial.print(':');
  Serial.print(HOST_PORT);
  Serial.print(" ... ");

  if (g_client.connect(HOST_IP, HOST_PORT)) {
    g_client.setNoDelay(true);
    g_tcp_to_uart = 0;
    g_uart_to_tcp = 0;
    g_logged_first_rx = false;
    g_logged_first_tx = false;
    Serial.println("OK");
    return true;
  }

  Serial.println("fail (is run_rplidar_wifi.sh running?)");
  return false;
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(200);
  Serial.println();
  Serial.println("RPLIDAR A1 UART-TCP client bridge");

#if SWAP_UART_PINS
  const int rx = PIN_LIDAR_TX;
  const int tx = PIN_LIDAR_RX;
  Serial.println("SWAP_UART_PINS=1 (GPIO17=RX, GPIO16=TX)");
#else
  const int rx = PIN_LIDAR_RX;
  const int tx = PIN_LIDAR_TX;
  Serial.println("SWAP_UART_PINS=0 (GPIO16=RX, GPIO17=TX)");
#endif

  Serial.print("UART2 RX=GPIO");
  Serial.print(rx);
  Serial.print(" TX=GPIO");
  Serial.print(tx);
  Serial.print(" baud=");
  Serial.println(LIDAR_BAUD);

  LidarSerial.setRxBufferSize(UART_RX_BUF);
  LidarSerial.setTimeout(1);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, rx, tx);

  connect_wifi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting");
    if (g_client) {
      g_client.stop();
    }
    connect_wifi();
  }

  if (!ensure_host_connected()) {
    drain_lidar_rx();
    print_stats();
    delay(20);
    return;
  }

  pump_uart_to_tcp();
  pump_tcp_to_uart();
  print_stats();
}
