#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#include <WiFi.h>
#include <WiFiUdp.h>

extern "C" {
bool platformio_transport_open(struct uxrCustomTransport *transport);
bool platformio_transport_close(struct uxrCustomTransport *transport);
size_t platformio_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf,
                                  size_t len, uint8_t *errcode);
size_t platformio_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len,
                                 int timeout, uint8_t *errcode);
}

static struct micro_ros_agent_locator g_microros_agent_locator;

static void setup_microros_wifi_udp(char *wifi_ssid, char *wifi_pass, IPAddress agent_ip,
                                    uint16_t agent_port) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);

  Serial.print("WiFi connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi FAILED — check SSID/password; micro-ROS will retry when agent runs");
  } else {
    Serial.printf("WiFi OK  IP=%s\n", WiFi.localIP().toString().c_str());
  }

  g_microros_agent_locator.address = agent_ip;
  g_microros_agent_locator.port = agent_port;
  Serial.printf("micro-ROS agent target %s:%u\n", agent_ip.toString().c_str(), agent_port);

  rmw_uros_set_custom_transport(
      false, (void *)&g_microros_agent_locator, platformio_transport_open,
      platformio_transport_close, platformio_transport_write, platformio_transport_read);
}
#endif

// Serial: micro-ROS owns USB Serial — no Serial.print after this call.
// WiFi:   Serial remains free for debug prints.
inline void init_microros_transport(char *wifi_ssid, char *wifi_pass, const char *agent_ip,
                                    uint16_t agent_port) {
  Serial.begin(115200);
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  IPAddress agent_addr;
  agent_addr.fromString(agent_ip);
  setup_microros_wifi_udp(wifi_ssid, wifi_pass, agent_addr, agent_port);
  delay(2000);
#else
  (void)wifi_ssid;
  (void)wifi_pass;
  (void)agent_ip;
  (void)agent_port;
  set_microros_serial_transports(Serial);
#endif
}
