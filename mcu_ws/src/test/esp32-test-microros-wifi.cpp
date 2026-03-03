/**
 * @file esp32-test-microros-wifi.cpp
 * @brief ESP32 micro-ROS WiFi transport test
 *
 * Minimal test that connects to WiFi and a micro-ROS agent over UDP,
 * then publishes a heartbeat string message every second.
 *
 * Before building, update WIFI_SSID, WIFI_PASSWORD, AGENT_IP, and LOCAL_IP
 * in platformio.ini under [esp32_microros_wifi].
 *
 * Run the agent on your network:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *
 * Verify with:
 *   ros2 topic echo /esp32_wifi_test/status std_msgs/msg/String
 */

#include <Arduino.h>
#include <ESP32WifiSubsystem.h>
#include <WiFi.h>
#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/string.h>

#include "microros_manager_robot.h"

using namespace Subsystem;

// Inline heartbeat publisher — replaces ExampleMicrorosSubsystem dependency.
static rcl_publisher_t g_pub;
static std_msgs__msg__String g_msg;

class SimpleHeartbeat : public IMicroRosParticipant {
 public:
  bool onCreate(rcl_node_t* node, rclc_executor_t*) override {
    return rclc_publisher_init_best_effort(
               &g_pub, node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
               "esp32_wifi_test/status") == RCL_RET_OK;
  }
  void onDestroy() override { g_pub = rcl_get_zero_initialized_publisher(); }
  void publish(const char* text) {
    if (!g_pub.impl) return;
    g_msg.data = micro_ros_string_utilities_set(g_msg.data, text);
    rcl_publish(&g_pub, &g_msg, NULL);
  }
};

// --- WiFi subsystem (manages connection lifecycle + auto-reconnect) ---
static IPAddress g_local_ip(LOCAL_IP);
static ESP32WifiSubsystemSetup g_wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, g_local_ip,
    IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0),
    /*connection_timeout_ms=*/10000,
    /*reconnect_interval_ms=*/5000,
    /*max_retries=*/0);
static ESP32WifiSubsystem g_wifi(g_wifi_setup);

MicrorosManagerSetup managerSetup("esp32_wifi_test");
MicrorosManager manager(managerSetup);
static SimpleHeartbeat heartbeat;

void setup() {
  Serial.begin(921600);
  delay(2000);  // Allow serial monitor to connect

  Serial.println("=== ESP32 micro-ROS WiFi Transport Test ===");
  {
    uint8_t agent_ip[] = AGENT_IP;
    Serial.printf("Agent: %d.%d.%d.%d:%d\n", agent_ip[0], agent_ip[1],
                  agent_ip[2], agent_ip[3], AGENT_PORT);
  }
  Serial.printf("SSID:  %s\n", WIFI_SSID);

  // Connect WiFi first (managed by ESP32WifiSubsystem)
  g_wifi.init();
  g_wifi.begin();
  Serial.println("Waiting for WiFi...");
  while (!g_wifi.isConnected()) {
    g_wifi.update();
    delay(10);
  }
  Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());

  manager.init();
  manager.registerParticipant(&heartbeat);
  manager.begin();

  Serial.println("Setup complete. Waiting for agent...");
}

void loop() {
  g_wifi.update();
  manager.update();

  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms > 1000) {
    if (manager.isConnected()) {
      Serial.println("[OK] Agent connected - publishing heartbeat");
      heartbeat.publish("OK");
    } else {
      Serial.println("[..] Waiting for agent connection...");
    }
    last_ms = now;
  }
}
