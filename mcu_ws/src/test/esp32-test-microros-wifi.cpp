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
#include <WiFi.h>
#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/string.h>

#include "microros_manager_robot.h"

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

Subsystem::MicrorosManagerSetup managerSetup("esp32_wifi_test");
Subsystem::MicrorosManager manager(managerSetup);
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

  manager.init();
  manager.begin();
  manager.registerParticipant(&heartbeat);

  Serial.printf("WiFi status: %d (3=connected)\n", WiFi.status());
  Serial.printf("Local IP:  %s\n", WiFi.localIP().toString().c_str());
  Serial.println("Setup complete. Waiting for agent...");
}

void loop() {
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
