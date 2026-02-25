/**
 * @file teensy-test-debug.cpp
 * @brief Minimal micro-ROS debug test — single-threaded, no TeensyThreads.
 *
 * Everything runs from loop(). This eliminates threading, mutexes, and
 * transport contention as variables.  If this doesn't connect, the problem
 * is the transport/agent/library — not our application code.
 *
 * Built-in LED:
 *   - Slow blink (1 Hz)  = WAITING_AGENT
 *   - Fast blink (4 Hz)  = AGENT_CONNECTED
 *   - Solid ON            = create_entities failed
 *
 * micro-ROS agent (on host):
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
 *
 * Verify:
 *   ros2 topic echo /mcu_robot/heartbeat
 */

#include <Arduino.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/string.h>
#include <micro_ros_utilities/string_utilities.h>

using namespace Subsystem;

// ── Globals ──────────────────────────────────────────────────────────────

static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// Inline heartbeat — no separate subsystem class needed.
static rcl_publisher_t g_pub;
static std_msgs__msg__String g_msg;

// Simple participant that creates one publisher.
class SimpleHeartbeat : public IMicroRosParticipant {
 public:
  bool onCreate(rcl_node_t* node, rclc_executor_t* /*executor*/) override {
    return rclc_publisher_init_best_effort(
               &g_pub, node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
               "/mcu_robot/heartbeat") == RCL_RET_OK;
  }
  void onDestroy() override {
    g_pub = rcl_get_zero_initialized_publisher();
  }
};

static SimpleHeartbeat g_hb;

// ── LED helpers ─────────────────────────────────────────────────────────

static bool g_led_state = false;
static uint32_t g_led_last = 0;

static void updateLED() {
  uint32_t interval;
  if (g_mr.isConnected()) {
    interval = 125;  // fast blink
  } else {
    interval = 500;  // slow blink
  }
  if (millis() - g_led_last >= interval) {
    g_led_state = !g_led_state;
    digitalWrite(LED_BUILTIN, g_led_state ? HIGH : LOW);
    g_led_last = millis();
  }
}

// ── Arduino entry points ────────────────────────────────────────────────

void setup() {
  Serial.begin(0);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }
  pinMode(LED_BUILTIN, OUTPUT);

  g_mr.init();
  g_mr.registerParticipant(&g_hb);
  g_msg.data = micro_ros_string_utilities_set(g_msg.data, "HEARTBEAT");

  // begin() sets up the transport — no more Serial output after this.
  g_mr.begin();
}

void loop() {
  g_mr.update();

  // Publish heartbeat every 200ms when connected
  static uint32_t last_pub = 0;
  if (g_pub.impl && (millis() - last_pub >= 200)) {
    rcl_publish(&g_pub, &g_msg, NULL);
    last_pub = millis();
  }

  updateLED();
  delay(10);
}
