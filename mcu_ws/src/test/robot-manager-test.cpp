#include <Arduino.h>

#include <vector>

#include "robot/machines/RobotManager.h"
#include <microros_manager_robot.h>
#include <ExampleMicrorosSubsystem.h>

using namespace Subsystem;

// --- micro-ROS manager and example subsystem ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

static ExampleSubsystemSetup g_ex_setup("example_subsystem");
static ExampleSubsystem g_example(g_ex_setup);

// --- RobotManager wiring ---
// Create RobotObject entries matching timekeepers where they should run
static RobotObject g_obj_mr(g_mr, MS_20); // run at 50 Hz

static std::vector<RobotObject*> g_objects{&g_obj_mr};
static RobotManagerSetup g_rm_setup("robot_manager", g_objects);
static RobotManager g_rm(g_rm_setup);

void setup() {
  // Attach the example ROS participant to the micro-ROS manager
  g_mr.registerParticipant(&g_example);

  // Initialize all robot objects via RobotManager
  (void)g_rm.init();

  // Move micro-ROS manager to active state (transport setup etc.)
  g_mr.begin();
}

void loop() {
  // Let RobotManager drive timed updates for all objects
  g_rm.update();
}
