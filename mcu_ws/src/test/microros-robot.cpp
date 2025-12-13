// Test harness for MicrorosManager: publishes heartbeat and state
#include <Arduino.h>
#include "robot/microros/microros_manager_robot.h"

using namespace Subsystem;

static MicrorosManager* mgr;

void setup() {
	// Create manager
	static MicrorosManagerSetup setup("robot_manager_test");
	static MicrorosManager manager(setup);
	mgr = &manager;

	// Initialize and begin (sets transports)
	mgr->init();
	mgr->begin();

	// Seed initial state
	mgr->setState("BOOT");
}

void loop() {
	static uint32_t t = 0;
	static float theta = 0.0f;
	t += 1;
	theta += 1.0f; // degrees

	// (TF pose updates removed)

	// Cycle state string every few seconds
	if (t % 500 == 0) mgr->setState("IDLE");
	else if (t % 500 == 250) mgr->setState("RUNNING");

	// Drive reconnection + publishing
	mgr->update();

	delay(10);
}
