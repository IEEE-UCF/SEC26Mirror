// Test harness for MicrorosManager: publishes TF (pose), heartbeat, and state
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

	// Seed initial state and pose
	mgr->setState("BOOT");
	mgr->setPose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void loop() {
	static uint32_t t = 0;
	static float theta = 0.0f;
	t += 1;
	theta += 1.0f; // degrees

	// Fake circular position and oscillating altitude
	float x = 0.5f * cos(theta * 3.14159f / 180.0f);
	float y = 0.5f * sin(theta * 3.14159f / 180.0f);
	float z = 0.1f * sin(theta * 2.0f * 3.14159f / 180.0f);

	// Fake orientation (roll/pitch/yaw in degrees)
	float roll = 5.0f * sin(theta * 3.14159f / 180.0f);
	float pitch = 3.0f * cos(theta * 3.14159f / 180.0f);
	float yaw = fmod(theta, 360.0f);

	// Update manager cached pose
	mgr->setPose(x, y, z, roll, pitch, yaw);

	// Cycle state string every few seconds
	if (t % 500 == 0) mgr->setState("IDLE");
	else if (t % 500 == 250) mgr->setState("RUNNING");

	// Drive reconnection + publishing
	mgr->update();

	delay(10);
}
