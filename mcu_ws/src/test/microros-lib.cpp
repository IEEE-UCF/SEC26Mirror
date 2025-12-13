// Minimal test that starts the microros subsystem and runs it
#include <Arduino.h>
#include <microros_manager_example.h>

using namespace Subsystem;

// Configure the microros subsystem
static MicrorosManagerSetup microrosSetup("microros_test");
static MicrorosManager microros(microrosSetup);

void setup() {
	// Initialize and start microros subsystem
	microros.init();
	microros.begin();
}

void loop() {
	// Run microros state machine
	microros.update();
	// Yield a bit; adjust if needed per board
	delay(1);
}

