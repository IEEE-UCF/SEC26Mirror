// Fast I2C Scanner — scans all Wire buses on Teensy 4.1 or single Wire on ESP32
// Prints results as a compact table, rescans every 2 seconds.

#include <Arduino.h>
#include <Wire.h>

#ifdef ESP32
#define NUM_BUSES 1
static TwoWire* buses[] = {&Wire};
static const char* names[] = {"Wire"};
#else
#define NUM_BUSES 3
static TwoWire* buses[] = {&Wire, &Wire1, &Wire2};
static const char* names[] = {"Wire0", "Wire1", "Wire2"};
#endif

static void scanBus(TwoWire* bus, const char* name) {
  int found = 0;
  Serial.printf("  %s: ", name);
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus->beginTransmission(addr);
    if (bus->endTransmission() == 0) {
      if (found++) Serial.print(", ");
      Serial.printf("0x%02X", addr);
    }
  }
  if (!found) Serial.print("none");
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  // Wait for serial with 2s timeout
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  for (int i = 0; i < NUM_BUSES; i++) {
    buses[i]->begin();
    buses[i]->setClock(400000);
  }

  Serial.println("\n=== I2C Scanner ===");
}

void loop() {
  Serial.printf("\n[%lums] Scanning...\n", millis());
  for (int i = 0; i < NUM_BUSES; i++) {
    scanBus(buses[i], names[i]);
  }
  delay(2000);
}
