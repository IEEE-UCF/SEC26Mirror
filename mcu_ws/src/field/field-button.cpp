#include <field-colors.h>
#include <field-element.h>
#include <raw-rgb-analog-led.h>

#include "button.h"

// Pin configs
const uint8_t ledPin = 32;
const uint8_t swPin = 5;

const std::array<uint8_t, 3> ryg = {26, 25, 33};

const uint8_t r = 15;
const uint8_t g = 2;
const uint8_t b = 4;

// RGB setup and class
RawDrivers::RGBAnalogLEDSetup setupRGB =
    RawDrivers::RGBAnalogLEDSetup("Antenna1", {r, g, b});

RawDrivers::RGBAnalogLED RGB1 = RawDrivers::RGBAnalogLED(setupRGB);

// Button configuration
Field::ButtonSetup setupButton =
    Field::ButtonSetup("Button", ryg, ledPin, swPin);
Field::ButtonDriver Button1 = Field::ButtonDriver(setupButton);

// Field element for ESP-NOW communication
Field::FieldElement fieldElement(Field::ElementId::BUTTON);

// Color options
RawDrivers::RGBColor* colorOptions[] = {&Field::colorRed, &Field::colorGreen,
                                        &Field::colorBlue, &Field::colorPurple};
Field::FieldColor fieldColorOptions[] = {
    Field::FieldColor::RED, Field::FieldColor::GREEN, Field::FieldColor::BLUE,
    Field::FieldColor::PURPLE};

int randomNum;
bool taskCompleted = false;
bool cycleDisplayActive = false;
uint8_t cycleHue = 0;
unsigned long lastCycleUpdate = 0;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 1000;

// HSV to RGB conversion for smooth cycling
void hsvToRgb(uint8_t h, uint8_t s, uint8_t v, uint8_t& rOut, uint8_t& gOut,
              uint8_t& bOut) {
  uint8_t region = h / 43;
  uint8_t remainder = (h - (region * 43)) * 6;
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0:
      rOut = v;
      gOut = t;
      bOut = p;
      break;
    case 1:
      rOut = q;
      gOut = v;
      bOut = p;
      break;
    case 2:
      rOut = p;
      gOut = v;
      bOut = t;
      break;
    case 3:
      rOut = p;
      gOut = q;
      bOut = v;
      break;
    case 4:
      rOut = t;
      gOut = p;
      bOut = v;
      break;
    default:
      rOut = v;
      gOut = p;
      bOut = q;
      break;
  }
}

void onCommand(Field::FieldCommand cmd) {
  if (cmd == Field::FieldCommand::START) {
    Button1.reset();
    taskCompleted = false;
    cycleDisplayActive = false;
    RGB1.setColor(Field::colorOff);
    RGB1.update();
  } else if (cmd == Field::FieldCommand::CYCLE_DISPLAY) {
    cycleDisplayActive = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Field-Button starting...");

  Button1.init();
  RGB1.init();

  // Initialize ESP-NOW
  fieldElement.initComms();
  fieldElement.setCommandCallback(onCommand);

  // Seed random with ESP32 hardware RNG for true randomness on each reset
  randomSeed(esp_random());

  // Generate random color
  randomNum = random(0, 4);
  Serial.print("Random color: ");
  Serial.println(Field::colorToString(fieldColorOptions[randomNum]));

  // Send initial color to earth
  fieldElement.sendColorReport(fieldColorOptions[randomNum]);

  Serial.println("Ready - press button 3 times");
}

void loop() {
  unsigned long now = millis();

  // Process incoming ESP-NOW messages
  fieldElement.processMessages();

  // Handle cycle display animation
  if (cycleDisplayActive || fieldElement.isCycleDisplayActive()) {
    cycleDisplayActive = true;
    if (now - lastCycleUpdate > 20) {
      lastCycleUpdate = now;
      uint8_t rOut, gOut, bOut;
      hsvToRgb(cycleHue++, 255, 150, rOut, gOut, bOut);
      RawDrivers::RGBColor cycleColor(rOut, gOut, bOut);
      RGB1.setColor(cycleColor);
      RGB1.update();
    }
  } else {
    // Normal operation
    Button1.update();

    if (Button1.getStatus() && !taskCompleted) {
      Serial.println("TASK COMPLETE!");
      taskCompleted = true;

      fieldElement.setStatus(Field::ElementStatus::ACTIVATED);

      RGB1.setColor(*colorOptions[randomNum]);
      RGB1.update();
    }
  }

  // Send periodic status updates
  if (now - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
    lastStatusUpdate = now;
    if (!taskCompleted &&
        fieldElement.getStatus() != Field::ElementStatus::ACTIVATED) {
      fieldElement.setStatus(Field::ElementStatus::NOT_ACTIVATED);
    }
    fieldElement.sendStatus();
  }
}
