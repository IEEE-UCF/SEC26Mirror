#include <field-colors.h>
#include <field-element.h>
#include <pressure.h>
#include <raw-rgb-analog-led.h>

const uint8_t forcePin = 34;  // Must use ADC1 pin (32-39) - ADC2 blocked by WiFi/ESP-NOW

const uint8_t r = 18;
const uint8_t g = 5;
const uint8_t b = 19;
const uint8_t ledPin = 4;

// RGB setup and class
RawDrivers::RGBAnalogLEDSetup setup1 =
    RawDrivers::RGBAnalogLEDSetup("Antenna3", {r, g, b});

RawDrivers::RGBAnalogLED RGB1 = RawDrivers::RGBAnalogLED(setup1);

// Pressure configuration
Field::PressureSetup setupPressure = Field::PressureSetup("Pressure", forcePin);
Field::PressureDriver Pressure1 = Field::PressureDriver(setupPressure);

// Field element for ESP-NOW communication
Field::FieldElement fieldElement(Field::ElementId::PRESSURE);

// Color options
RawDrivers::RGBColor* colorOptions[] = {
    &Field::colorRed, &Field::colorGreen, &Field::colorBlue, &Field::colorPurple};
Field::FieldColor fieldColorOptions[] = {
    Field::FieldColor::RED, Field::FieldColor::GREEN,
    Field::FieldColor::BLUE, Field::FieldColor::PURPLE};

int randomNum;
bool cycleDisplayActive = false;
uint8_t cycleHue = 0;
unsigned long lastCycleUpdate = 0;
unsigned long lastStatusUpdate = 0;
unsigned long lastBlinkUpdate = 0;
bool blinkState = false;
bool gameStarted = false;  // True after START command received
bool taskCompleted = false;  // True when duck removed after being placed
const unsigned long STATUS_UPDATE_INTERVAL = 1000;
const unsigned long BLINK_INTERVAL = 100;  // Fast blink (100ms on/off)

// HSV to RGB conversion for smooth cycling
void hsvToRgb(uint8_t h, uint8_t s, uint8_t v, uint8_t& rOut, uint8_t& gOut, uint8_t& bOut) {
  uint8_t region = h / 43;
  uint8_t remainder = (h - (region * 43)) * 6;
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0: rOut = v; gOut = t; bOut = p; break;
    case 1: rOut = q; gOut = v; bOut = p; break;
    case 2: rOut = p; gOut = v; bOut = t; break;
    case 3: rOut = p; gOut = q; bOut = v; break;
    case 4: rOut = t; gOut = p; bOut = v; break;
    default: rOut = v; gOut = p; bOut = q; break;
  }
}

void onCommand(Field::FieldCommand cmd) {
  if (cmd == Field::FieldCommand::START) {
    Pressure1.reset();
    cycleDisplayActive = false;
    blinkState = false;
    gameStarted = true;
    taskCompleted = false;
    digitalWrite(ledPin, LOW);
    RGB1.setColor(Field::colorOff);
    RGB1.update();
    Serial.println("START received - game started");
  } else if (cmd == Field::FieldCommand::CYCLE_DISPLAY) {
    cycleDisplayActive = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Field-Pressure starting...");

  Pressure1.init();
  RGB1.init();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Initialize ESP-NOW
  fieldElement.initComms();
  fieldElement.setCommandCallback(onCommand);

  // Seed random with ESP32 hardware RNG for true randomness on each reset
  randomSeed(esp_random());

  // Generate random color
  randomNum = random(0, 4);
  Serial.print("Random color index: ");
  Serial.println(randomNum);
  Serial.print("Color name: ");
  Serial.println(Field::colorToString(fieldColorOptions[randomNum]));
  Serial.print("RGB values - R:");
  Serial.print(colorOptions[randomNum]->_r);
  Serial.print(" G:");
  Serial.print(colorOptions[randomNum]->_g);
  Serial.print(" B:");
  Serial.println(colorOptions[randomNum]->_b);

  // Send initial color to earth
  fieldElement.sendColorReport(fieldColorOptions[randomNum]);

  Serial.println("Ready - apply pressure");
}

void loop() {
  unsigned long now = millis();

  // Process incoming ESP-NOW messages
  fieldElement.processMessages();

  // Update pressure sensor
  Pressure1.update();
  // Note: getStatus() returns true when duck is OFF (reading <= 30)
  // So we invert it: duckOnPlate = true when duck IS on plate
  bool duckOnPlate = !Pressure1.getStatus();

  // Handle cycle display animation (takes priority)
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
  } else if (taskCompleted) {
    // ACTIVATED: Show solid color, indicator LED on
    RGB1.setColor(*colorOptions[randomNum]);
    RGB1.update();
    digitalWrite(ledPin, HIGH);
  } else if (gameStarted) {
    // NOT_ACTIVATED: Duck should be on plate, waiting for removal
    // LED off while duck is on plate
    RGB1.setColor(Field::colorOff);
    RGB1.update();
    digitalWrite(ledPin, LOW);

    // Check if duck was removed
    if (!duckOnPlate) {
      Serial.println("Duck removed - ACTIVATED!");
      taskCompleted = true;
      fieldElement.setStatus(Field::ElementStatus::ACTIVATED);
      RGB1.setColor(*colorOptions[randomNum]);
      RGB1.update();
      digitalWrite(ledPin, HIGH);
    }
  } else {
    // READY state (before START command)
    // Blink rapidly if duck NOT on plate, turn off if duck IS on plate
    static bool lastDuckState = false;
    if (duckOnPlate != lastDuckState) {
      lastDuckState = duckOnPlate;
      Serial.print("Duck state changed: ");
      Serial.println(duckOnPlate ? "ON PLATE" : "OFF PLATE");
    }

    if (!duckOnPlate) {
      // No duck - blink rapidly
      if (now - lastBlinkUpdate > BLINK_INTERVAL) {
        lastBlinkUpdate = now;
        blinkState = !blinkState;
        if (blinkState) {
          RGB1.setColor(*colorOptions[randomNum]);
        } else {
          RGB1.setColor(Field::colorOff);
        }
        RGB1.update();
      }
    } else {
      // Duck on plate - turn off LED
      RGB1.setColor(Field::colorOff);
      RGB1.update();
    }
  }

  // Send periodic status updates
  if (now - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
    lastStatusUpdate = now;
    if (taskCompleted) {
      fieldElement.setStatus(Field::ElementStatus::ACTIVATED);
    } else if (gameStarted) {
      fieldElement.setStatus(Field::ElementStatus::NOT_ACTIVATED);
    }
    // READY status is set by initComms()
    fieldElement.sendStatus();
  }
}
