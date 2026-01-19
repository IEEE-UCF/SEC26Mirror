#include <field-colors.h>
#include <field-element.h>
#include <keypad.h>
#include <raw-rgb-analog-led.h>

const int ROW_NUM = 4;
const int COLUMN_NUM = 3;

char keys[ROW_NUM][COLUMN_NUM] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}

};
// in ascending order
byte row_pins[ROW_NUM] =
    // R1,R2,R3,R4
    // 7,2,3,5
    {23, 22, 21, 19};  // look at a adafruit 3x4 phone-style matrix keypad(model
                       // 1824 or PID 1824)
byte column_pins[COLUMN_NUM] =
    // C1,C2,C3
    // 6,8,4
    {33, 25, 18};
int led_pin = 4;
int r = 5;
int g = 16;
int b = 17;

// RGB setup and class
RawDrivers::RGBAnalogLEDSetup setup1 =
    RawDrivers::RGBAnalogLEDSetup("Antenna4", {r, g, b});

RawDrivers::RGBAnalogLED RGB1 = RawDrivers::RGBAnalogLED(setup1);

Field::KeypadSetup setupKeypad = Field::KeypadSetup("Keypad");
Field::KeypadDriver Keypad1 = Field::KeypadDriver(
    setupKeypad, makeKeymap(keys), row_pins, column_pins, ROW_NUM, COLUMN_NUM);

// Field element for ESP-NOW communication
Field::FieldElement fieldElement(Field::ElementId::KEYPAD);

// Color options
RawDrivers::RGBColor* colorOptions[] = {
    &Field::colorRed, &Field::colorGreen, &Field::colorBlue, &Field::colorPurple};
Field::FieldColor fieldColorOptions[] = {
    Field::FieldColor::RED, Field::FieldColor::GREEN,
    Field::FieldColor::BLUE, Field::FieldColor::PURPLE};

int randomNum;
bool taskCompleted = false;
bool cycleDisplayActive = false;
uint8_t cycleHue = 0;
unsigned long lastCycleUpdate = 0;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 1000;

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
    Keypad1.reset();
    taskCompleted = false;
    cycleDisplayActive = false;
    digitalWrite(led_pin, LOW);
    RGB1.setColor(Field::colorOff);
    RGB1.update();
  } else if (cmd == Field::FieldCommand::CYCLE_DISPLAY) {
    cycleDisplayActive = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Field-Keypad starting...");

  RGB1.init();
  Keypad1.init();

  pinMode(led_pin, OUTPUT);

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

  Serial.println("Ready - enter password");
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
    Keypad1.update();

    if (Keypad1.getStatus() && !taskCompleted) {
      Serial.println("PASSWORD SUCCESS - Task complete!");
      taskCompleted = true;
      digitalWrite(led_pin, HIGH);

      fieldElement.setStatus(Field::ElementStatus::ACTIVATED);

      RGB1.setColor(*colorOptions[randomNum]);
      RGB1.update();
    }
  }

  // Send periodic status updates
  if (now - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
    lastStatusUpdate = now;
    if (!taskCompleted && fieldElement.getStatus() != Field::ElementStatus::ACTIVATED) {
      fieldElement.setStatus(Field::ElementStatus::NOT_ACTIVATED);
    }
    fieldElement.sendStatus();
  }
}
