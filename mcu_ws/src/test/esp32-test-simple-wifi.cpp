#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  WiFi.begin("UCFIEEEBot", "goodlife");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // This will print a number (1, 4, 5, or 6)
    Serial.print("Status: ");
    Serial.println(WiFi.status()); 
  }
}

void loop() {
    Serial.print("Main loop \n");
    delay(1000);
}