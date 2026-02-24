/**
 * @file teensy-test-teensythreads.cpp
 * @brief TeensyThreads blink + serial example for Teensy boards.
 */

#include <Arduino.h>
#include <TeensyThreads.h>

static void task1(void*) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWriteFast(LED_BUILTIN, LOW);
    threads.delay(500);

    digitalWriteFast(LED_BUILTIN, HIGH);
    threads.delay(500);
  }
}

static void task2(void*) {
  while (true) {
    Serial.println("TICK");
    threads.delay(1000);

    Serial.println("TOCK");
    threads.delay(1000);
  }
}

void setup() {
  Serial.begin(0);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(PSTR("\r\nBooting TeensyThreads. Built by gcc " __VERSION__
                      " on " __DATE__ ".\r\n"));

  threads.addThread(task1, nullptr, 512);
  threads.addThread(task2, nullptr, 512);

  Serial.println(PSTR("setup(): threads started."));
  Serial.flush();
}

void loop() { threads.delay(100); }
