#pragma once

#include <Arduino.h>

// Minimal local stub for CD74HC4067 to avoid external dependency during build.
// This implements a basic interface used by AnalogMuxDriver: constructor
// with four select pins and a channel(uint8_t) method.
class CD74HC4067 {
 public:
  CD74HC4067(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3)
      : s0_(s0), s1_(s1), s2_(s2), s3_(s3) {
    pinMode(s0_, OUTPUT);
    pinMode(s1_, OUTPUT);
    pinMode(s2_, OUTPUT);
    pinMode(s3_, OUTPUT);
  }

  void channel(uint8_t ch) {
    digitalWrite(s0_, ch & 0x01);
    digitalWrite(s1_, (ch >> 1) & 0x01);
    digitalWrite(s2_, (ch >> 2) & 0x01);
    digitalWrite(s3_, (ch >> 3) & 0x01);
  }

 private:
  uint8_t s0_, s1_, s2_, s3_;
};
