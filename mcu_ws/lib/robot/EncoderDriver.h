/**
 * @file EncoderDriver.h
 * @author Trevor Cannon
 * @brief Defines the Encoder Driver wrapper for a
 * TeensyDuino Encoder
 * @date 12/10/2025
 */

#ifndef ENCODERWRAPPER_H
#define ENCODERWRAPPER_H

#include <BaseDriver.h>
#include <Encoder.h>

namespace Drivers {

class EncoderDriverSetup : public Classes::BaseSetup {
 public:
  const int pin1, pin2;

  ~EncoderDriverSetup() = default;
  EncoderDriverSetup() = delete;

  EncoderDriverSetup(const char* _id, int p1, int p2)
      : Classes::BaseSetup(_id), pin1(p1), pin2(p2) {};

 private:
};

struct EncoderData {
  long position;
  EncoderData(long position) : position(position) {}
};

class EncoderDriver : public Classes::BaseDriver {
 public:
  ~EncoderDriver() override = default;

  EncoderDriver(const EncoderDriverSetup& setup)
      : BaseDriver(setup),
        setup_(setup),
        encoder_(setup.pin1, setup.pin2),
        position_(0) {};

  bool init() override;
  void update() override;
  EncoderData getPosition() { return position_; }
  const char* getInfo() override;

 private:
  const EncoderDriverSetup setup_;
  Encoder encoder_;
  EncoderData position_;
  char infoBuffer_[64];
};
}  // namespace Drivers

#endif
