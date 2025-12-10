#ifndef ENCODERWRAPPER_H
#define ENCODERWRAPPER_H

#include <BaseDriver.h>
#include <Encoder.h>

namespace Drivers {

struct EncoderDriverSetup : public Classes::BaseSetup {
  int pin1, pin2;
};

struct EncoderData {
  long position;
};

class EncoderDriver : public Classes::BaseDriver {
 public:
  ~EncoderDriver() override = default;

  EncoderDriver(const EncoderDriverSetup& setup)
      : BaseDriver(setup), setup_(setup), encoder_(setup.pin1, setup.pin2) {
    position_.position = 0;
  }

  bool init() override;
  void update() override;
  EncoderData getPosition() { return position_; }
  char* getInfo() override;

 private:
  EncoderDriverSetup setup_;
  Encoder encoder_;
  EncoderData position_;
};
}  // namespace Drivers

#endif
