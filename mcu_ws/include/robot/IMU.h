#include <Adafruit_BNO08x.h>
#include <BaseDriver.h>

// i2c address default is 0x4A

namespace Drivers {
struct IMUDriverSetup : Classes::BaseSetup {
  const byte reset_pin = -1;
};

struct IMUDriverData {
  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;
};

class IMUDriver : public Classes::BaseDriver {
 public:
  ~IMUDriver() override = default;

  IMUDriver(const IMUDriverSetup& setup)
      : setup_(setup), BaseDriver(setup), imu_(setup.reset_pin) {};

  bool init() override;
  void update() override;
  char* getInfo() override;

  IMUDriverData getData() { return data_; };

 private:
  IMUDriverSetup setup_;
  IMUDriverData data_;

  Adafruit_BNO08x imu_;
  sh2_SensorValue_t sensorValue_;
};
}  // namespace Drivers
