/**
 * @file TankDriveLocalization.h
 * @author Trevor Cannon
 * @brief Track robot's position using gyro and encoder readings for tank drive
 * @date 12/14/2025
 * @refactored 12/21/2025 - Aldem Pido - Added setup object pattern
 * @refactored 12/22/2025 - Renamed to TankDriveLocalization
 */

#ifndef TANKDRIVELOCALIZATION_H
#define TANKDRIVELOCALIZATION_H

#define _USE_MATH_DEFINES
#include <Pose2D.h>

#include <cmath>
#include <cstdio>

namespace Drive {

/**
 * @brief Configuration setup for TankDriveLocalization
 * Contains robot-specific constants needed for odometry calculations
 */
class TankDriveLocalizationSetup {
 public:
  TankDriveLocalizationSetup(const char* _id, float _track_width,
                             float _wheel_diameter, int _raw_ticks_per_rev,
                             int _gear_ratio, float _start_x = 0.0f,
                             float _start_y = 0.0f, float _start_theta = 0.0f)
      : id(_id),
        track_width(_track_width),
        wheel_diameter(_wheel_diameter),
        raw_ticks_per_rev(_raw_ticks_per_rev),
        gear_ratio(_gear_ratio),
        start_x(_start_x),
        start_y(_start_y),
        start_theta(_start_theta) {
    // Calculate derived constants
    wheel_radius = wheel_diameter * 0.5f;
    wheel_circumference = M_PI * wheel_diameter;
    ticks_per_revolution = raw_ticks_per_rev * gear_ratio;
    inches_per_tick = wheel_circumference / ticks_per_revolution;
  }

  const char* id;

  // Robot physical constants
  float track_width;
  float wheel_diameter;
  int raw_ticks_per_rev;
  int gear_ratio;

  // Derived constants (calculated in constructor)
  float wheel_radius;
  float wheel_circumference;
  long ticks_per_revolution;
  float inches_per_tick;

  // Starting pose
  float start_x;
  float start_y;
  float start_theta;
};

/**
 * @brief TankDriveLocalization class for tracking robot pose via odometry
 */
class TankDriveLocalization {
 public:
  ~TankDriveLocalization() = default;
  explicit TankDriveLocalization(const TankDriveLocalizationSetup& setup)
      : setup_(setup),
        currentPose(setup.start_x, setup.start_y, setup.start_theta) {}

  Pose2D getPose() const { return currentPose; }
  void update(long leftTicks, long rightTicks, float yaw);
  const char* getInfo();
  void reset();

 private:
  const TankDriveLocalizationSetup& setup_;
  Pose2D currentPose;
  long prevLeftTicks = 0;
  long prevRightTicks = 0;
  char infoBuffer_[64];
};

}  // namespace Drive

#endif
