/**
 * @file field-colors.h
 * @brief Shared RGB color definitions for field elements
 * @date 01/18/2026
 */

#ifndef FIELD_COLORS_H
#define FIELD_COLORS_H

#include <raw-rgb-analog-led.h>

namespace Field {

// Standard colors for field elements
inline RawDrivers::RGBColor colorOff(0, 0, 0);
inline RawDrivers::RGBColor colorRed(150, 0, 0);
inline RawDrivers::RGBColor colorGreen(0, 150, 0);
inline RawDrivers::RGBColor colorBlue(0, 0, 100);
inline RawDrivers::RGBColor colorPurple(200, 0, 150);

}  // namespace Field

#endif
