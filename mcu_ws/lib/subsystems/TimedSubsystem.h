/**
 * @file TimedSubsystem.h
 * @brief Backward-compatibility alias — TimedSubsystem is now RTOSSubsystem.
 */
#pragma once

#include "RTOSSubsystem.h"

namespace Subsystem {
using TimedSubsystem = RTOSSubsystem;
}  // namespace Subsystem
