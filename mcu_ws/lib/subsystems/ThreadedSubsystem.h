/**
 * @file ThreadedSubsystem.h
 * @brief Backward-compatibility alias — ThreadedSubsystem is now RTOSSubsystem.
 */
#pragma once

#include "RTOSSubsystem.h"

namespace Subsystem {
using ThreadedSubsystem = RTOSSubsystem;
}  // namespace Subsystem
