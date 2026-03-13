#pragma once
/**
 * @file DroneConfigStore.h
 * @brief NVS (Preferences) persistence for drone flight parameters.
 *
 * Uses index-based keys ("p0" through "p27") because some param names
 * exceed the 15-char ESP32 Preferences key limit.
 */

#include <Preferences.h>
#include <cstdio>

#include "DroneFlightSubsystem.h"

namespace Drone {

class DroneConfigStore {
 public:
  /// Load saved params from NVS and apply to flight controller.
  /// Returns number of params loaded.
  size_t load(DroneFlightSubsystem& flight) {
    Preferences prefs;
    if (!prefs.begin("drone_cfg", true)) {  // read-only
      return 0;
    }

    const char* names[32];
    float defaults[32];
    size_t n = flight.getAllParams(defaults, names, 32);

    size_t loaded = 0;
    for (size_t i = 0; i < n; i++) {
      char key[8];
      snprintf(key, sizeof(key), "p%u", (unsigned)i);
      if (prefs.isKey(key)) {
        float val = prefs.getFloat(key, defaults[i]);
        flight.setParam(names[i], val);
        loaded++;
      }
    }

    prefs.end();
    return loaded;
  }

  /// Save all current params from flight controller to NVS.
  /// Returns number of params saved.
  size_t save(DroneFlightSubsystem& flight) {
    Preferences prefs;
    if (!prefs.begin("drone_cfg", false)) {  // read-write
      return 0;
    }

    const char* names[32];
    float values[32];
    size_t n = flight.getAllParams(values, names, 32);

    size_t saved = 0;
    for (size_t i = 0; i < n; i++) {
      char key[8];
      snprintf(key, sizeof(key), "p%u", (unsigned)i);
      if (prefs.putFloat(key, values[i]) > 0) {
        saved++;
      }
    }

    prefs.end();
    return saved;
  }

  /// Format config as "CONFIG:name=val,name=val,..." debug strings.
  /// Calls debugLog() for each chunk (splits at ~450 chars to stay under
  /// XRCE-DDS MTU). Returns number of chunks published.
  size_t formatConfigString(DroneFlightSubsystem& flight,
                            void (*logFn)(const char*)) {
    const char* names[32];
    float values[32];
    size_t n = flight.getAllParams(values, names, 32);

    char buf[480];
    size_t pos = 0;
    size_t chunks = 0;

    // Start first chunk
    pos = snprintf(buf, sizeof(buf), "CONFIG:");

    for (size_t i = 0; i < n; i++) {
      char entry[64];
      int elen = snprintf(entry, sizeof(entry), "%s=%.6g", names[i], values[i]);

      // Check if adding this entry would overflow the buffer
      if (pos + elen + 2 > sizeof(buf) - 1) {
        // Publish current chunk and start new one
        logFn(buf);
        chunks++;
        pos = snprintf(buf, sizeof(buf), "CONFIG:");
      }

      if (pos > 7) {  // not first entry in chunk
        buf[pos++] = ',';
      }
      memcpy(buf + pos, entry, elen);
      pos += elen;
      buf[pos] = '\0';
    }

    // Publish final chunk
    if (pos > 7) {
      logFn(buf);
      chunks++;
    }

    return chunks;
  }
};

}  // namespace Drone
