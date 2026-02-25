# Debug: Micro-ROS Serial Transport Broken on Teensy → Pi

## Context

Micro-ROS on Teensy 4.1 → Raspberry Pi (direct USB serial) is completely broken. **No** micro-ROS program works — including a stock demo from `micro-ROS-demos-platformio` and a bare test environment without TeensyThreads. The agent sees either zero-byte deserialization errors or connect/disconnect loops.

This worked before. It broke after cmake 3.28 workarounds were applied and the TeensyThreads migration happened. Since the bare env (no TeensyThreads) also fails, TeensyThreads is ruled out as the cause. The problem is in the micro-ROS library build (`libmicroros.a`) or the Pi-side environment.

## Systematic Debugging Checklist

Work through these in order. Each level rules out a category of causes.

---

### Level 1: Nuke and rebuild libmicroros.a (MOST LIKELY)

The library was rebuilt after cmake changes. If the build was partial, used wrong flags, or linked against wrong ARM toolchain artifacts, the transport layer inside `libmicroros.a` is corrupt.

```bash
# In Docker container on Pi:
cd /home/ubuntu/mcu_workspaces/sec26mcu

# 1. Check what cmake version is being used during the micro-ROS build
cmake --version
# Expected: 3.28.x (with the workaround applied)

# 2. Check if libmicroros.a exists and is reasonable size (should be 200-500 KB)
ls -la libs_external/teensy/micro_ros_platformio/libmicroros/libmicroros.a

# 3. Verify it's ARM Cortex-M7 code, not x86
# Use PlatformIO's bundled toolchain (arm-none-eabi-nm is NOT installed system-wide)
NM=$(find ~/.platformio/packages/toolchain-gccarmnoneeabi*/bin -name "arm-none-eabi-nm" | head -1)
echo "Using: $NM"
$NM libs_external/teensy/micro_ros_platformio/libmicroros/libmicroros.a | head -20
# Should show ARM symbols. If it shows x86 or errors, the library is corrupt.
#
# Alternative if PlatformIO toolchain not found:
#   file libs_external/teensy/micro_ros_platformio/libmicroros/libmicroros.a
#   # Should say "current ar archive" — then check an object inside:
#   strings libs_external/teensy/micro_ros_platformio/libmicroros/libmicroros.a | grep -i "cortex\|thumb\|aeabi"
#   # Should find ARM-related strings. If empty, likely x86.

# 4. Check that XRCE-DDS transport symbols exist
$NM libs_external/teensy/micro_ros_platformio/libmicroros/libmicroros.a | grep -i "uxr_init_custom_transport\|uxr_run_session_until_confirm\|uxr_create_session"
# Should find matches. If empty, the XRCE-DDS client wasn't built properly.

# 5. FULL nuke and rebuild
rm -rf libs_external/teensy/micro_ros_platformio/libmicroros
rm -rf libs_external/teensy/micro_ros_platformio/build
rm -rf .pio/build

# 6. Rebuild — WATCH THE OUTPUT for errors
pio run -e teensy-test-microros-bare 2>&1 | tee /tmp/microros_build.log

# 7. After build, check for errors in the log
grep -i "error\|fatal\|fail" /tmp/microros_build.log
```

**What to look for**: Any cmake errors about `CMAKE_C_STANDARD_COMPUTED_DEFAULT`, missing GNU targets, or colcon build failures. The build takes 5-15 minutes. If it completes without errors, flash and test.

---

### Level 2: Verify agent/library version match

The micro-ROS agent and `libmicroros.a` must be built for the **same ROS2 distro**. A jazzy agent with a humble library (or vice versa) will fail with protocol mismatches.

```bash
# Check agent version
ros2 run micro_ros_agent micro_ros_agent --help 2>&1 | head -5
# Or:
dpkg -l | grep micro-ros

# Check what distro the library was built for
# (from platformio.ini: board_microros_distro = jazzy)
grep "board_microros_distro" /home/ubuntu/mcu_workspaces/sec26mcu/platformio.ini

# Check ROS2 distro on the Pi
echo $ROS_DISTRO
# Should be "jazzy"

# Verify agent is installed for jazzy
apt list --installed 2>/dev/null | grep micro-ros
```

**Fix if mismatched**: Rebuild the agent for jazzy, or rebuild libmicroros.a for the Pi's distro.

---

### Level 3: Pi-side serial environment

Rule out OS-level interference with the USB serial port.

```bash
# 3a. ModemManager (THE #1 killer of USB serial on Linux)
systemctl status ModemManager
# If active: sudo systemctl stop ModemManager && sudo systemctl disable ModemManager

# 3b. Check USB device enumeration
dmesg | grep -i "ttyACM\|cdc_acm\|teensy\|usb"
# Look for errors, disconnects, or failed enumeration

# 3c. Check nothing else is holding the port
sudo lsof /dev/ttyACM0
# Should show ONLY the micro-ROS agent (or nothing if agent not running)

# 3d. Check permissions
ls -la /dev/ttyACM0
groups
# User must be in 'dialout' group

# 3e. Check serial port configuration
stty -F /dev/ttyACM0 -a
# Look for: crtscts (hardware flow control) — should be OFF
# Fix: stty -F /dev/ttyACM0 raw -echo -echoe -echok -crtscts

# 3f. Check if brltty (Braille display driver) is grabbing the port
# This is another common USB serial killer on Ubuntu
systemctl status brltty
dpkg -l | grep brltty
# If installed: sudo apt remove brltty
```

---

### Level 4: Raw serial verification (bypass micro-ROS entirely)

Confirm the USB serial link actually works between Teensy and Pi.

**On the Teensy** — flash a simple serial echo test (NO micro-ROS):
```cpp
// Use env: teensy-test-serial-echo (or similar non-microros env)
void setup() {
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.write(c);  // echo back
    digitalToggle(LED_BUILTIN);
  }
}
```

**On the Pi**:
```bash
# Send data and check for echo
echo "HELLO" > /dev/ttyACM0
cat /dev/ttyACM0 &
echo "TEST123" > /dev/ttyACM0
# Should see "TEST123" echoed back

# Or use minicom/screen:
screen /dev/ttyACM0 921600
# Type characters — they should echo back, LED should toggle
```

**If echo fails**: The problem is at the USB/serial hardware level (cable, port, driver). Try a different USB port or cable.

**If echo works**: The serial link is fine, the problem is in the micro-ROS library/transport.

---

### Level 5: Transport function verification

If the serial link works but micro-ROS doesn't, add LED-based debugging to the transport functions to verify they're being called.

Temporarily modify `libs_external/teensy/micro_ros_platformio/platform_code/arduino/serial/micro_ros_transport.cpp`:

```cpp
// Add at the top:
static volatile uint32_t write_count = 0;
static volatile uint32_t read_count = 0;

size_t platformio_transport_write(...) {
  // Toggle LED on every write
  write_count++;
  if (write_count % 10 == 0) digitalToggle(LED_BUILTIN);

  Stream * stream = (Stream *) transport->args;
  size_t sent = stream->write(buf, len);
  return sent;
}

size_t platformio_transport_read(...) {
  read_count++;
  Stream * stream = (Stream *) transport->args;
  stream->setTimeout(timeout);
  return stream->readBytes((char *)buf, len);
}
```

Flash the minimal test. If the LED toggles, the transport functions are being called and data is being sent. If the LED never toggles, the XRCE-DDS layer inside libmicroros.a is broken (never calls the transport).

---

### Level 6: Try the upstream micro_ros_platformio (not the IEEE-UCF fork)

If all else fails, test whether the **official** micro_ros_platformio library works:

```bash
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Backup current fork
mv libs_external/teensy/micro_ros_platformio libs_external/teensy/micro_ros_platformio.bak

# Clone official
git clone https://github.com/micro-ROS/micro_ros_platformio.git libs_external/teensy/micro_ros_platformio
cd libs_external/teensy/micro_ros_platformio
git checkout jazzy  # or the appropriate branch

# Rebuild
cd /home/ubuntu/mcu_workspaces/sec26mcu
rm -rf .pio/build
pio run -e teensy-test-microros-bare
```

**If official works but fork doesn't**: The IEEE-UCF fork's `new` branch has a bug. Check git log for recent changes:
```bash
cd libs_external/teensy/micro_ros_platformio.bak
git log --oneline -20
git diff origin/main..HEAD  # or whatever the upstream tracking branch is
```

---

### Level 7: Check the IEEE-UCF fork's git history

```bash
cd libs_external/teensy/micro_ros_platformio
git log --oneline -20
git branch -a
git diff HEAD~5..HEAD -- microros_utils/ platform_code/ metas/
```

Look for any recent changes to:
- `microros_utils/library_builder.py` (build flags, cmake toolchain)
- `microros_utils/repositories.py` (repo URLs, branches, distro mappings)
- `platform_code/arduino/serial/` (transport implementation)
- `metas/` (colcon build configuration)
- `extra_script.py` (PlatformIO integration)

---

## Quick Reference: Key Files

| File | Purpose |
|------|---------|
| `libs_external/teensy/micro_ros_platformio/libmicroros/libmicroros.a` | The built micro-ROS static library |
| `libs_external/teensy/micro_ros_platformio/extra_script.py` | PlatformIO build orchestration |
| `libs_external/teensy/micro_ros_platformio/microros_utils/library_builder.py` | CMake toolchain + colcon build |
| `libs_external/teensy/micro_ros_platformio/platform_code/arduino/serial/micro_ros_transport.cpp` | Serial transport (write/read) |
| `libs_external/teensy/micro_ros_platformio/metas/colcon.meta` | Default entity limits |
| `custom_microros.meta` | Project override (15 pub, 4 srv) |
| `platformio.ini` | `[teensy_microros_bare]` and `[teensy_microros]` configs |

## Decision Tree

```
Start here
│
├─ Level 1: Full nuke + rebuild
│   ├─ Build fails → Fix cmake/colcon errors
│   └─ Build succeeds → Flash and test
│       ├─ Works → DONE (was corrupt library)
│       └─ Still broken → Continue
│
├─ Level 2: Agent version match?
│   ├─ Mismatch → Rebuild agent or library
│   └─ Match → Continue
│
├─ Level 3: Pi serial environment
│   ├─ ModemManager/brltty running → Disable, retest
│   └─ Clean → Continue
│
├─ Level 4: Raw serial echo test
│   ├─ Echo fails → Hardware/driver issue (cable, port, kernel)
│   └─ Echo works → Continue
│
├─ Level 5: Transport function LED test
│   ├─ LED never toggles → libmicroros.a internals broken
│   └─ LED toggles → HDLC framing or agent-side issue
│
├─ Level 6: Try official micro_ros_platformio
│   ├─ Official works → IEEE-UCF fork has a bug
│   └─ Official also broken → Pi environment or Teensy USB issue
│
└─ Level 7: Bisect IEEE-UCF fork git history
    └─ Find the breaking commit
```
