# UWB Beacon Firmware

Firmware for the SEC26 UWB positioning beacons. Three beacons (ID 10, 11, 12) are deployed at field corners as anchors for trilateration positioning of the robot, minibot, and drone.

## Hardware

- **MCU:** ESP32-WROOM
- **UWB Module:** Decawave DW3000
- **Transport:** WiFi UDP micro-ROS to Pi agent (port 8888)
- **Operating Mode:** ANCHOR (responder) + inter-beacon TAG ranging

### Pin Assignments

| Function | Pin | Notes |
|----------|-----|-------|
| DW3000 SPI CS | GPIO 5 | Chip select |
| DW3000 Reset | GPIO 4 | Soft reset if not wired |
| DW3000 IRQ | GPIO 17 | Unused (polling mode) |
| SPI MOSI | GPIO 23 | Default SPI |
| SPI MISO | GPIO 19 | Default SPI |
| SPI CLK | GPIO 18 | Default SPI |

### Network Configuration

| Beacon | Env | BEACON_ID | Static IP | MAC | OTA Hostname |
|--------|-----|-----------|-----------|-----|--------------|
| Beacon 1 | `beacon1` | 10 | 192.168.4.20 | AA:BB:CC:DD:EE:10 | `sec26-beacon-10` |
| Beacon 2 | `beacon2` | 11 | 192.168.4.21 | AA:BB:CC:DD:EE:11 | `sec26-beacon-11` |
| Beacon 3 | `beacon3` | 12 | 192.168.4.22 | AA:BB:CC:DD:EE:12 | `sec26-beacon-12` |

- **WiFi SSID:** UCFIEEEBot
- **WiFi Password:** goodlife
- **Agent IP:** 192.168.4.1:8888

### Field Placement

| Beacon | Position (m) | Location |
|--------|-------------|----------|
| 10 | x=-7.5, y=-3.5, z=0.15 | Bottom-left corner |
| 11 | x=-7.5, y=3.5, z=0.15 | Top-left corner |
| 12 | x=7.5, y=0.0, z=0.15 | Right side center |

## Building and Flashing

All commands run inside the Docker container.

```bash
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build
pio run -e beacon1
pio run -e beacon2
pio run -e beacon3

# Flash via USB
pio run -e beacon1 --target upload

# Flash via OTA (beacon must be on WiFi)
pio run -e beacon1 --target upload --upload-port 192.168.4.20
pio run -e beacon2 --target upload --upload-port 192.168.4.21
pio run -e beacon3 --target upload --upload-port 192.168.4.22

# Serial monitor
pio device monitor -e beacon1
```

## ROS2 Interface

### Publishers

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/mcu_robot/heartbeat` | `std_msgs/String` | Best-effort | 1 Hz | Heartbeat alive signal |
| `mcu_uwb/range_{my}_{peer}` | `mcu_msgs/UWBRange` | Best-effort | 2 Hz | Inter-beacon range (e.g., `mcu_uwb/range_10_11`) |

Inter-beacon ranging is directional — lower ID initiates to higher ID:
- Beacon 10 publishes: `mcu_uwb/range_10_11`, `mcu_uwb/range_10_12`
- Beacon 11 publishes: `mcu_uwb/range_11_12`
- Beacon 12 publishes: none (no higher-ID peers)

### Subscribers

None.

### Services

None.

## Message Definitions

### `mcu_msgs/msg/UWBRange`

```
std_msgs/Header header
uint8 tag_id            # Initiator device ID
uint8 anchor_id         # Responder device ID
float32 distance        # Distance in centimeters
float32 signal_strength # Quality indicator (always 0.0)
int32 clock_offset      # Raw DW3000 clock offset
uint64 tx_timestamp     # TX timestamp (DW3000 ticks)
uint64 rx_timestamp     # RX timestamp (DW3000 ticks)
bool valid              # True if measurement valid
uint8 error_code        # 0 = no error, 1 = timeout
```

## Usage Examples

### Check beacon topics

```bash
ros2 topic list | grep uwb
```

Expected output (with all 3 beacons online):
```
mcu_uwb/range_10_11
mcu_uwb/range_10_12
mcu_uwb/range_11_12
```

### Monitor inter-beacon ranges

```bash
ros2 topic echo mcu_uwb/range_10_11
ros2 topic hz mcu_uwb/range_10_11   # Expect ~2 Hz
```

### Monitor heartbeat

```bash
ros2 topic echo /mcu_robot/heartbeat
```

### Check all beacons are alive

```bash
ros2 topic hz /mcu_robot/heartbeat   # Expect ~3 Hz (1 Hz × 3 beacons)
```

## Architecture

### UWB Ranging Protocol

Beacons use **double-sided two-way ranging** (DS-TWR) with a 4-frame exchange:

```
TAG (initiator)          ANCHOR (responder)
     |--- Frame 1 -------->|
     |<-- Frame 2 ---------|
     |--- Frame 3 -------->|
     |<-- RTInfo -----------|  (range calculation data)
```

In anchor mode, beacons respond to TAG requests from the robot (ID=13), minibot (ID=14), and drone (ID=15). Lower-ID beacons also act as TAG to range higher-ID peers for self-calibration.

### Inter-Beacon Peer Configuration

```
Beacon 10 → ranges to [11, 12] (2 peers)
Beacon 11 → ranges to [12]     (1 peer)
Beacon 12 → no peers           (highest ID)
```

Peer ranging interval: 500 ms (2 Hz per peer).

### Main Loop (Polling Architecture)

Beacons do not use FreeRTOS tasks. All work runs in the Arduino `loop()`:

```
loop():
  g_wifi.update()        # WiFi reconnection
  ArduinoOTA.handle()    # OTA check
  g_mr.update()          # micro-ROS spin + publish
  g_hb.update()          # Heartbeat (1 Hz)
  g_uwb.update()         # UWB ranging + inter-beacon
  delay(1)               # Yield to WiFi stack
```

### Power Savings (Post-Init)

- CPU frequency: 240 MHz → 80 MHz
- WiFi TX power: 20 dBm → 10 dBm
- WiFi modem sleep: intentionally disabled (causes missed UDP pings)

### Registered Participants

| Participant | Publishers | Subscribers | Services |
|------------|-----------|-------------|----------|
| HeartbeatSubsystem | 1 | 0 | 0 |
| UWBSubsystem | 0–2 (inter-beacon) | 0 | 0 |
