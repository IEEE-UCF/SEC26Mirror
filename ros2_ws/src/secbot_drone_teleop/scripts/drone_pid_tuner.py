#!/usr/bin/env python3
"""
Curses-based TUI for tuning drone PID gains in real time.

Subscribes to /mcu_drone/state for live attitude telemetry and calls
/mcu_drone/set_param to push gain changes to the flight controller.

Full console control: arm, disarm, ready, takeoff, land, tare, save config.
Syncs config from drone on connect via CONFIG: debug messages.
"""

import curses
import threading
import sys
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mcu_msgs.msg import DroneState
from mcu_msgs.srv import DroneArm, DroneSetParam, DroneTakeoff, DroneLand, DroneTare
from std_msgs.msg import String


# ── Parameter definitions (matches DroneFlightSubsystem::setParam) ────────

@dataclass
class Param:
    name: str        # service param_name string
    default: float   # from DroneConfig.h
    value: float = 0.0
    step: float = 0.0
    pending: bool = False  # service call in-flight

    def __post_init__(self):
        self.value = self.default
        if self.step == 0.0:
            self.step = abs(self.default) * 0.1 if self.default != 0.0 else 0.001


@dataclass
class ParamGroup:
    label: str
    params: list  # list[Param]


def build_params() -> list:
    """Build parameter list in tuning order: rate (inner) -> angle (outer) -> yaw -> alt -> scalars."""
    return [
        ParamGroup("Roll Rate (inner)", [
            Param("roll_rate_kp",  0.003),
            Param("roll_rate_ki",  0.001),
            Param("roll_rate_kd",  0.00003),
        ]),
        ParamGroup("Pitch Rate (inner)", [
            Param("pitch_rate_kp", 0.003),
            Param("pitch_rate_ki", 0.001),
            Param("pitch_rate_kd", 0.00003),
        ]),
        ParamGroup("Roll Angle (outer)", [
            Param("roll_angle_kp", 5.0),
            Param("roll_angle_ki", 0.5),
            Param("roll_angle_kd", 0.0),
        ]),
        ParamGroup("Pitch Angle (outer)", [
            Param("pitch_angle_kp", 5.0),
            Param("pitch_angle_ki", 0.5),
            Param("pitch_angle_kd", 0.0),
        ]),
        ParamGroup("Yaw Rate", [
            Param("yaw_rate_kp",   0.005),
            Param("yaw_rate_ki",   0.001),
            Param("yaw_rate_kd",   0.0),
        ]),
        ParamGroup("Altitude", [
            Param("altitude_kp",   1.2),
            Param("altitude_ki",   0.15),
            Param("altitude_kd",   0.0),
        ]),
        ParamGroup("Altitude Velocity", [
            Param("alt_vel_kp",    0.5),
            Param("alt_vel_ki",    0.0),
            Param("alt_vel_kd",    0.0),
        ]),
        ParamGroup("Scalars", [
            Param("hover_throttle",   0.45),
            Param("motor_max_slew",   0.5),
            Param("max_roll_deg",     30.0),
            Param("max_pitch_deg",    30.0),
            Param("max_yaw_rate_dps", 160.0),
            Param("motor_min_output", 0.045),
            Param("ready_throttle",   0.35),
        ]),
    ]


# ── State name lookup ─────────────────────────────────────────────────────

STATE_NAMES = {
    0: "INIT", 1: "UNARMED", 2: "ARMED", 3: "LAUNCHING",
    4: "VEL_CTRL", 5: "LANDING", 6: "EMERG", 7: "READY",
}


# ── ROS2 Node ─────────────────────────────────────────────────────────────

class PidTunerNode(Node):
    def __init__(self):
        super().__init__("drone_pid_tuner")

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_sub = self.create_subscription(
            DroneState, "/mcu_drone/state", self._state_cb, best_effort
        )
        self.debug_sub = self.create_subscription(
            String, "/mcu_drone/debug", self._debug_cb, best_effort
        )
        self.set_param_cli = self.create_client(
            DroneSetParam, "/mcu_drone/set_param"
        )
        self.arm_cli = self.create_client(
            DroneArm, "/mcu_drone/arm"
        )
        self.takeoff_cli = self.create_client(
            DroneTakeoff, "/mcu_drone/takeoff"
        )
        self.land_cli = self.create_client(
            DroneLand, "/mcu_drone/land"
        )
        self.tare_cli = self.create_client(
            DroneTare, "/mcu_drone/tare"
        )
        self.save_cli = self.create_client(
            DroneTare, "/mcu_drone/save_config"
        )
        self.ready_cli = self.create_client(
            DroneTare, "/mcu_drone/ready_for_takeoff"
        )
        self.get_config_cli = self.create_client(
            DroneTare, "/mcu_drone/get_config"
        )

        # Latest telemetry (written by ROS thread, read by curses thread)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alt = 0.0
        self.state = 1  # UNARMED
        self.debug_text = ""
        self.last_result = ""
        self.pending_config = {}  # name -> value from CONFIG: messages
        self._config_requested = False
        self._lock = threading.Lock()

    def _state_cb(self, msg: DroneState):
        should_request = False
        with self._lock:
            self.roll = msg.roll
            self.pitch = msg.pitch
            self.yaw = msg.yaw
            self.alt = msg.altitude
            self.state = msg.state
            if not self._config_requested:
                self._config_requested = True
                should_request = True
        # Request config outside the lock (call_async is non-blocking but
        # keeping service calls outside locks is cleaner).
        if should_request:
            self._request_config()

    def _debug_cb(self, msg: String):
        with self._lock:
            text = msg.data
            # Parse CONFIG: messages for param sync
            if text.startswith("CONFIG:"):
                pairs = text[7:].split(",")
                for pair in pairs:
                    if "=" in pair:
                        name, val_str = pair.split("=", 1)
                        try:
                            self.pending_config[name.strip()] = float(val_str.strip())
                        except ValueError:
                            pass
            else:
                self.debug_text = text

    def send_param(self, param: Param):
        """Async service call. Marks param.pending; cleared on response."""
        if param.pending:
            return
        req = DroneSetParam.Request()
        req.param_name = param.name
        req.value = float(param.value)
        param.pending = True

        future = self.set_param_cli.call_async(req)
        future.add_done_callback(lambda f, p=param: self._param_done(f, p))

    def _param_done(self, future, param: Param):
        param.pending = False
        try:
            resp = future.result()
            with self._lock:
                self.last_result = f"{param.name} = {param.value:.6g}: {resp.message}"
        except Exception as e:
            with self._lock:
                self.last_result = f"ERR {param.name}: {e}"

    def send_arm(self, arm: bool):
        req = DroneArm.Request()
        req.arm = arm
        future = self.arm_cli.call_async(req)
        future.add_done_callback(lambda f, a=arm: self._arm_done(f, a))

    def _arm_done(self, future, arm: bool):
        try:
            resp = future.result()
            action = "ARM" if arm else "DISARM"
            with self._lock:
                self.last_result = f"{action}: {resp.message}"
        except Exception as e:
            with self._lock:
                self.last_result = f"ARM ERR: {e}"

    def send_takeoff(self, altitude: float = 1.0):
        req = DroneTakeoff.Request()
        req.target_altitude = altitude
        future = self.takeoff_cli.call_async(req)
        future.add_done_callback(self._simple_done("TAKEOFF"))

    def send_land(self):
        req = DroneLand.Request()
        future = self.land_cli.call_async(req)
        future.add_done_callback(self._simple_done("LAND"))

    def send_tare(self):
        req = DroneTare.Request()
        future = self.tare_cli.call_async(req)
        future.add_done_callback(self._simple_done("TARE"))

    def send_save_config(self):
        req = DroneTare.Request()
        future = self.save_cli.call_async(req)
        future.add_done_callback(self._simple_done("SAVE"))

    def send_ready(self):
        req = DroneTare.Request()
        future = self.ready_cli.call_async(req)
        future.add_done_callback(self._simple_done("READY"))

    def send_get_config(self):
        req = DroneTare.Request()
        future = self.get_config_cli.call_async(req)
        future.add_done_callback(self._simple_done("GET_CONFIG"))

    def _request_config(self):
        """Request config from drone (called on first state message)."""
        self.send_get_config()

    def _simple_done(self, label: str):
        def callback(future):
            try:
                resp = future.result()
                with self._lock:
                    self.last_result = f"{label}: {resp.message}"
            except Exception as e:
                with self._lock:
                    self.last_result = f"{label} ERR: {e}"
        return callback

    def get_telemetry(self):
        with self._lock:
            return (self.roll, self.pitch, self.yaw, self.alt,
                    self.state, self.debug_text, self.last_result)

    def take_pending_config(self):
        """Take and clear pending config dict (thread-safe)."""
        with self._lock:
            if not self.pending_config:
                return None
            cfg = self.pending_config.copy()
            self.pending_config.clear()
            return cfg


# ── Flat param helpers ────────────────────────────────────────────────────

def flat_params(groups):
    """Return flat list of (group_label_or_None, Param_or_None) for display rows."""
    rows = []
    for g in groups:
        rows.append((g.label, None))  # group header
        for p in g.params:
            rows.append((None, p))    # param row
    return rows


def all_params(groups):
    """Return flat list of all Param objects."""
    return [p for g in groups for p in g.params]


# ── Value formatting ──────────────────────────────────────────────────────

def fmt_val(v):
    av = abs(v)
    if av == 0.0:
        return "0.0"
    if av < 0.001:
        return f"{v:.6f}"
    if av < 0.1:
        return f"{v:.5f}"
    if av < 10.0:
        return f"{v:.4f}"
    return f"{v:.2f}"


def fmt_step(s):
    return fmt_val(s)


# ── Curses TUI ────────────────────────────────────────────────────────────

def run_tui(stdscr, node: PidTunerNode):
    curses.curs_set(0)
    stdscr.timeout(100)  # 10 Hz refresh
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN, -1)     # group headers
    curses.init_pair(2, curses.COLOR_GREEN, -1)     # selected param
    curses.init_pair(3, curses.COLOR_YELLOW, -1)    # pending
    curses.init_pair(4, curses.COLOR_RED, -1)       # changed from default
    curses.init_pair(5, curses.COLOR_WHITE, -1)     # normal

    groups = build_params()
    rows = flat_params(groups)
    params = all_params(groups)
    params_by_name = {p.name: p for p in params}

    # Cursor indexes into params list (not rows)
    cursor = 0
    scroll_offset = 0

    # Input mode
    typing = False
    type_buf = ""
    type_mode = "value"  # "value" or "takeoff"

    while True:
        # ── Apply pending config from drone ──
        cfg = node.take_pending_config()
        if cfg:
            for name, val in cfg.items():
                if name in params_by_name:
                    params_by_name[name].value = val

        # ── Telemetry ──
        roll, pitch, yaw, alt, state, debug_text, last_result = node.get_telemetry()
        state_name = STATE_NAMES.get(state, f"?{state}")

        height, width = stdscr.getmaxyx()
        stdscr.erase()

        # ── Header ──
        row = 0
        title = " DRONE PID TUNER"
        telemetry = f"State: {state_name}   Alt: {alt:.2f}m"
        stdscr.addnstr(row, 0, title, width, curses.A_BOLD | curses.color_pair(1))
        stdscr.addnstr(row, max(len(title) + 2, 25), telemetry, width - 25)
        row += 1

        if debug_text:
            stdscr.addnstr(row, 1, debug_text[:width - 2], width - 2, curses.color_pair(5))
            row += 1

        att_line = f" Roll: {roll:7.1f}    Pitch: {pitch:7.1f}    Yaw: {yaw:7.1f}"
        stdscr.addnstr(row, 0, att_line, width)
        row += 1

        # Separator
        stdscr.addnstr(row, 0, "\u2500" * min(width - 1, 60), width)
        row += 1

        # ── Parameter list (scrollable) ──
        list_start = row
        list_height = height - list_start - 5  # reserve 5 lines for footer
        if list_height < 3:
            list_height = 3

        # Build display rows with param index tracking
        display = []  # (is_header, text, param_obj_or_None, param_idx_or_None)
        pidx = 0
        for label, param in rows:
            if param is None:
                display.append((True, label, None, None))
            else:
                display.append((False, None, param, pidx))
                pidx += 1

        # Find display row of cursor
        cursor_display_row = 0
        for i, (is_hdr, _, _, pi) in enumerate(display):
            if not is_hdr and pi == cursor:
                cursor_display_row = i
                break

        # Adjust scroll
        if cursor_display_row < scroll_offset:
            scroll_offset = cursor_display_row
        if cursor_display_row >= scroll_offset + list_height:
            scroll_offset = cursor_display_row - list_height + 1
        # Keep header visible if possible
        if scroll_offset > 0:
            # Check if first visible row is a param — show its group header
            pass

        visible = display[scroll_offset : scroll_offset + list_height]

        for di, (is_hdr, text, param, pi) in enumerate(visible):
            y = list_start + di
            if y >= height - 5:
                break

            if is_hdr:
                stdscr.addnstr(y, 1, text, width - 2, curses.A_BOLD | curses.color_pair(1))
            else:
                selected = (pi == cursor)
                marker = " > " if selected else "   "
                name_str = f"{marker}{param.name:<22s}"
                val_str = fmt_val(param.value)
                step_str = fmt_step(param.step)

                # Color: green if selected, yellow if pending, red if changed
                if param.pending:
                    attr = curses.color_pair(3)
                elif selected:
                    attr = curses.color_pair(2) | curses.A_BOLD
                elif param.value != param.default:
                    attr = curses.color_pair(4)
                else:
                    attr = curses.color_pair(5)

                line = f"{name_str} {val_str:>12s}   [step: {step_str}]"
                stdscr.addnstr(y, 0, line, width - 1, attr)

        # ── Footer ──
        footer_y = height - 5
        if footer_y > list_start:
            stdscr.addnstr(footer_y, 0, "\u2500" * min(width - 1, 60), width)

        if typing:
            if type_mode == "takeoff":
                prompt = f" Takeoff altitude (m, default 1.0): {type_buf}_"
            else:
                prompt = f" Enter value for {params[cursor].name}: {type_buf}_"
            stdscr.addnstr(footer_y + 1, 0, prompt, width - 1, curses.A_BOLD)
        else:
            keys1 = " jk:nav  hl:fine  []:coarse  Enter:type  c:copy  r:reset  R:resetAll"
            keys2 = " a:arm  d:disarm  D:EMERG  f:ready  t:takeoff  L:land  T:tare  S:save  q:quit"
            stdscr.addnstr(footer_y + 1, 0, keys1[:width - 1], width - 1, curses.color_pair(5))
            stdscr.addnstr(footer_y + 2, 0, keys2[:width - 1], width - 1, curses.color_pair(5))

        if last_result:
            stdscr.addnstr(footer_y + 3, 1, last_result[:width - 2], width - 2, curses.color_pair(5))

        stdscr.refresh()

        # ── Input ──
        try:
            key = stdscr.getch()
        except curses.error:
            continue

        if key == -1:
            continue

        if typing:
            if key in (curses.KEY_ENTER, 10, 13):
                typing = False
                if type_mode == "takeoff":
                    try:
                        alt_val = float(type_buf) if type_buf else 1.0
                    except ValueError:
                        alt_val = 1.0
                    node.send_takeoff(alt_val)
                else:
                    try:
                        params[cursor].value = float(type_buf)
                        node.send_param(params[cursor])
                    except ValueError:
                        pass
                type_buf = ""
                type_mode = "value"
            elif key == 27:  # ESC
                typing = False
                type_buf = ""
                type_mode = "value"
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                type_buf = type_buf[:-1]
            elif 32 <= key < 127:
                ch = chr(key)
                if ch in "0123456789.-eE+":
                    type_buf += ch
            continue

        # Navigation
        if key in (curses.KEY_UP, ord('k')):
            cursor = max(0, cursor - 1)
        elif key in (curses.KEY_DOWN, ord('j')):
            cursor = min(len(params) - 1, cursor + 1)

        # Fine adjust
        elif key in (curses.KEY_RIGHT, ord('l')):
            params[cursor].value += params[cursor].step
            node.send_param(params[cursor])
        elif key in (curses.KEY_LEFT, ord('h')):
            params[cursor].value -= params[cursor].step
            node.send_param(params[cursor])

        # Coarse adjust (5x)
        elif key == ord(']'):
            params[cursor].value += params[cursor].step * 5
            node.send_param(params[cursor])
        elif key == ord('['):
            params[cursor].value -= params[cursor].step * 5
            node.send_param(params[cursor])

        # Type exact value
        elif key in (curses.KEY_ENTER, 10, 13):
            typing = True
            type_buf = ""
            type_mode = "value"

        # Copy roll -> pitch (rate and angle)
        elif key == ord('c'):
            roll_rate = groups[0].params   # Roll Rate
            pitch_rate = groups[1].params  # Pitch Rate
            roll_angle = groups[2].params  # Roll Angle
            pitch_angle = groups[3].params # Pitch Angle
            for src, dst in zip(roll_rate, pitch_rate):
                dst.value = src.value
                node.send_param(dst)
            for src, dst in zip(roll_angle, pitch_angle):
                dst.value = src.value
                node.send_param(dst)

        # Reset selected
        elif key == ord('r'):
            params[cursor].value = params[cursor].default
            node.send_param(params[cursor])

        # Reset ALL
        elif key == ord('R'):
            for p in params:
                p.value = p.default
                node.send_param(p)

        # Arm
        elif key == ord('a'):
            node.send_arm(True)

        # Disarm (normal)
        elif key == ord('d'):
            node.send_arm(False)

        # Emergency disarm (same call, explicit key for flying state)
        elif key == ord('D'):
            node.send_arm(False)

        # Ready for takeoff
        elif key == ord('f'):
            node.send_ready()

        # Takeoff (enter altitude)
        elif key == ord('t'):
            typing = True
            type_buf = ""
            type_mode = "takeoff"

        # Land
        elif key == ord('L'):
            node.send_land()

        # Tare IMU
        elif key == ord('T'):
            node.send_tare()

        # Save config to EEPROM
        elif key == ord('S'):
            node.send_save_config()

        # Quit
        elif key == ord('q'):
            break


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = PidTunerNode()

    # Spin ROS2 in daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(lambda stdscr: run_tui(stdscr, node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
