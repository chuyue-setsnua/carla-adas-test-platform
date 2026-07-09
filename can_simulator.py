"""
CAN Bus Simulator for CARLA
============================

Simulates automotive CAN bus data transmission from CARLA sensor data.
Generates standard CAN frames and logs them in Vector .asc format,
compatible with CANalyzer, CANoe, and python-can.

Standard CAN Message IDs
------------------------
  0x0C0 - Vehicle speed (front wheel speed, km/h * 100, 2 bytes)
  0x0C4 - Steering angle (deg * 10, signed, 2 bytes) + steering rate
  0x1A0 - Brake pressure (bar * 10, 2 bytes)
  0x1A4 - Throttle position (0-255, 1 byte)
  0x200 - Yaw rate (deg/s * 10, signed, 2 bytes)
  0x220 - Longitudinal acceleration (m/s^2 * 100, signed, 2 bytes)
  0x300 - Radar target distance (m * 10, 2 bytes) + relative speed (m/s * 10, signed, 2 bytes)

Usage
-----
    from can_simulator import CANDatalogger

    logger = CANDatalogger("E:/CARLA/test_report/can_log.asc")
    # In main loop:
    frames = logger.log_from_vehicle(ego_vehicle, lead_vehicle)
    for f in frames:
        logger.log_frame(f)
    logger.close()

    # Or use the helper directly:
    from can_simulator import encode_can_data
    frames = encode_can_data(ego_vehicle, ego_vehicle, lead_vehicle)
"""

import math
import struct
import time
from datetime import datetime


# ---------------------------------------------------------------------------
# CAN Message ID constants
# ---------------------------------------------------------------------------
CAN_ID_VEHICLE_SPEED   = 0x0C0
CAN_ID_STEERING        = 0x0C4
CAN_ID_BRAKE_PRESSURE  = 0x1A0
CAN_ID_THROTTLE        = 0x1A4
CAN_ID_YAW_RATE        = 0x200
CAN_ID_LONGITUDINAL_ACCEL = 0x220
CAN_ID_RADAR_TARGET    = 0x300

# Human-readable names for each CAN ID
CAN_ID_NAMES = {
    CAN_ID_VEHICLE_SPEED:      "VehicleSpeed",
    CAN_ID_STEERING:           "SteeringAngle",
    CAN_ID_BRAKE_PRESSURE:     "BrakePressure",
    CAN_ID_THROTTLE:           "ThrottlePosition",
    CAN_ID_YAW_RATE:           "YawRate",
    CAN_ID_LONGITUDINAL_ACCEL: "LongitudinalAccel",
    CAN_ID_RADAR_TARGET:       "RadarTarget",
}


# ---------------------------------------------------------------------------
# CANFrame
# ---------------------------------------------------------------------------
class CANFrame:
    """Represents a single CAN 2.0A frame (standard 11-bit identifier).

    Attributes
    ----------
    can_id : int
        11-bit CAN identifier (e.g. 0x0C0).
    dlc : int
        Data Length Code, always 8 for classic CAN.
    data : bytearray
        8-byte payload.
    timestamp : float
        Absolute timestamp in seconds (epoch or simulation time).
    channel : int
        Logical CAN channel (default 1).
    direction : str
        ``"Rx"`` or ``"Tx"`` (default ``"Rx"``).
    """

    DLC = 8  # classic CAN always uses 8-byte payloads

    def __init__(self, can_id, data=None, timestamp=None, channel=1, direction="Rx"):
        self.can_id = can_id & 0x7FF  # mask to 11 bits
        self.dlc = self.DLC
        self.data = bytearray(data) if data is not None else bytearray(self.DLC)
        # Pad or truncate to exactly 8 bytes
        if len(self.data) < self.DLC:
            self.data.extend(b'\x00' * (self.DLC - len(self.data)))
        self.data = self.data[:self.DLC]
        self.timestamp = timestamp if timestamp is not None else time.time()
        self.channel = channel
        self.direction = direction

    # ------------------------------------------------------------------
    # Encoding helpers
    # ------------------------------------------------------------------
    def encode(self):
        """Pack the frame into raw bytes (ID + DLC + data).

        Returns
        -------
        bytes
            4-byte big-endian CAN ID followed by 8 data bytes (12 bytes total).
        """
        return struct.pack(">I", self.can_id) + bytes(self.data)

    def to_asc(self, base_time=None):
        """Format the frame as a single Vector .asc log line.

        Parameters
        ----------
        base_time : float or None
            If provided, the timestamp in the output is relative to this base.
            Otherwise the absolute ``self.timestamp`` is used as-is.

        Returns
        -------
        str
            A line such as::

                0.050000 1  0C0             Rx   d 8 00 00 00 00 00 00 00 00
        """
        ts = self.timestamp if base_time is None else (self.timestamp - base_time)
        if ts < 0:
            ts = 0.0

        hex_data = " ".join(f"{b:02X}" for b in self.data)
        can_id_str = f"{self.can_id:03X}"
        # The .asc format pads the CAN ID field to ~15 characters
        line = (
            f"  {ts:12.6f} {self.channel}  "
            f"{can_id_str:<15s} {self.direction}   "
            f"d {self.dlc} {hex_data}"
        )
        return line

    def __repr__(self):
        name = CAN_ID_NAMES.get(self.can_id, "Unknown")
        hex_data = " ".join(f"{b:02X}" for b in self.data)
        return (
            f"CANFrame(id=0x{self.can_id:03X} [{name}], "
            f"data=[{hex_data}], ts={self.timestamp:.6f})"
        )


# ---------------------------------------------------------------------------
# Encoding helpers (signal-level)
# ---------------------------------------------------------------------------
def _clamp(value, lo, hi):
    """Clamp *value* to the range [lo, hi]."""
    return max(lo, min(hi, value))


def _pack_uint16(value):
    """Pack an unsigned 16-bit integer (big-endian) into 2 bytes."""
    value = _clamp(int(round(value)), 0, 0xFFFF)
    return struct.pack(">H", value)


def _pack_int16(value):
    """Pack a signed 16-bit integer (big-endian) into 2 bytes."""
    value = _clamp(int(round(value)), -32768, 32767)
    return struct.pack(">h", value)


def _pack_uint8(value):
    """Pack an unsigned 8-bit integer into 1 byte."""
    value = _clamp(int(round(value)), 0, 255)
    return struct.pack("B", value)


def _velocity_to_kmh(velocity):
    """Convert a CARLA ``carla.Vector3D`` velocity (m/s) to km/h scalar speed."""
    speed_ms = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    return speed_ms * 3.6


def _velocity_to_kmh_longitudinal(velocity, transform):
    """Compute longitudinal speed in km/h from velocity and vehicle transform.

    Falls back to total speed magnitude when the forward vector cannot be
    determined (e.g. running outside CARLA).
    """
    try:
        forward = transform.get_forward_vector()
        # Dot product gives the longitudinal component
        v_long = velocity.x * forward.x + velocity.y * forward.y
        return abs(v_long) * 3.6
    except Exception:
        return _velocity_to_kmh(velocity)


# ---------------------------------------------------------------------------
# CAN frame builders (one per message ID)
# ---------------------------------------------------------------------------
def build_vehicle_speed_frame(speed_kmh, timestamp=None):
    """Build a 0x0C0 Vehicle Speed frame.

    Signal layout (big-endian):
        Bytes 0-1: Front wheel speed, unsigned, km/h * 100
        Bytes 2-7: Reserved (0x00)

    Parameters
    ----------
    speed_kmh : float
        Vehicle speed in km/h (non-negative).
    timestamp : float or None
        Frame timestamp.
    """
    raw = _pack_uint16(speed_kmh * 100)
    data = bytearray(raw + b'\x00' * 6)
    return CANFrame(CAN_ID_VEHICLE_SPEED, data, timestamp)


def build_steering_frame(steering_angle_deg, steering_rate_deg_s=0.0, timestamp=None):
    """Build a 0x0C4 Steering Angle frame.

    Signal layout (big-endian):
        Bytes 0-1: Steering angle, signed, degrees * 10
        Bytes 2-3: Steering rate, signed, deg/s * 10
        Bytes 4-7: Reserved (0x00)

    Parameters
    ----------
    steering_angle_deg : float
        Steering wheel angle in degrees (positive = left).
    steering_rate_deg_s : float
        Rate of change of steering angle in deg/s.
    timestamp : float or None
        Frame timestamp.
    """
    raw_angle = _pack_int16(steering_angle_deg * 10)
    raw_rate = _pack_int16(steering_rate_deg_s * 10)
    data = bytearray(raw_angle + raw_rate + b'\x00' * 4)
    return CANFrame(CAN_ID_STEERING, data, timestamp)


def build_brake_pressure_frame(pressure_bar, timestamp=None):
    """Build a 0x1A0 Brake Pressure frame.

    Signal layout (big-endian):
        Bytes 0-1: Brake pressure, unsigned, bar * 10
        Bytes 2-7: Reserved (0x00)

    Parameters
    ----------
    pressure_bar : float
        Brake pressure in bar (non-negative).
    timestamp : float or None
        Frame timestamp.
    """
    raw = _pack_uint16(pressure_bar * 10)
    data = bytearray(raw + b'\x00' * 6)
    return CANFrame(CAN_ID_BRAKE_PRESSURE, data, timestamp)


def build_throttle_frame(throttle_position, timestamp=None):
    """Build a 0x1A4 Throttle Position frame.

    Signal layout (big-endian):
        Byte 0: Throttle position, unsigned, 0-255 (0x00=closed, 0xFF=wide open)
        Bytes 1-7: Reserved (0x00)

    Parameters
    ----------
    throttle_position : float
        Normalised throttle 0.0 .. 1.0 (will be scaled to 0-255).
    timestamp : float or None
        Frame timestamp.
    """
    raw = _pack_uint8(throttle_position * 255)
    data = bytearray(raw + b'\x00' * 7)
    return CANFrame(CAN_ID_THROTTLE, data, timestamp)


def build_yaw_rate_frame(yaw_rate_deg_s, timestamp=None):
    """Build a 0x200 Yaw Rate frame.

    Signal layout (big-endian):
        Bytes 0-1: Yaw rate, signed, deg/s * 10
        Bytes 2-7: Reserved (0x00)

    Parameters
    ----------
    yaw_rate_deg_s : float
        Yaw rate in degrees/second (positive = counter-clockwise).
    timestamp : float or None
        Frame timestamp.
    """
    raw = _pack_int16(yaw_rate_deg_s * 10)
    data = bytearray(raw + b'\x00' * 6)
    return CANFrame(CAN_ID_YAW_RATE, data, timestamp)


def build_longitudinal_accel_frame(accel_ms2, timestamp=None):
    """Build a 0x220 Longitudinal Acceleration frame.

    Signal layout (big-endian):
        Bytes 0-1: Longitudinal acceleration, signed, m/s^2 * 100
        Bytes 2-7: Reserved (0x00)

    Parameters
    ----------
    accel_ms2 : float
        Longitudinal acceleration in m/s^2.
    timestamp : float or None
        Frame timestamp.
    """
    raw = _pack_int16(accel_ms2 * 100)
    data = bytearray(raw + b'\x00' * 6)
    return CANFrame(CAN_ID_LONGITUDINAL_ACCEL, data, timestamp)


def build_radar_target_frame(distance_m, relative_speed_ms, timestamp=None):
    """Build a 0x300 Radar Target frame.

    Signal layout (big-endian):
        Bytes 0-1: Target distance, unsigned, m * 10
        Bytes 2-3: Relative speed, signed, m/s * 10 (negative = approaching)
        Bytes 4-7: Reserved (0x00)

    Parameters
    ----------
    distance_m : float
        Distance to radar target in metres.
    relative_speed_ms : float
        Relative speed in m/s (negative when closing in).
    timestamp : float or None
        Frame timestamp.
    """
    raw_dist = _pack_uint16(distance_m * 10)
    raw_speed = _pack_int16(relative_speed_ms * 10)
    data = bytearray(raw_dist + raw_speed + b'\x00' * 4)
    return CANFrame(CAN_ID_RADAR_TARGET, data, timestamp)


# ---------------------------------------------------------------------------
# High-level encoder
# ---------------------------------------------------------------------------
def encode_can_data(vehicle, ego_vehicle=None, radar_target_vehicle=None,
                    prev_velocity=None, prev_steering=None, dt=0.05,
                    timestamp=None):
    """Generate a full set of CAN frames from a CARLA vehicle's current state.

    This is the main helper that other CARLA scripts should call each tick.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The ego vehicle whose state is being encoded onto the CAN bus.
    ego_vehicle : carla.Vehicle or None
        Reference ego vehicle for radar calculations.  When *None* the radar
        target frame (0x300) is omitted.  Typically the same as *vehicle*.
    radar_target_vehicle : carla.Vehicle or None
        The lead / target vehicle for the radar distance frame.  When *None*
        the radar frame is omitted.
    prev_velocity : float or None
        Previous longitudinal speed in m/s (used to compute acceleration).
        If *None*, longitudinal acceleration defaults to 0.
    prev_steering : float or None
        Previous steering angle in degrees (used to compute steering rate).
        If *None*, steering rate defaults to 0.
    dt : float
        Simulation tick duration in seconds (default 0.05 s = 20 Hz).
    timestamp : float or None
        Explicit timestamp.  Defaults to ``time.time()``.

    Returns
    -------
    list[CANFrame]
        A list of CANFrame objects ready for logging.
    """
    if timestamp is None:
        timestamp = time.time()

    frames = []

    # -- Extract raw CARLA state -------------------------------------------
    try:
        velocity = vehicle.get_velocity()
        transform = vehicle.get_transform()
    except Exception:
        # Running outside a live CARLA connection -- return empty set
        return frames

    speed_ms = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    speed_kmh = speed_ms * 3.6

    # Longitudinal speed (dot with forward vector)
    try:
        fwd = transform.get_forward_vector()
        v_long = velocity.x * fwd.x + velocity.y * fwd.y
    except Exception:
        v_long = speed_ms

    # Steering: read from the last applied control
    try:
        control = vehicle.get_control()
        steering_angle = -control.steer * 720.0  # CARLA steer [-1,1] -> degrees
        throttle = control.throttle                # 0.0 .. 1.0
        brake = control.brake                      # 0.0 .. 1.0 (normalised)
    except Exception:
        steering_angle = 0.0
        throttle = 0.0
        brake = 0.0

    # Steering rate (deg/s)
    steering_rate = 0.0
    if prev_steering is not None and dt > 0:
        steering_rate = (steering_angle - prev_steering) / dt

    # Longitudinal acceleration (m/s^2)
    long_accel = 0.0
    if prev_velocity is not None and dt > 0:
        long_accel = (v_long - prev_velocity) / dt

    # Yaw rate -- approximate from angular velocity if available, else 0
    yaw_rate = 0.0
    try:
        ang_vel = vehicle.get_angular_velocity()
        yaw_rate = ang_vel.z  # deg/s around Z axis
    except Exception:
        pass

    # Brake pressure heuristic: CARLA brake is 0..1, map to ~0..200 bar
    brake_pressure_bar = brake * 200.0

    # -- Build frames ------------------------------------------------------
    frames.append(build_vehicle_speed_frame(speed_kmh, timestamp))
    frames.append(build_steering_frame(steering_angle, steering_rate, timestamp))
    frames.append(build_brake_pressure_frame(brake_pressure_bar, timestamp))
    frames.append(build_throttle_frame(throttle, timestamp))
    frames.append(build_yaw_rate_frame(yaw_rate, timestamp))
    frames.append(build_longitudinal_accel_frame(long_accel, timestamp))

    # -- Radar target frame (optional) -------------------------------------
    if ego_vehicle is not None and radar_target_vehicle is not None:
        try:
            ego_loc = ego_vehicle.get_transform().location
            tgt_loc = radar_target_vehicle.get_transform().location
            dx = tgt_loc.x - ego_loc.x
            dy = tgt_loc.y - ego_loc.y
            distance = math.sqrt(dx * dx + dy * dy)

            ego_v = ego_vehicle.get_velocity()
            tgt_v = radar_target_vehicle.get_velocity()
            ego_fwd = ego_vehicle.get_transform().get_forward_vector()
            # Relative longitudinal speed (negative = target approaching)
            rel_vx = tgt_v.x - ego_v.x
            rel_vy = tgt_v.y - ego_v.y
            rel_speed = rel_vx * ego_fwd.x + rel_vy * ego_fwd.y

            frames.append(build_radar_target_frame(distance, rel_speed, timestamp))
        except Exception:
            pass

    return frames


# ---------------------------------------------------------------------------
# CANDatalogger
# ---------------------------------------------------------------------------
class CANDatalogger:
    """Logs CAN frames to a Vector .asc file.

    The .asc format is the de-facto industry standard for CAN trace logging
    and can be replayed in Vector CANalyzer / CANoe or parsed with python-can.

    Parameters
    ----------
    filename : str
        Path to the output .asc file.  Parent directories are created
        automatically if they do not exist.
    channel : int
        Default CAN channel number written into each line (default 1).

    Examples
    --------
    >>> from can_simulator import CANDatalogger
    >>> logger = CANDatalogger("E:/CARLA/test_report/can_log.asc")
    >>> # Inside the CARLA main loop:
    >>> frames = logger.log_from_vehicle(ego_vehicle, lead_vehicle)
    >>> for f in frames:
    ...     logger.log_frame(f)
    >>> logger.close()
    """

    def __init__(self, filename="E:/CARLA/test_report/can_log.asc", channel=1):
        import os
        self.filename = filename
        self.channel = channel
        self._frame_count = 0
        self._base_time = time.time()

        # Ensure the output directory exists
        parent = os.path.dirname(self.filename)
        if parent:
            os.makedirs(parent, exist_ok=True)

        # Write .asc header
        now = datetime.now()
        date_str = now.strftime("%a %b %d %I:%M:%S %p %Y")
        self._file = open(self.filename, "w", newline="\n")
        self._file.write(f"date {date_str}\n")
        self._file.write("base hex  timestamps absolute\n")
        self._file.write("internal events logged\n")
        self._file.write(f"Begin Triggerblock {date_str}\n")
        self._write_asc_line(f"   0.000000 Start of measurement")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _write_asc_line(self, line):
        """Write a single line to the .asc file."""
        self._file.write(line + "\n")
        self._file.flush()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def log_frame(self, frame):
        """Write a :class:`CANFrame` to the .asc log.

        Parameters
        ----------
        frame : CANFrame
            The frame to log.
        """
        frame.channel = self.channel
        self._write_asc_line(frame.to_asc(base_time=self._base_time))
        self._frame_count += 1

    def log_from_vehicle(self, vehicle, radar_target_vehicle=None,
                         prev_velocity=None, prev_steering=None, dt=0.05):
        """Extract vehicle state, build all CAN frames, log them, and return.

        This is the primary entry point for CARLA scenario scripts.

        Parameters
        ----------
        vehicle : carla.Vehicle
            The ego vehicle.
        radar_target_vehicle : carla.Vehicle or None
            Optional lead / target vehicle for the radar frame.
        prev_velocity : float or None
            Previous longitudinal speed in m/s for acceleration calculation.
        prev_steering : float or None
            Previous steering angle in degrees for steering-rate calculation.
        dt : float
            Tick duration in seconds (default 0.05).

        Returns
        -------
        list[CANFrame]
            The list of frames that were generated and logged.
        """
        timestamp = time.time()
        frames = encode_can_data(
            vehicle,
            ego_vehicle=vehicle,
            radar_target_vehicle=radar_target_vehicle,
            prev_velocity=prev_velocity,
            prev_steering=prev_steering,
            dt=dt,
            timestamp=timestamp,
        )
        for frame in frames:
            self.log_frame(frame)
        return frames

    def close(self):
        """Finalise and close the .asc file."""
        self._write_asc_line("End TriggerBlock")
        self._file.close()

    def get_message_count(self):
        """Return the total number of CAN frames logged so far.

        Returns
        -------
        int
        """
        return self._frame_count


# ---------------------------------------------------------------------------
# Demo / self-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 72)
    print("  CAN Bus Simulator for CARLA -- Demo")
    print("=" * 72)
    print()

    # --- Build individual frames ------------------------------------------
    print("[1] Building individual CAN frames")
    print("-" * 72)

    ts = 0.0

    f_speed = build_vehicle_speed_frame(85.3, ts)
    print(f"  Speed 85.3 km/h   -> {f_speed}")

    f_steer = build_steering_frame(-12.5, 3.2, ts)
    print(f"  Steer -12.5 deg   -> {f_steer}")

    f_brake = build_brake_pressure_frame(45.0, ts)
    print(f"  Brake 45.0 bar    -> {f_brake}")

    f_thr = build_throttle_frame(0.65, ts)
    print(f"  Throttle 65%      -> {f_thr}")

    f_yaw = build_yaw_rate_frame(-5.7, ts)
    print(f"  Yaw -5.7 deg/s    -> {f_yaw}")

    f_accel = build_longitudinal_accel_frame(-3.2, ts)
    print(f"  Accel -3.2 m/s^2  -> {f_accel}")

    f_radar = build_radar_target_frame(42.5, -8.3, ts)
    print(f"  Radar 42.5m/-8.3  -> {f_radar}")

    print()

    # --- Show .asc output -------------------------------------------------
    print("[2] Vector .asc format output")
    print("-" * 72)

    demo_frames = [f_speed, f_steer, f_brake, f_thr, f_yaw, f_accel, f_radar]
    for f in demo_frames:
        print(f.to_asc(base_time=0.0))

    print()

    # --- Encode/decode round-trip -----------------------------------------
    print("[3] Binary encode() round-trip")
    print("-" * 72)
    raw = f_speed.encode()
    print(f"  Encoded vehicle speed: {raw.hex()}")
    decoded_id = struct.unpack(">I", raw[:4])[0]
    decoded_data = raw[4:]
    speed_raw = struct.unpack(">H", decoded_data[:2])[0]
    print(f"  Decoded ID: 0x{decoded_id:03X}, speed raw: {speed_raw} -> {speed_raw / 100:.1f} km/h")

    print()

    # --- Write a sample .asc file -----------------------------------------
    print("[4] Writing sample .asc file")
    print("-" * 72)

    import os
    sample_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_output")
    os.makedirs(sample_dir, exist_ok=True)
    sample_path = os.path.join(sample_dir, "demo_can_log.asc")

    logger = CANDatalogger(sample_path)
    sim_time = time.time()
    speeds = [0.0, 10.5, 25.3, 40.0, 60.7, 80.0, 80.0, 75.2, 60.0, 40.0]
    for i, spd in enumerate(speeds):
        t = sim_time + i * 0.05
        frames = [
            build_vehicle_speed_frame(spd, t),
            build_steering_frame(5.0 * (1 if i < 5 else -1), 0.0, t),
            build_brake_pressure_frame(0.0 if spd > 50 else 30.0, t),
            build_throttle_frame(0.8 if i < 5 else 0.2, t),
            build_yaw_rate_frame(2.0 * (1 if i < 5 else -2.0), t),
            build_longitudinal_accel_frame(1.5 if i < 5 else -1.0, t),
        ]
        for f in frames:
            logger.log_frame(f)

    logger.close()
    print(f"  Written {logger.get_message_count()} frames to: {sample_path}")

    print()
    print("=" * 72)
    print("  Demo complete. Import this module in your CARLA scripts to use.")
    print("=" * 72)
