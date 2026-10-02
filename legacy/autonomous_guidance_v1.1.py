#!/usr/bin/env python3
"""
ATTACK DRONE AUTONOMY SCRIPT

DESIGN INTENT:
- Mid-level autonomy only (guidance, not control)
- Autopilot handles attitude, stabilization, safety
- This script provides velocity + heading in GUIDED mode
"""

# -------------------------
# Standard Python libraries
# -------------------------

import math          # Mathematical functions (trigonometry, sqrt, etc.)
import time          # Timing, rate control, timestamps

# -------------------------
# MAVLink interface library
# -------------------------

from pymavlink import mavutil  # Used to communicate with ArduPilot/PX4 via MAVLink

# =========================
# CONFIGURATION
# =========================

DEVICE = "/dev/ttyUSB0"   # Serial device where the flight controller is connected
BAUD   = 57600            # Baud rate for MAVLink serial communication

# MAVLink system IDs
# These uniquely identify vehicles on the MAVLink network
ATTACK_SYSID = 1          # System ID of the attacking (ownship) drone
TARGET_SYSID = 2          # System ID of the target drone

# -------------------------
# Orbit parameters
# -------------------------

ORBIT_RADIUS = 150.0      # Desired orbit radius around target [meters]
ORBIT_DIR    = 1          # Orbit direction: +1 = clockwise, -1 = counter-clockwise

# -------------------------
# Speed limits
# -------------------------

MIN_SPEED    = 6.0        # Minimum commanded airspeed [m/s]
MAX_SPEED    = 25.0       # Maximum commanded airspeed [m/s]
SPEED_MARGIN = 1.3        # Speed advantage over target (30% faster)

# -------------------------
# Commanding behavior
# -------------------------

CONTROL_ENABLED = True    # Master enable for sending MAVLink commands
COMMAND_RATE_HZ = 10      # Rate at which velocity commands are sent to autopilot

# -------------------------
# RC channel configuration
# -------------------------

RC_ATTACK_CH = 6          # RC channel index for ATTACK command (1-based)
RC_ABORT_CH  = 7          # RC channel index for ABORT command (1-based)
RC_HIGH      = 1600       # PWM threshold considered "switch ON"

# -------------------------
# Turn-rate limiting
# -------------------------

MAX_TURN_RATE_DEG_S = 18.0  # Maximum allowed heading change rate [deg/s]

# =========================
# PHYSICS LIMITS  <<< NEW
# =========================

MAX_CENTRIPETAL_ACC = 5.0  # Maximum allowable lateral acceleration [m/s^2]

# DESIGN NOTE:
# Fixed-wing aircraft cannot sustain arbitrary lateral acceleration.
# a_c = v^2 / r  ⇒  v_max = sqrt(a_max * r)
# 5 m/s^2 ≈ ~0.5 g, conservative and safe for demonstrations.

# =========================
# INTERCEPT LEAD GAIN  <<< NEW
# =========================

LEAD_GAIN = 0.5  # Dimensionless scaling for lead-pursuit aggressiveness

# DESIGN NOTE:
# LEAD_GAIN controls how aggressively we bias toward target velocity.
# 0.3–0.7 is typical; 0.5 is a stable middle ground.

# =========================
# UTILITY FUNCTIONS
# =========================

def wrap_360(deg):
    """
    Wrap an angle into the [0, 360) degree range.
    Prevents heading discontinuities.
    """
    return deg % 360

def angle_diff(a, b):
    """
    Compute the signed smallest difference between two angles (degrees).
    Result is in range [-180, +180].
    """
    return (a - b + 180) % 360 - 180

def clamp(val, lo, hi):
    """
    Constrain a value between lower (lo) and upper (hi) bounds.
    """
    return max(lo, min(hi, val))

def deg2rad(d):
    """
    Convert degrees to radians.
    """
    return math.radians(d)

def rad2deg(r):
    """
    Convert radians to degrees.
    """
    return math.degrees(r)

def gps_to_ned(lat, lon, lat0, lon0):
    """
    Convert GPS latitude/longitude into local NED (North-East-Down) meters.

    Inputs:
    - lat, lon   : target position [deg]
    - lat0, lon0 : reference position [deg]

    Outputs:
    - north, east displacement [meters]
    """
    R = 6378137.0                  # Earth radius [meters]
    dlat = deg2rad(lat - lat0)     # Latitude difference [rad]
    dlon = deg2rad(lon - lon0)     # Longitude difference [rad]

    north = dlat * R
    east  = dlon * R * math.cos(deg2rad(lat0))

    return north, east

def bearing_from_ned(n, e):
    """
    Compute bearing (deg) from North-East components.
    0° = North, 90° = East.
    """
    return wrap_360(rad2deg(math.atan2(e, n)))

# =========================
# STATE CONTAINERS
# =========================

class DroneState:
    """
    Container for the latest known state of a drone.
    Stores only what is required for guidance.
    """
    def __init__(self):
        self.lat = None    # Latitude [deg]
        self.lon = None    # Longitude [deg]
        self.vx  = 0.0     # North velocity [m/s] (NED frame)
        self.vy  = 0.0     # East velocity [m/s] (NED frame)
        self.hdg = None    # Heading [deg]
        self.mode = None   # Autopilot flight mode (string)

    def valid(self):
        """
        Check whether position data is available.
        Autonomy is disabled until GPS is valid.
        """
        return self.lat is not None and self.lon is not None

# =========================
# MAVLINK CONNECTION
# =========================

print("Connecting to MAVLink...")

# Open MAVLink serial connection
mav = mavutil.mavlink_connection(DEVICE, baud=BAUD)

# Block until a heartbeat is received
mav.wait_heartbeat()

print("MAVLink connected")

# Instantiate state objects
attack = DroneState()   # Ownship (attack drone)
target = DroneState()   # Target drone

# RC switch states
rc_attack = False       # True when ATTACK switch is ON
rc_abort  = False       # True when ABORT switch is ON

# Command timing & smoothing state
last_cmd_time = 0               # Timestamp of last MAVLink command sent
last_heading_cmd = None         # Previously commanded heading [deg]
last_time = time.time()         # Used to compute loop dt

# =========================
# MAIN LOOP
# =========================

while True:
    # -------------------------
    # Timing
    # -------------------------

    now = time.time()            # Current wall-clock time [s]
    dt = now - last_time         # Time step since last loop [s]
    last_time = now              # Update timestamp

    # -------------------------
    # MAVLink message handling
    # -------------------------

    msg = mav.recv_match(blocking=False)  # Non-blocking read
    if msg:
        sysid = msg.get_srcSystem()       # Which vehicle sent the message
        mtype = msg.get_type()            # Message type string

        # -------- RC INPUT --------
        if mtype == "RC_CHANNELS":
            ch = msg.chan_raw             # Raw RC PWM values (array)
            if len(ch) >= max(RC_ATTACK_CH, RC_ABORT_CH):
                rc_attack = ch[RC_ATTACK_CH - 1] > RC_HIGH
                rc_abort  = ch[RC_ABORT_CH  - 1] > RC_HIGH

        # -------- ATTACK DRONE TELEMETRY --------
        if sysid == ATTACK_SYSID:
            if mtype == "GLOBAL_POSITION_INT":
                attack.lat = msg.lat / 1e7
                attack.lon = msg.lon / 1e7
            elif mtype == "LOCAL_POSITION_NED":
                attack.vx = msg.vx
                attack.vy = msg.vy
            elif mtype == "VFR_HUD":
                attack.hdg = msg.heading
            elif mtype == "HEARTBEAT":
                attack.mode = mavutil.mode_string_v10(msg)

        # -------- TARGET DRONE TELEMETRY --------
        if sysid == TARGET_SYSID:
            if mtype == "GLOBAL_POSITION_INT":
                target.lat = msg.lat / 1e7
                target.lon = msg.lon / 1e7
            elif mtype == "LOCAL_POSITION_NED":
                target.vx = msg.vx
                target.vy = msg.vy

    # -------------------------
    # Telemetry validity gate
    # -------------------------

    if not attack.valid() or not target.valid():
        time.sleep(0.05)
        continue

    # =========================
    # ABORT HANDLING
    # =========================

    if rc_abort:
        print("\033[2J\033[H", end="")
        print("!!! ABORT ACTIVE !!!")
        print("Autonomy DISABLED")
        time.sleep(0.1)
        continue

    # =========================
    # RELATIVE GEOMETRY
    # =========================

    # Relative position of target w.r.t. attack drone in NED frame
    Nt, Et = gps_to_ned(
        target.lat, target.lon,
        attack.lat, attack.lon
    )

    distance = math.hypot(Nt, Et)       # Horizontal separation [m]
    bearing  = bearing_from_ned(Nt, Et) # Bearing to target [deg]

    # Line-of-sight unit vector
    if distance > 1:
        uN = Nt / distance
        uE = Et / distance
    else:
        uN, uE = 0.0, 0.0

    # =========================
    # SPEED LOGIC
    # =========================

    target_speed = math.hypot(target.vx, target.vy)

    commanded_speed = clamp(
        target_speed * SPEED_MARGIN,
        MIN_SPEED,
        MAX_SPEED
    )

    # =========================
    # MODE LOGIC
    # =========================

    if rc_attack:
        mode = "ATTACK"
        desired_heading = bearing
        commanded_speed = MAX_SPEED

    elif distance > ORBIT_RADIUS + 20:
        mode = "INTERCEPT"

        # LOS unit vector
        rN, rE = uN, uE

        # Target velocity vector
        vtN, vtE = target.vx, target.vy

        # Remove radial component (perpendicular only)
        vt_dot_r = vtN * rN + vtE * rE
        vt_perp_N = vtN - vt_dot_r * rN
        vt_perp_E = vtE - vt_dot_r * rE

        # Lead-biased pursuit vector
        leadN = rN + LEAD_GAIN * vt_perp_N / max(commanded_speed, 1.0)
        leadE = rE + LEAD_GAIN * vt_perp_E / max(commanded_speed, 1.0)

        norm = math.hypot(leadN, leadE)
        if norm > 0:
            leadN /= norm
            leadE /= norm

        desired_heading = bearing_from_ned(leadN, leadE)

    else:
        mode = "ORBIT"

        max_orbit_speed = math.sqrt(MAX_CENTRIPETAL_ACC * ORBIT_RADIUS)
        commanded_speed = min(commanded_speed, max_orbit_speed)

        angle = math.atan2(Et, Nt)
        vN = -ORBIT_DIR * commanded_speed * math.sin(angle)
        vE =  ORBIT_DIR * commanded_speed * math.cos(angle)
        desired_heading = bearing_from_ned(vN, vE)

    # =========================
    # TURN RATE LIMIT
    # =========================

    if attack.hdg is not None:
        if last_heading_cmd is None:
            last_heading_cmd = attack.hdg

        max_delta = MAX_TURN_RATE_DEG_S * dt
        delta = angle_diff(desired_heading, last_heading_cmd)
        delta = clamp(delta, -max_delta, max_delta)

        desired_heading = wrap_360(last_heading_cmd + delta)
        last_heading_cmd = desired_heading

    # =========================
    # VELOCITY VECTOR
    # =========================

    vN = commanded_speed * math.cos(deg2rad(desired_heading))
    vE = commanded_speed * math.sin(deg2rad(desired_heading))
    vD = 0.0

    # =========================
    # TIME-TO-INTERCEPT
    # =========================

    v_rel_N = vN - target.vx
    v_rel_E = vE - target.vy
    closing_speed = v_rel_N * uN + v_rel_E * uE

    if closing_speed > 0.5 and distance > ORBIT_RADIUS:
        time_to_intercept = (distance - ORBIT_RADIUS) / closing_speed
    else:
        time_to_intercept = None

    # =========================
    # SEND COMMAND
    # =========================

    if (CONTROL_ENABLED and
        attack.mode == "GUIDED" and
        now - last_cmd_time > 1.0 / COMMAND_RATE_HZ):

        mav.mav.set_position_target_local_ned_send(
            int(now * 1e6),                         # Time (µs)
            ATTACK_SYSID,                           # Target system
            0,                                      # Target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # NED frame
            0b0000111111000111,                     # Ignore position, accel, yaw rate
            0, 0, 0,                                # Position (ignored)
            vN, vE, vD,                             # Velocity commands [m/s]
            0, 0, 0,                                # Acceleration (ignored)
            deg2rad(desired_heading),               # Yaw [rad]
            0                                       # Yaw rate (ignored)
        )

        last_cmd_time = now

    # =========================
    # DISPLAY
    # =========================

    print("\033[2J\033[H", end="")

    print("ATTACK DRONE")
    print(f"  Mode:    {attack.mode}")
    print(f"  Lat:     {attack.lat:.6f}")
    print(f"  Lon:     {attack.lon:.6f}")
    print(f"  Vn/Ve:   {attack.vx:.1f} / {attack.vy:.1f}\n")

    print("TARGET DRONE")
    print(f"  Lat:     {target.lat:.6f}")
    print(f"  Lon:     {target.lon:.6f}")
    print(f"  Speed:   {target_speed:.1f} m/s\n")

    print("RELATIVE")
    print(f"  Distance: {distance:.1f} m")
    print(f"  Bearing:  {bearing:.1f} deg")

    if time_to_intercept is not None:
        print(f"  Time-to-Intercept: {time_to_intercept:.1f} s")
    else:
        print(f"  Time-to-Intercept: ---")

    print("\nGUIDANCE")
    print(f"  Mode:     {mode}")
    print(f"  Cmd Speed:{commanded_speed:.1f} m/s")
    print(f"  Heading:  {desired_heading:.1f} deg")

    print("\nRC INPUTS")
    print(f"  ATTACK: {'ON' if rc_attack else 'OFF'}")
    print(f"  ABORT:  {'ON' if rc_abort  else 'OFF'}")

    time.sleep(0.05)
