#!/usr/bin/env python3
"""
ATTACK DRONE AUTONOMY SCRIPT

DESIGN INTENT:
- This script provides mid-level autonomy.
- The autopilot still handles stabilization, attitude control, and safety.
- This script ONLY provides guidance (velocity + heading) in GUIDED mode.
"""

import math
import time
from pymavlink import mavutil

# =========================
# CONFIGURATION
# =========================

DEVICE = "/dev/ttyUSB0"
BAUD   = 57600

# DESIGN NOTE:
# MAVLink system IDs must be unique on the network.
# Hard-coding these avoids ambiguity when multiple vehicles are present.
ATTACK_SYSID = 1
TARGET_SYSID = 2

# Orbit parameters
ORBIT_RADIUS = 150.0          # meters
ORBIT_DIR    = 1              # +1 CW, -1 CCW

# DESIGN NOTE:
# 150 m was selected as:
# - Outside typical prop-wash / collision envelope
# - Inside visual & sensor tracking range
# - Large enough to avoid aggressive banking in orbit

# Speed limits
MIN_SPEED    = 6.0            # m/s
MAX_SPEED    = 25.0           # m/s
SPEED_MARGIN = 1.3

# DESIGN NOTE:
# SPEED_MARGIN > 1 ensures the attacker can actually close distance.
# 1.3 gives a 30% advantage without forcing max throttle constantly.

CONTROL_ENABLED = True
COMMAND_RATE_HZ = 10

# DESIGN NOTE:
# 10 Hz is a compromise:
# - Fast enough for smooth guidance
# - Slow enough to avoid flooding MAVLink or fighting the autopilot

# RC channels (1-based)
RC_ATTACK_CH = 6
RC_ABORT_CH  = 7
RC_HIGH      = 1600

# DESIGN NOTE:
# RC channels are intentionally simple HIGH/LOW switches.
# This minimizes pilot error during stressful demos or flight tests.

MAX_TURN_RATE_DEG_S = 18.0

# DESIGN NOTE:
# 18 deg/s approximates safe fixed-wing turn limits
# Prevents snap rolls, stalls, or excessive bank angles

# =========================
# UTILITY FUNCTIONS
# =========================

def wrap_360(deg):
    return deg % 360

def angle_diff(a, b):
    # DESIGN NOTE:
    # Ensures shortest-turn logic (e.g. 350° → 10° turns +20°, not -340°)
    return (a - b + 180) % 360 - 180

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def deg2rad(d):
    return math.radians(d)

def rad2deg(r):
    return math.degrees(r)

def gps_to_ned(lat, lon, lat0, lon0):
    """
    Converts GPS into local N/E meters.

    DESIGN NOTE:
    Flat-earth approximation is valid here because:
    - Engagement ranges are < 1 km
    - Error << orbit radius
    """
    R = 6378137.0
    dlat = deg2rad(lat - lat0)
    dlon = deg2rad(lon - lon0)
    north = dlat * R
    east  = dlon * R * math.cos(deg2rad(lat0))
    return north, east

def bearing_from_ned(n, e):
    return wrap_360(rad2deg(math.atan2(e, n)))

# =========================
# STATE CONTAINERS
# =========================

class DroneState:
    """
    Stores the latest telemetry snapshot for a drone.
    """
    def __init__(self):
        self.lat = None
        self.lon = None
        self.vx  = 0.0
        self.vy  = 0.0
        self.hdg = None
        self.mode = None

    def valid(self):
        # DESIGN NOTE:
        # Position validity gates all autonomy.
        # No GPS = no autonomy.
        return self.lat is not None and self.lon is not None

# =========================
# MAVLINK CONNECTION
# =========================

print("Connecting to MAVLink...")
mav = mavutil.mavlink_connection(DEVICE, baud=BAUD)
mav.wait_heartbeat()
print("MAVLink connected")

attack = DroneState()
target = DroneState()

rc_attack = False
rc_abort  = False

last_cmd_time = 0
last_heading_cmd = None
last_time = time.time()

# =========================
# MAIN LOOP
# =========================

while True:
    now = time.time()
    dt = now - last_time
    last_time = now

    msg = mav.recv_match(blocking=False)
    if msg:
        sysid = msg.get_srcSystem()
        mtype = msg.get_type()

        # -------- RC INPUT --------
        if mtype == "RC_CHANNELS":
            ch = msg.chan_raw
            if len(ch) >= max(RC_ATTACK_CH, RC_ABORT_CH):
                rc_attack = ch[RC_ATTACK_CH - 1] > RC_HIGH
                rc_abort  = ch[RC_ABORT_CH  - 1] > RC_HIGH

        # DESIGN NOTE:
        # RC parsing is passive (read-only).
        # Pilot can ALWAYS override by switching modes on transmitter.

        # -------- ATTACK DRONE --------
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

        # -------- TARGET DRONE --------
        if sysid == TARGET_SYSID:
            if mtype == "GLOBAL_POSITION_INT":
                target.lat = msg.lat / 1e7
                target.lon = msg.lon / 1e7

            elif mtype == "LOCAL_POSITION_NED":
                target.vx = msg.vx
                target.vy = msg.vy

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
        print("Waiting for pilot intervention")

        # DESIGN NOTE:
        # Abort intentionally does NOT send commands.
        # Autopilot reverts to pilot control instantly.
        time.sleep(0.1)
        continue

    # =========================
    # RELATIVE GEOMETRY
    # =========================

    Nt, Et = gps_to_ned(target.lat, target.lon,
                        attack.lat, attack.lon)

    distance = math.hypot(Nt, Et)
    bearing  = bearing_from_ned(Nt, Et)

    # Unit LOS vector
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

    # DESIGN NOTE:
    # Speed is adaptive, not fixed.
    # This allows interception of both slow and fast targets.

    # =========================
    # MODE LOGIC
    # =========================

    if rc_attack:
        mode = "ATTACK"
        desired_heading = bearing
        commanded_speed = MAX_SPEED

        # DESIGN NOTE:
        # ATTACK ignores orbit logic intentionally.
        # This is a terminal, pilot-authorised behaviour.

    elif distance > ORBIT_RADIUS + 20:
        mode = "INTERCEPT"
        desired_heading = bearing

        # DESIGN NOTE:
        # +20 m hysteresis prevents orbit/intercept mode thrashing.

    else:
        mode = "ORBIT"
        angle = math.atan2(Et, Nt)

        vN = -ORBIT_DIR * commanded_speed * math.sin(angle)
        vE =  ORBIT_DIR * commanded_speed * math.cos(angle)
        desired_heading = bearing_from_ned(vN, vE)

        # DESIGN NOTE:
        # Orbit is velocity-tangent based.
        # This avoids waypoint chasing and produces smoother circles.

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

        # DESIGN NOTE:
        # Turn-rate limiting is CRITICAL for fixed-wing safety.

    # =========================
    # VELOCITY VECTOR
    # =========================

    vN = commanded_speed * math.cos(deg2rad(desired_heading))
    vE = commanded_speed * math.sin(deg2rad(desired_heading))
    vD = 0.0

    # =========================
    # TIME-TO-INTERCEPT
    # =========================

    target_radial_speed = target.vx * uN + target.vy * uE
    closing_speed = commanded_speed - target_radial_speed

    if closing_speed > 0.5 and distance > ORBIT_RADIUS:
        time_to_intercept = (distance - ORBIT_RADIUS) / closing_speed
    else:
        time_to_intercept = None

    # DESIGN NOTE:
    # Time-to-intercept is advisory only.
    # It is NOT used for control, only situational awareness.

    # =========================
    # SEND COMMAND
    # =========================

    if (CONTROL_ENABLED and
        attack.mode == "GUIDED" and
        now - last_cmd_time > 1.0 / COMMAND_RATE_HZ):

        mav.mav.set_position_target_local_ned_send(
            int(now * 1e6),
            ATTACK_SYSID,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            vN, vE, vD,
            0, 0, 0,
            deg2rad(desired_heading),
            0
        )
        last_cmd_time = now

        # DESIGN NOTE:
        # We command velocity, NOT position.
        # This allows smooth continuous guidance and easy abort.

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
