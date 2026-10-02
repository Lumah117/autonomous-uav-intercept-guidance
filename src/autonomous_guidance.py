#!/usr/bin/env python3
"""

ATTACK DRONE AUTONOMY SCRIPT
Version 1.2
26/01/26
Author: Christopher Mitchell


DESIGN INTENT:
- Script for Mid-level autonomy only (guidance, not control)
- Autopilot handles attitude, stabilization, safety
- This script provides velocity + heading in GUIDED mode
- Script has in-built 'ABORT' and Manual Overrides, allowing pilot to take control whenever. 
- Script also contains logic for payload deployment and parachute deployment

AUTONOMOUS ENGAGEMENT SEQUENCE (AES): 
- Script designed for MANUAL take off (via RC), once aircraft reaches altitude;
    1. Click into 'GUIDED' mode (via RC).
    2. Script will read targets telemetry and position.
    3. Script then calculates the optimium speed, bearing, etc to Intercept Target.
    4. Drone then enters into 'INTERCEPT' mode.
    5. Autonomous flying to get to a distance of 150m from Target.
    6. Once target is 150m away, Drone enters into 'ORBIT' mode.
    7. Drone will then begin to Orbit the target until 'ATTACK' command issued (via RC)
    8. Once 'ATTACK' command received, drone turns to the target and heads straight for it.
    9. Once the Target is within 20m from drone, command issued to deploy NET.
    10. Once net has been deployed, Drone executes a sharp turn away to avoid the target.
    11. The drone then goes back to orbiting the downed target.
    12. Drone continues to ORBIT until Return To Base (RTB) command issued.

"""
# -------------------------
# Standard Dependency libraries
# -------------------------

import math          # Mathematical functions (trigonometry, sqrt, etc.)
import time          # Timing, rate control, timestamps
import matplotlib.pyplot as plt  
from collections import deque
import threading
import sys
import select

# -------------------------
# MAVLink interface library
# -------------------------

from pymavlink import mavutil  # Used to communicate with ArduPilot/PX4 via MAVLink

# =========================
# CONFIGURATION
# =========================

#DEVICE = "/dev/ttyUSB0"   # Serial device where the flight controller is connected
#BAUD   = 57600            # Baud rate for MAVLink serial communication

# =========================
# CONNECTION CONFIGURATION
# =========================

# For Gazebo / SITL use UDP
DEVICE = "udp:127.0.0.1:14541"

# Set to None for UDP
BAUD = None

# MAVLink system IDs
# These uniquely identify vehicles on the MAVLink network
ATTACK_SYSID = 2          # System ID of the attacking (ownship) drone
TARGET_SYSID = 1          # System ID of the target drone

# -------------------------
# Orbit parameters
# -------------------------

ORBIT_RADIUS     = 150.0      # Desired orbit radius around target [meters]
ORBIT_DIR        = 1          # Orbit direction: +1 = clockwise, -1 = counter-clockwise
ORBIT_ERROR_INT  = 0.0
ORBIT_KP         = 0.7
ORBIT_KI         = 0.06

# -------------------------
# Speed limits
# -------------------------

MIN_SPEED    = 12.0        # Minimum commanded airspeed [m/s]
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
RC_PARACHUTE_CH = 8       # RC channel index for PARACHUTE command
RC_RTB_CH = 9             # RC channel index for RTB command

RC_HIGH      = 1600       # PWM threshold considered "switch ON"

# -------------------------
# Turn-rate limiting
# -------------------------

MAX_TURN_RATE_DEG_S = 25.0  # Maximum allowed heading change rate [deg/s]

# =========================
# PHYSICS LIMITS
# =========================

MAX_CENTRIPETAL_ACC = 5.0  # Maximum allowable lateral acceleration [m/s^2]

# DESIGN NOTE:
# Fixed-wing aircraft cannot sustain arbitrary lateral acceleration.
# a_c = v^2 / r  ⇒  v_max = sqrt(a_max * r)
# 5 m/s^2 ≈ ~0.5 g, conservative and safe for demonstrations.

# =========================
# INTERCEPT LEAD GAIN
# =========================

LEAD_GAIN = 0.5  # Dimensionless scaling for lead-pursuit aggressiveness

# DESIGN NOTE:
# LEAD_GAIN controls how aggressively we bias toward target velocity.
# 0.3–0.7 is typical; 0.5 is a stable middle ground.

# =========================
# DEPLOYMENT ENVELOPE LIMITS
# =========================

NET_DEPLOY_DISTANCE_M       = 20.0
NET_MIN_CLOSING_SPEED       = 2.0
NET_MAX_TURN_RATE_DEG_S     = 5.0
NET_MIN_AIRSPEED            = 10.0
NET_MAX_AIRSPEED            = 22.0
NET_MIN_DEPLOY_ALTITUDE     = 50.0    # Minimum altitude limit for net deployment

# DESIGN NOTE:
# These are safety-critical constants.
# They must exist independently of MAVLink connectivity or runtime state.

# =========================
# POST-DEPLOY AVOIDANCE TUNING (VERTICAL CLIMB)
# =========================

AVOIDANCE_DURATION_S = 2.0          # Duration of climb-out [seconds]
AVOIDANCE_CLIMB_RATE = 5.0          # Vertical climb rate [m/s] (positive = UP)
AVOIDANCE_SPEED      = MAX_SPEED    # Maintain forward speed during climb

# DESIGN NOTE:
# Vertical separation is prioritised over horizontal manoeuvring.
# Fixed-wing aircraft can pitch up safely post-deployment to avoid collision.

# =========================
# TELEMETRY VALIDITY LIMITS
# =========================

TARGET_TELEMETRY_TIMEOUT = 0.5  # seconds

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

def ned_offset_to_gps(lat_deg, lon_deg, north_m, east_m):
    """
    Convert a local N/E offset in meters to GPS coordinates.
    """
    R = 6378137.0

    dlat = north_m / R
    dlon = east_m / (R * math.cos(deg2rad(lat_deg)))

    lat_out = lat_deg + rad2deg(dlat)
    lon_out = lon_deg + rad2deg(dlon)

    return lat_out, lon_out

def bearing_from_ned(n, e):
    """
    Compute bearing (deg) from North-East components.
    0° = North, 90° = East.
    """
    return wrap_360(rad2deg(math.atan2(e, n)))

def choose_orbit_direction(attack, Nt, Et):
    """
    Choose orbit direction based on relative geometry and motion.

    Returns:
        +1 → Clockwise
        -1 → Counter-clockwise
    """

    # If velocity is too small, fall back to heading-based logic
    speed = math.hypot(attack.vx, attack.vy)
    if speed < 1.0:
        if attack.hdg is None:
            return 1
        # fallback: smallest turn to tangent
        bearing = bearing_from_ned(Nt, Et)
        tangent_cw  = wrap_360(bearing + 90)
        tangent_ccw = wrap_360(bearing - 90)

        turn_cw  = angle_diff(tangent_cw, attack.hdg)
        turn_ccw = angle_diff(tangent_ccw, attack.hdg)

        return 1 if turn_cw <= turn_ccw else -1

    # --- CORE LOGIC ---
    # Cross product (2D)
    # z = v × r
    cross = attack.vx * Et - attack.vy * Nt

    if cross > 0:
        return -1   # CCW
    elif cross < 0:
        return +1   # CW
    else:
        return 1    # Fallback

def update_live_plot(ax, attack_traj_n, attack_traj_e,
                     target_traj_n, target_traj_e,
                     attack_n, attack_e, target_n, target_e,
                     orbit_radius):
    ax.clear()

    # Plot trails
    ax.plot(attack_traj_e, attack_traj_n, label="Attack Path")
    ax.plot(target_traj_e, target_traj_n, label="Target Path")

    # Plot current positions
    ax.plot(attack_e, attack_n, marker="o", linestyle="None", label="Attack Drone")
    ax.plot(target_e, target_n, marker="x", linestyle="None", label="Target Drone")

    # Plot desired orbit circle around target
    orbit = plt.Circle((target_e, target_n), orbit_radius, fill=False, linestyle="--")
    ax.add_patch(orbit)

    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title("Live Drone Trajectories")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()

    plt.pause(0.001)

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

        # Altitude fields
        self.alt_rel = None   # Relative altitude to home [meters]
        self.alt_msl = None   # Altitude above mean sea level [meters]

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

# IMPORTANT:
# PX4 already binds its local onboard ports (14580 / 14581),
# so the script must LISTEN on the PX4 remote ports instead.
# Based on your mavlink status:
#   Quad  -> remote port 14540
#   Plane -> remote port 14541
mav_attack = mavutil.mavlink_connection(
    "udpin:127.0.0.1:14541",
    source_system=200
)

mav_target = mavutil.mavlink_connection(
    "udpin:127.0.0.1:14540",
    source_system=201
)

print("Waiting for heartbeats...")

# Wait for heartbeat from plane
mav_attack.wait_heartbeat(timeout=10)
print(f"Plane heartbeat received (SYSID {mav_attack.target_system})")

# Wait for heartbeat from quad
mav_target.wait_heartbeat(timeout=10)
print(f"Target heartbeat received (SYSID {mav_target.target_system})")

print("Connections established")

# Use plane connection for commands
mav = mav_attack

# Instantiate state objects
attack = DroneState()
target = DroneState()

# =========================
# REQUEST HOME POSITION
# =========================

# Ask autopilot to send its stored home position
try:
    mav.mav.command_long_send(
        ATTACK_SYSID,
        0,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,
        0,0,0,0,0,0,0
    )
except Exception as e:
    print(f"Home position request failed: {e}")

# Storage for home position once received
home_lat = None
home_lon = None
home_alt = None

# =========================
# RUNTIME STATE VARIABLES
# =========================

sw_attack = False
sw_abort = False 
sw_rtb = False
sw_parachute = False 

# RC switch states
rc_attack = False       # True when ATTACK switch is ON
rc_abort  = False       # True when ABORT switch is ON
rc_parachute = False    # True when PARACHUTE switch is ON
rc_rtb = False          # True when RTB switch is ON

# ABORT LATCH STATE (prevents re-entering autonomy until RC ABORT switch is OFF)
abort_latched = False        # True while ABORT switch is ON
abort_prev = False           # Previous raw RC abort state (edge detect)
abort_mode_sent = False      # Only send LOITER once per ABORT event
abort_recovery_active = False

# PAYLOAD DEPLOYMENT STATE
payload_deployed = False
# Latch: once True, payload cannot be redeployed
# Reset only on reboot or explicit ground command (NOT in flight)

# POST-DEPLOY AVOIDANCE STATE
avoidance_active = False        # True while executing break-away manoeuvre
avoidance_start_time = None     # Timestamp when avoidance began
avoidance_climb_active = False  # True while commanding vertical escape

# PARACHUTE DEPLOYMENT STATE
parachute_deployed = False      # True only after paracute is triggered
parachute_prev = False          # Edge detect
parachute_rising = False        # ensures variable isnt referenced before assignment

# RTB LATCH STATE
rtb_latched = False             # True while RTB is active
rtb_prev = False                # Previous raw RC rtb state (edge detect)
rtb_mode_sent = False           # Only send RTB once per event
rtb_rising = False              # Ensures variable isnt referenced before assignment

# PRINT LATCHES (Prevents message spam in terminal)
abort_message_printed = False
rtb_message_printed = False
parachute_lockout_message_printed = False

# Command timing & smoothing state
last_cmd_time = 0               # Timestamp of last MAVLink command sent
last_heading_cmd = None         # Previously commanded heading [deg]
last_time = time.time()         # Used to compute loop dt

# ORBIT DIRECTIONAL STATE
orbit_dir_locked = False

last_target_msg_time = None



# =========================
# LIVE PLOT STATE
# =========================

PLOT_ENABLED = True
PLOT_HISTORY = 500   # number of past points to keep

attack_traj_n = deque(maxlen=PLOT_HISTORY)
attack_traj_e = deque(maxlen=PLOT_HISTORY)

target_traj_n = deque(maxlen=PLOT_HISTORY)
target_traj_e = deque(maxlen=PLOT_HISTORY)

if PLOT_ENABLED:
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))

# =========================
# PAYLOAD MANAGER
# =========================

def payload_manager(deploy_authorized):
    """
    Handles payload deployment.
    This function is deliberately isolated for safety and reviewability.
    """
    global payload_deployed

    if deploy_authorized and not payload_deployed:
        print("\n*** NET DEPLOYMENT AUTHORIZED ***")

        # NET DEPLOYMENT -  -  - IMPLEMENT HERE:
        # -------------------------------------------- #
        # Trigger payload hardware
        # e.g. GPIO, MAV_CMD_DO_SET_SERVO, etc
        # -------------------------------------------- #

        payload_deployed = True
        print("*** NET DEPLOYED (LATCHED) ***")

        # Trigger post-deployment avoidance manoeuvre
        global avoidance_active, avoidance_start_time
        avoidance_active = True
        avoidance_start_time = time.time()

# =========================
# PARACHUTE MANAGER (NEW)
# =========================

def parachute_manager():
    """
    Handles parachute deployment.
    Isolated for safety, traceability, and clean hardware integration.
    """
    global parachute_deployed

    if parachute_deployed:
        return

    print("\n*** PARACHUTE DEPLOYMENT SEQUENCE INITIATED ***")

    # PARACHUTE DEPLOYMENT - - - IMPLEMENT HERE:
    # -------------------------------------------- #
    # HARDWARE TRIGGER FOR PARACHUTE PCB
    # Insert GPIO / serial / I2C / relay trigger here
    # -------------------------------------------- #

    parachute_deployed = True
    print("*** PARACHUTE RELEASE TRIGGER SENT (LATCHED) ***")

# ========================
# KEYBOARD COMMAND INPUT
# ========================

def keyboard_command_listener():
    """
    Text-based command input for SITL testing.

    Commands:
        attack / attack off
        abort / abort off
        rtb / rtb off
        parachute / parachute off
        status
        help
    """
    global sw_attack, sw_abort, sw_rtb, sw_parachute

    while True:
        try:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                cmd = sys.stdin.readline().strip().lower()
            #cmd = input().strip().lower()

                # --- ATTACK ---
                if cmd == "attack":
                    sw_attack = True
                    print("\n[SW CMD] ATTACK ON")

                elif cmd == "attack off":
                    sw_attack = False
                    print("\n[SW CMD] ATTACK OFF")

                # --- ABORT ---
                elif cmd == "abort":
                    sw_abort = True
                    print("\n[SW CMD] ABORT ON")

                elif cmd == "abort off":
                    sw_abort = False
                    print("[\nSW CMD] ABORT OFF")

                # --- RTB ---
                elif cmd == "rtb":
                    sw_rtb = True
                    print("\n[SW CMD] RTB ON")

                elif cmd == "rtb off":
                    sw_rtb = False
                    print("\n[SW CMD] RTB OFF")

                # --- PARACHUTE ---
                elif cmd == "parachute":
                    sw_parachute = True
                    print("\n[SW CMD] PARACHUTE ON")

                elif cmd == "parachute off":
                    sw_parachute = False
                    print("\n[SW CMD] PARACHUTE OFF")

                # --- STATUS ---
                elif cmd == "status":
                    print(f"[STATUS] ATTACK={sw_attack}, ABORT={sw_abort}, RTB={sw_rtb}, PARACHUTE={sw_parachute}")

                # --- HELP ---
                elif cmd == "help":
                    print("""
Available commands:
  attack / attack off
  abort / abort off
  rtb / rtb off
  parachute / parachute off
  status
  help
""")

                else:
                    print(f"[SW CMD] Unknown command: {cmd}")

        except Exception as e:
            print(f"[SW CMD ERROR] {e}")

# =========================
# START KEYBOARD THREAD (SITL CONTROL)
# =========================
keyboard_thread = threading.Thread(
    target=keyboard_command_listener,
    daemon=True
)
keyboard_thread.start()

# =========================
# MAIN LOOP
# =========================

while True:

    #global ORBIT_ERROR_INT, ORBIT_DIR, orbit_dir_locked
    mode = None   # Safe default for display and logic (prevents mode issues)
    desired_heading = attack.hdg if attack.hdg is not None else 0.0 # Ensures a safe default heading exists
    #avoidance_climb_active = False # Initialising avoidance to false (prevents state issues)
    goto_send_command = False   # Always defined at top of loop
    parachute_rising = False   # Ensure defined before RC handler
    abort_rising = False
    rtb_rising = False
    parachute_rising = False

    # -------------------------
    # Timing
    # -------------------------

    now = time.time()            # Current wall-clock time [s]
    dt = now - last_time         # Time step since last loop [s]
    last_time = now              # Update timestamp

    # -------------------------
    # MAVLink message handling
    # -------------------------

    # Drain BOTH MAVLink sockets each loop so neither vehicle starves the other.
    # Only this telemetry section has been changed; all downstream logic is unchanged.
    pending_messages = []

    while True:
        msg_attack = mav_attack.recv_match(blocking=False)
        if msg_attack is None:
            break
        pending_messages.append(msg_attack)

    while True:
        msg_target = mav_target.recv_match(blocking=False)
        if msg_target is None:
            break
        pending_messages.append(msg_target)

    for msg in pending_messages:
        sysid = msg.get_srcSystem()       # Which vehicle sent the message (ownship vs target telemetry)
        mtype = msg.get_type()            # Message type string

        # -------- RC INPUT --------
        if mtype in ["RC_CHANNELS", "RC_CHANNELS_RAW"]:
            ch = [
                msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw
            ] # Raw RC PWM values (array)

            if len(ch) >= max(RC_ATTACK_CH, RC_ABORT_CH, RC_PARACHUTE_CH, RC_RTB_CH):
                rc_attack = (ch[RC_ATTACK_CH - 1] > RC_HIGH) or sw_attack
                rc_abort  = (ch[RC_ABORT_CH  - 1] > RC_HIGH) or sw_abort
                rc_parachute = (ch[RC_PARACHUTE_CH - 1] > RC_HIGH) or sw_parachute
                rc_rtb = (ch[RC_RTB_CH - 1] > RC_HIGH) or sw_rtb

                # --- ABORT latch logic ---
                abort_latched = rc_abort

                # Rising edge: OFF -> ON (used to send LOITER once)
                abort_rising = (rc_abort and not abort_prev)
                abort_prev = rc_abort

                # Reset one-shot LOITER command when ABORT cancelled
                if not abort_latched:
                    abort_mode_sent = False
                    abort_message_printed = False   # Reset so message prints again next time

                # Parachute rising edge detect (OFF -> ON)
                parachute_rising = (rc_parachute and not parachute_prev)
                parachute_prev = rc_parachute

                # --- RTB latch logic ---
                rtb_latched = rc_rtb

                # Rising edge detect
                rtb_rising = (rc_rtb and not rtb_prev)
                rtb_prev = rc_rtb

                # Reset one-shot RTL command when RTB cancelled
                if not rtb_latched:
                    rtb_mode_sent = False
                    rtb_message_printed = False     # Reset so message prints again next time

        # -------- ATTACK DRONE TELEMETRY --------
        if sysid == ATTACK_SYSID:
            if mtype == "GLOBAL_POSITION_INT":
                attack.lat = msg.lat / 1e7
                attack.lon = msg.lon / 1e7
                attack.alt_msl = msg.alt / 1000.0
                attack.alt_rel = msg.relative_alt / 1000.0
            elif mtype == "LOCAL_POSITION_NED":
                attack.vx = msg.vx
                attack.vy = msg.vy
            elif mtype == "HOME_POSITION":
                home_lat = msg.latitude / 1e7
                home_lon = msg.longitude / 1e7
                home_alt = msg.altitude / 1000.0
            elif mtype == "VFR_HUD":
                attack.hdg = msg.heading
            elif mtype == "HEARTBEAT":
                attack.mode = mavutil.mode_string_v10(msg)

        # -------- TARGET DRONE TELEMETRY --------
        if sysid == TARGET_SYSID:
            if mtype == "GLOBAL_POSITION_INT":
                target.lat = msg.lat / 1e7
                target.lon = msg.lon / 1e7
                target.alt_msl = msg.alt / 1000.0
                target.alt_rel = msg.relative_alt / 1000.0
                last_target_msg_time = now

            elif mtype == "LOCAL_POSITION_NED":
                target.vx = msg.vx
                target.vy = msg.vy

    # -------------------------
    # Telemetry validity gate
    # -------------------------

    if not attack.valid() or not target.valid():
        time.sleep(0.05)    # Loop pacing (~20Hz) to reduce CPU load
        continue

    # =========================
    # APPLY SOFTWARE COMMAND OVERRIDES (CRITICAL)
    # =========================
    if sw_attack:
            rc_attack = True
            sw_attack = False

    if sw_abort:
            rc_abort = True
            sw_abort = False

    if sw_rtb:
        rc_rtb = True
        sw_rtb = False

    if sw_parachute:
        rc_parachute = True
        sw_parachute = False

    # Sync Latches with Command State
    abort_latched = rc_abort
    rtb_latched = rc_rtb

    # =========================
    # COMMAND PRIORITY (MUTUAL EXCLUSION)
    # =========================

    if rc_abort:
        rc_rtb = False
        rc_attack = False

    elif rc_rtb:
        rc_attack = False

    # =========================
    # ABORT HANDLING
    # =========================

    if abort_latched:

        print(">>> ABORT → RETURN TO ORBIT <<<")

        # --- Cancel all behaviours ---
        avoidance_active = False
        avoidance_climb_active = False
        avoidance_start_time = None

        # Cancel RTB + ATTACK
        rc_rtb = False
        rtb_latched = False
        rtb_mode_sent = False
        rc_attack = False

        # Ensure control remains active
        CONTROL_ENABLED = True
        goto_send_command = False

        abort_recovery_active = True
        orbit_dir_locked = False
        ORBIT_ERROR_INT = 0.0

        sw_abort = False
        rc_abort = False
        abort_latched = False

        # Force OFFBOARD (retain script control)
        if abort_rising:
            try:
                mav.set_mode("OFFBOARD")
                print("Switched to OFFBOARD (ABORT)")
            except Exception as e:
                print(f"Failed to set OFFBOARD: {e}")

        # FORCE ORBIT MODE
        mode = "ORBIT"

        # Reset orbit controller (important)
        orbit_dir_locked = False
        ORBIT_ERROR_INT = 0.0

        # Display
        if not abort_message_printed:
            print("\033[2J\033[H", end="")
            print("!!! ABORT ACTIVE !!!")
            print("Returning to ORBIT (OFFBOARD control)")
            abort_message_printed = True

        time.sleep(0.05)
    
    # =========================
    # RTB HANDLING
    # =========================

    if rtb_latched and not abort_latched and not parachute_deployed: #and not avoidance_active:

        # Stop all autonomous behaviour
        avoidance_active = False
        avoidance_climb_active = False
        avoidance_start_time = None
        rc_attack = False
        # Stop controller
        CONTROL_ENABLED = False
        goto_send_command = False

        # Only send RTL once per activation
        if rtb_rising and not rtb_mode_sent:
            try:
                #mav.set_mode("RTL")
                mav.mav.command_long_send(
                    ATTACK_SYSID,
                    0,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )

                # Force PX4 Out of OFFBOARD (SCRIPT control)
                mav.set_mode("AUTO.RTL")

                print("\n!!! RTB TRIGGERED — Switching to RTL Mode !!!")
                rtb_mode_sent = True
            except Exception as e:
                print(f"Failed to set RTL mode: {e}")

        # Display state (Print info to the terminal for user)
        if not rtb_message_printed:
            print("\033[2J\033[H", end="")
            print("!!! RTB ACTIVE !!!")
            print("Autonomy DISABLED — Returning To Base under RTL")
            rtb_message_printed = True

        mode = "RTL"
        time.sleep(0.1)
        #continue

    # =========================
    # PARACHUTE HANDLING
    # =========================

    if parachute_rising and not parachute_deployed:

        parachute_manager()   # Trigger PCB hardware

        print("\n!!! PARACHUTE DEPLOY TRIGGERED — CUTTING MOTORS !!!")

        # --- MOTOR CUT (Flight Termination) ---
        try:
            mav.mav.command_long_send(
                ATTACK_SYSID,
                0,
                mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            print("*** MOTORS TERMINATED VIA MAV_CMD_DO_FLIGHTTERMINATION ***")
        except Exception as e:
            print(f"Failed to cut motors: {e}")

        # --- Disengage all autonomy modes immediately ---
        avoidance_active = False
        avoidance_climb_active = False
        avoidance_start_time = None

        abort_latched = False
        rtb_latched = False
        rtb_mode_sent = False
        rtb_message_printed = False

        abort_mode_sent = False
        abort_message_printed = False

        # Skip guidance for the remainder of the flight
        print("*** AUTONOMY HALTED — DRONE NOW IN PARACHUTE DESCENT ***")
        time.sleep(0.2)
        continue

    # =========================
    # PARACHUTE LOCKOUT
    # =========================

    if parachute_deployed:
        # Force display mode
        mode = "PARACHUTE_DESCENT"

        # Zero all command variables so nothing downstream changes them
        desired_heading = attack.hdg if attack.hdg is not None else 0
        commanded_speed = 0
        vN = vE = vD = 0

        # Skip all further logic including mode transitions
        if not parachute_lockout_message_printed:
            print("*** AUTONOMY LOCKOUT: PARACHUTE DEPLOYED ***")
            parachute_lockout_message_printed = True

        time.sleep(0.1)
        continue

    # =========================
    # POST-DEPLOY AVOIDANCE OVERRIDE (HARD)
    # =========================

    goto_send_command = False  # default: normal mode logic allowed

    if avoidance_active:
        elapsed = now - avoidance_start_time

        if elapsed < AVOIDANCE_DURATION_S:
            # Disable deployment logic during avoidance
            mode = "AVOIDANCE_CLIMB"

            # Hold last known heading or current heading
            if attack.hdg is not None:
                desired_heading = attack.hdg
            elif last_heading_cmd is not None:
                desired_heading = last_heading_cmd
            else:
                desired_heading = 0.0  # safe default

            commanded_speed = AVOIDANCE_SPEED
            avoidance_climb_active = True

        else:
            # End avoidance manoeuvre
            avoidance_active = False
            avoidance_climb_active = False
            avoidance_start_time = None

        # Skip ALL other guidance logic during avoidance
        goto_send_command = True

    # =========================
    # POST-DEPLOY ORBIT RESTORATION
    # =========================

    # After net deployment + avoidance, drone must return to ORBIT and stay there
    if payload_deployed and not abort_latched and not avoidance_active:
        mode = "ORBIT"

    # =========================
    # RELATIVE GEOMETRY
    # =========================

    # Relative position of target w.r.t. attack drone in NED frame
    Nt, Et = gps_to_ned(
        target.lat, target.lon,
        attack.lat, attack.lon
    )

    # Distance to home (if known)
    if home_lat is not None and home_lon is not None:
        Nh, Eh = gps_to_ned(home_lat, home_lon, attack.lat, attack.lon)
        distance_to_home = math.hypot(Nh, Eh)
    else:
        distance_to_home = None

    distance = math.hypot(Nt, Et)       # Horizontal separation [m]
    bearing  = bearing_from_ned(Nt, Et) # Bearing to target [deg]

    # =========================
    # PLOTTING COORDINATES
    # =========================
    # Use target as origin:
    # target = (0, 0)
    # attack = (-Nt, -Et)

    attack_plot_n = -Nt
    attack_plot_e = -Et

    target_plot_n = 0.0
    target_plot_e = 0.0
    
    # =========================
    # STORE TRAJECTORY
    # =========================
    attack_traj_n.append(attack_plot_n)
    attack_traj_e.append(attack_plot_e)

    target_traj_n.append(target_plot_n)
    target_traj_e.append(target_plot_e)

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
    # ABORT RECOVERY (CRITICAL FIX)
    # =========================

    if abort_recovery_active:

        # If still too close → force outward escape
        if distance < ORBIT_RADIUS * 0.9:

            mode = "ORBIT_RECOVERY"

            # Fly directly AWAY from target
            desired_heading = wrap_360 (bearing + 180.0) #bearing_from_ned(Nt, Et)
            commanded_speed = MAX_SPEED

            goto_send_command = True

        else:
            # Recovery complete
            abort_recovery_active = False

    # =========================
    # MODE LOGIC (skipped if avoidance active)
    # =========================

    if mode != "ORBIT":
        ORBIT_ERROR_INT = 0.0

    if CONTROL_ENABLED and not goto_send_command:

        if rc_attack:
            mode = "ATTACK"
            desired_heading = bearing
            commanded_speed = MAX_SPEED

        elif distance > ORBIT_RADIUS + 20:
            mode = "INTERCEPT"
            orbit_dir_locked = False

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

            if not orbit_dir_locked:
                ORBIT_DIR = choose_orbit_direction(attack, Nt, Et)
                orbit_dir_locked = True

            max_orbit_speed = math.sqrt(MAX_CENTRIPETAL_ACC * ORBIT_RADIUS)
            commanded_speed = min(commanded_speed, max_orbit_speed)

            angle = math.atan2(Et, Nt)

            # Tangential direction
            tangent = wrap_360(math.degrees(angle) + (ORBIT_DIR * 90))

            # =========================
            # INNER RADIUS ESCAPE (CRITICAL FIX)
            # =========================

            if distance < ORBIT_RADIUS * 0.7:
                # Too close → force outward escape
                desired_heading = bearing_from_ned(Nt, Et)
                commanded_speed = MAX_SPEED

            else:
                # =========================
                # PI ORBIT CONTROLLER (NEW)
                # =========================

                radius_error = distance - ORBIT_RADIUS

                # Integrate error
                ORBIT_ERROR_INT += radius_error * dt

                # Anti-windup clamp
                ORBIT_ERROR_INT = clamp(ORBIT_ERROR_INT, -100, 100)

                # PI correction
                correction = ORBIT_KP * radius_error + ORBIT_KI * ORBIT_ERROR_INT

                # Increase authority when close to target 
                if abs(radius_error) < 10:
                    correction *= 1.8 # near final radius

                # Tiny inward bias when slightly outside
                if radius_error > 0:
                    correction += 1.5

                # Ensure minimum correction authority when error exists
                if abs(radius_error) > 2:
                    min_corr = 8.0
                    if correction > 0:
                        correction = max(correction, min_corr)
                    else:
                        correction = min(correction, -min_corr)

                # Limit correction authority
                correction = clamp(correction, -60, 60)

                #desired_heading = tangent - correction
                desired_heading = wrap_360(tangent - ORBIT_DIR * correction)
            #mode = "ORBIT"

            #max_orbit_speed = math.sqrt(MAX_CENTRIPETAL_ACC * ORBIT_RADIUS)
            #commanded_speed = min(commanded_speed, max_orbit_speed)

            #angle = math.atan2(Et, Nt)

            # Tangential direction
            #tangent = wrap_360(math.degrees(angle) + (ORBIT_DIR * 90))

            # Radial correction (keeps orbit stable)
            #radius_error = distance - ORBIT_RADIUS
            #correction = clamp(radius_error * 1.5, -35, 35)

            #desired_heading = wrap_360(tangent - correction)
            #mode = "ORBIT"

            #max_orbit_speed = math.sqrt(MAX_CENTRIPETAL_ACC * ORBIT_RADIUS)
            #commanded_speed = min(commanded_speed, max_orbit_speed)

            #angle = math.atan2(Et, Nt)
            #desired_heading = wrap_360(math.degrees(angle) + (ORBIT_DIR * 90))
            #vN = -ORBIT_DIR * commanded_speed * math.sin(angle)
            #vE =  ORBIT_DIR * commanded_speed * math.cos(angle)
            #desired_heading = bearing_from_ned(vN, vE)

    #desired_heading = wrap_360(desired_heading)
    # =========================
    # TURN RATE LIMIT
    # =========================

    if not parachute_deployed and not avoidance_active and attack.hdg is not None:
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

    # Desired velocity in global NED frame (used for guidance math)
    #vN = commanded_speed * math.cos(deg2rad(desired_heading))
    #vE = commanded_speed * math.sin(deg2rad(desired_heading))

    # Convert to body frame for PX4 command
    #if attack.hdg is not None:
    heading_error = angle_diff(desired_heading, attack.hdg)
    #else:
    #    heading_error = 0.0

    vN = commanded_speed * math.cos(deg2rad(heading_error))
    vE = commanded_speed * math.sin(deg2rad(heading_error))

    # Vertical escape during post-deployment avoidance
    if avoidance_climb_active:
        vD = -AVOIDANCE_CLIMB_RATE
    else:
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
    # DEPLOYMENT GATE LOGIC
    # =========================

    # Telemetry freshness gate
    telemetry_fresh = (
        last_target_msg_time is not None and
        (now - last_target_msg_time) < TARGET_TELEMETRY_TIMEOUT
    )

    # Wings-level proxy using heading rate
    wings_level_proxy = (
        attack.hdg is not None and
        abs(angle_diff(desired_heading, attack.hdg)) < NET_MAX_TURN_RATE_DEG_S
    )

    # Line-of-sight alignment gate
    los_alignment = abs(angle_diff(desired_heading, bearing)) < 10.0

    # Minimum altitude limit gate
    altitude_safe = (
        attack.alt_rel is not None and
        target.alt_rel is not None and
        attack.alt_rel >= NET_MIN_DEPLOY_ALTITUDE and
        target.alt_rel >= NET_MIN_DEPLOY_ALTITUDE
    )

    # Cross-Track Error - Ensures actual alingment with target
    # Perpendicular error relative to line-of-sight
    cross_track_error = abs(-uE * Nt + uN * Et)

    DEPLOYMENT_AUTHORIZED = (
        attack.mode in ["GUIDED", "OFFBOARD", "AUTO"] and                      # PX4 uses OFFBOARD in SITL
        mode == "ATTACK" and
        distance <= NET_DEPLOY_DISTANCE_M and
        distance > ORBIT_RADIUS * 0.6 and

        not abort_latched and
        not rtb_latched and
        not parachute_deployed and
        
        closing_speed >= NET_MIN_CLOSING_SPEED and
        wings_level_proxy and
        los_alignment and
        NET_MIN_AIRSPEED <= commanded_speed <= NET_MAX_AIRSPEED and
        telemetry_fresh and
        altitude_safe and
        not payload_deployed
    )

    # Do not evaluate or allow deployment during post-deploy avoidance
    if (not avoidance_active) and (not abort_latched) and (not rtb_latched) and (not parachute_deployed):
        payload_manager(DEPLOYMENT_AUTHORIZED)  # One-shot payload authorisation check

    # =========================
    # SEND COMMAND
    # =========================

    if (CONTROL_ENABLED and
        #not abort_latched and
        not parachute_deployed and
        not rtb_latched and
        now - last_cmd_time > 1.0 / COMMAND_RATE_HZ):

        # Lookahead time for navigation target
        LOOKAHEAD_TIME_S = 6.0

        # Use your existing guidance heading/speed logic to define
        # a point ahead of the aircraft in the desired direction.
        lookahead_distance = max(commanded_speed * LOOKAHEAD_TIME_S, 60.0)

        cmd_north = lookahead_distance * math.cos(deg2rad(desired_heading))
        cmd_east  = lookahead_distance * math.sin(deg2rad(desired_heading))

        wp_lat, wp_lon = ned_offset_to_gps(
            attack.lat,
            attack.lon,
            cmd_north,
            cmd_east
        )

        # Hold the interceptor at the target's relative altitude when available,
        # otherwise hold its current altitude.
        if target.alt_rel is not None:
            cmd_alt_rel = target.alt_rel
        elif attack.alt_rel is not None:
            cmd_alt_rel = attack.alt_rel
        else:
            cmd_alt_rel = 120.0

        mav.mav.set_position_target_global_int_send(
            0,                                              # time_boot_ms
            ATTACK_SYSID,                                   # target system
            0,                                              # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,                             # use lat/lon/alt only
            int(wp_lat * 1e7),                              # lat_int
            int(wp_lon * 1e7),                              # lon_int
            float(cmd_alt_rel),                             # alt
            0, 0, 0,                                        # vx, vy, vz ignored
            0, 0, 0,                                        # afx, afy, afz ignored
            0,                                              # yaw ignored
            0                                               # yaw_rate ignored
        )

        last_cmd_time = now

    # =========================
    # DISPLAY
    # =========================

    # Code to display all relevant info in the terminal

    print("\033[2J\033[H", end="")

    print("ATTACK DRONE")
    print(f"  Mode:      {attack.mode}")
    print(f"  Lat:       {attack.lat:.6f}")
    print(f"  Lon:       {attack.lon:.6f}")
    print(f"  Alt (rel): {attack.alt_rel:.1f} m")
    print(f"  Alt (MSL): {attack.alt_msl:.1f} m")
    print(f"  Vn/Ve:     {attack.vx:.1f} / {attack.vy:.1f}\n")

    print("TARGET DRONE")
    print(f"  Lat:       {target.lat:.6f}")
    print(f"  Lon:       {target.lon:.6f}")
    print(f"  Alt (rel): {target.alt_rel:.1f} m")    
    print(f"  Alt (MSL): {target.alt_msl:.1f} m")
    print(f"  Speed:     {target_speed:.1f} m/s\n")

    print("RELATIVE")
    print(f"  Distance:  {distance:.1f} m")
    print(f"  Bearing:   {bearing:.1f} deg")

    if time_to_intercept is not None:
        print(f"  Time-to-Intercept: {time_to_intercept:.1f} s")
    else:
        print(f"  Time-to-Intercept: ---")

    # Override mode shown during RTB
    display_mode = mode
    if rtb_latched:
        display_mode = "RTL"

    print("\nGUIDANCE")
    print(f"  Mode:      {display_mode}")
    print(f"  Cmd Speed: {commanded_speed:.1f} m/s")
    print(f"  Heading:   {desired_heading:.1f} deg")

    print("\nRC INPUTS")
    print(f"  ATTACK:    {'ON' if rc_attack else 'OFF'}")
    print(f"  ABORT:     {'ON' if rc_abort  else 'OFF'}")
    print(f"  RTB:       {'ON' if rc_rtb else 'OFF'}")
    print(f"  PARACHUTE: {'ON' if rc_parachute else 'OFF'}")

    print("\nFAILSAFES")
    print(f"  PARACHUTE ARMED:    {'YES' if rc_parachute else 'NO'}")
    print(f"  PARACHUTE DEPLOYED: {'YES' if parachute_deployed else 'NO'}")
    print(f"  RETURN-TO-BASE:     {'ON' if rc_rtb else 'OFF'}")

    print("\nHOME")
    if distance_to_home is not None:
        print(f"  Dist to Home: {distance_to_home:.1f} m")
    else:
        print("  Dist to Home: ---")

    print("\nAVOIDANCE")
    print(f"  Avoidance: {'ACTIVE' if avoidance_active else 'OFF'}")

    sys.stdout.flush()

    # =========================
    # UPDATE LIVE PLOT
    # =========================
    update_live_plot(
        ax,
        attack_traj_n,
        attack_traj_e,
        target_traj_n,
        target_traj_e,
        attack_plot_n,
        attack_plot_e,
        target_plot_n,
        target_plot_e,
        ORBIT_RADIUS
    )   

    time.sleep(0.05)    # Loop pacing (~20Hz) to reduce CPU load
