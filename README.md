# Autonomous UAV Intercept & Orbit Guidance

A Python-based **mid-level autonomous UAV guidance system** developed to investigate target-relative intercept, pursuit and orbit behaviours using MAVLink-connected aircraft.

The system receives telemetry from an ownship UAV and a target UAV, transforms their positions into a local navigation frame, evaluates their relative geometry and generates physically constrained guidance commands for the ownship aircraft.

Development progressed through multiple versions from an initial intercept/orbit controller into a simulation-ready autonomy prototype incorporating **lead-pursuit guidance, flight-dynamics constraints, state management, human override, return-to-base behaviour, telemetry monitoring and live trajectory visualisation**.

> **Project scope:** This repository focuses on autonomous guidance, simulation and flight-control integration. Low-level aircraft stabilisation remains the responsibility of the underlying autopilot.

---

## Project Overview

The guidance architecture separates high-level autonomous decision-making from low-level flight control.

```text
                TARGET UAV
                    │
                    │ Telemetry
                    ▼
            ┌───────────────┐
            │               │
            │    MAVLink    │
            │               │
            └───────┬───────┘
                    │
                    ▼
        ┌───────────────────────┐
        │                       │
        │   AUTONOMY SOFTWARE   │
        │                       │
        │  Relative Geometry    │
        │  Intercept Guidance   │
        │  Orbit Guidance       │
        │  Flight Constraints   │
        │  State Management     │
        │                       │
        └───────────┬───────────┘
                    │
             Guidance Commands
                    │
                    ▼
            ┌───────────────┐
            │               │
            │   AUTOPILOT   │
            │               │
            │ Stabilisation │
            │ Attitude Ctrl │
            │ Flight Ctrl   │
            │               │
            └───────┬───────┘
                    │
                    ▼
                OWN UAV
```

The Python guidance layer therefore does **not** directly control individual aircraft actuators.

Instead, it provides mid-level guidance while the autopilot remains responsible for aircraft stabilisation and low-level flight control.

---

# Key Features

The project includes development and experimentation with:

- Python-based UAV autonomy
- MAVLink communication
- `pymavlink`
- Multi-vehicle telemetry
- GPS-to-local-coordinate conversion
- Target-relative geometry
- Distance and bearing calculations
- Intercept guidance
- Lead-pursuit guidance
- Autonomous orbit guidance
- Adaptive speed control
- Heading-rate limiting
- Centripetal-acceleration constraints
- PI-based orbit correction
- Human-in-the-loop control
- Manual autonomy override
- Return-to-base behaviour
- Runtime state management
- SITL simulation
- Gazebo integration
- Keyboard-based simulation controls
- Live trajectory plotting
- Terminal telemetry display

---

# Repository Structure

```text
autonomous-uav-intercept-guidance/
│
├── README.md
├── requirements.txt
├── LICENSE
│
├── src/
│   ├── autonomous_guidance.py
│   └── terminal_display.py
│
├── docs/
│   ├── guidance_mathematics.md
│   ├── architecture.md
│   └── development_history.md
│
├── legacy/
│   ├── autonomous_guidance_v1.0.py
│   ├── autonomous_guidance_v1.1.py
│   ├── autonomous_guidance_v1.2.py
│   └── autonomous_guidance_v1.3.py
│
└── media/
    ├── simulation/
    └── plots/
```

The `src` directory contains the current portfolio implementation.

Earlier versions are retained under `legacy` to document the evolution of the guidance architecture.

---

# Guidance Architecture

At each control cycle, the system processes telemetry from both aircraft and evaluates their relative state.

Conceptually:

```text
 Ownship Telemetry          Target Telemetry
        │                         │
        └──────────┬──────────────┘
                   ▼
           Validate Telemetry
                   │
                   ▼
        Convert GPS → Local N/E
                   │
                   ▼
        Calculate Relative Vector
                   │
          ┌────────┴────────┐
          ▼                 ▼
       Distance          Bearing
          │                 │
          └────────┬────────┘
                   ▼
           Guidance Logic
                   │
       ┌───────────┴───────────┐
       ▼                       ▼
   INTERCEPT                  ORBIT
       │                       │
       └───────────┬───────────┘
                   ▼
          Desired Heading
                   │
                   ▼
        Apply Physical Limits
                   │
                   ▼
          Velocity Command
                   │
                   ▼
               MAVLink
                   │
                   ▼
               Autopilot
```

---

# Coordinate Transformation

Aircraft telemetry is received as GPS latitude and longitude.

For guidance calculations, these positions are converted into a local **North-East coordinate frame** relative to a reference position.

For small operating areas, the implementation uses an Earth-radius approximation:

```text
ΔNorth ≈ ΔLatitude × R

ΔEast ≈ ΔLongitude × R × cos(latitude)
```

where:

```text
R = Earth radius
```

This produces relative displacement in metres and allows the guidance system to operate using conventional Cartesian geometry.

The detailed derivation and assumptions are documented in:

```text
docs/guidance_mathematics.md
```

---

# Relative Geometry

Once both aircraft positions are represented in the same local frame, the relative displacement can be calculated.

```text
                  Target
                     ●
                    /
                   /
                  /  Relative Distance
                 /
                /
Ownship ●──────┘
```

The relative North-East vector provides the information required to determine:

- Distance to target
- Line-of-sight bearing
- Desired intercept direction
- Orbit geometry
- Relative target movement

The bearing is calculated from the North-East components using:

```text
bearing = atan2(East, North)
```

with the resulting angle converted into a conventional navigation heading.

---

# Intercept Guidance

The initial guidance strategy directs the ownship toward the target while maintaining aircraft speed and turn-rate constraints.

A basic pure-pursuit strategy would continuously point toward the target's current location:

```text
          Target
             ●
            /
           /
          /
         /
        ●
     Ownship
```

However, when the target is moving this can cause the ownship to continually chase where the target **was**, rather than where it is heading.

The project therefore progressed toward **lead-biased pursuit guidance**.

---

# Lead-Pursuit Guidance

Target velocity is incorporated into the desired guidance vector.

Conceptually:

```text
Target Current Position
          ●──────────────► Target Velocity
           \
            \
             \
              X  Lead Point
             /
            /
           ●
       Ownship
```

A configurable lead gain determines how aggressively target velocity influences the intercept direction.

The implementation uses:

```python
LEAD_GAIN = 0.5
```

This creates a compromise between:

```text
Pure Pursuit ◄──────────────► Aggressive Lead
                  ▲
                  │
              LEAD_GAIN
```

allowing the ownship to bias its trajectory toward the target's predicted direction of movement rather than simply pointing at its instantaneous position.

---

# Adaptive Speed Control

The ownship command speed is adjusted relative to target motion while remaining within defined limits.

Representative configuration values include:

```python
MIN_SPEED = 12.0
MAX_SPEED = 25.0
SPEED_MARGIN = 1.3
```

The speed margin allows the guidance system to command a speed advantage while the minimum and maximum limits constrain the requested aircraft speed.

Conceptually:

```text
Target Speed
     │
     ▼
Apply Speed Margin
     │
     ▼
Candidate Command Speed
     │
     ▼
Clamp Between
MIN_SPEED and MAX_SPEED
     │
     ▼
Commanded Airspeed
```

---

# Flight-Dynamics Constraints

One of the later developments was the introduction of explicit physical constraints.

A fixed-wing aircraft cannot follow an arbitrarily tight circular trajectory at arbitrary speed.

Centripetal acceleration is related to velocity and radius by:

```text
        v²
a = ─────────
        r
```

Therefore:

```text
v = √(a × r)
```

The implementation defines a maximum allowable lateral acceleration:

```python
MAX_CENTRIPETAL_ACC = 5.0
```

which can be used to constrain commanded speed for a given turn or orbit radius.

This prevents the guidance layer from requesting trajectories that disregard the basic physical limitations of the aircraft model.

---

# Heading-Rate Limiting

Desired heading changes are also constrained.

The implementation includes:

```python
MAX_TURN_RATE_DEG_S = 25.0
```

Rather than allowing an instantaneous heading change:

```text
Current Heading
      │
      │          ┌──── Desired Heading
      │          │
──────┴──────────┴────────────► time
```

the commanded heading progresses toward the target heading at a bounded rate:

```text
Heading
   │
   │              ───── Desired
   │           /
   │        /
   │     /
   │  /
   └──────────────────────────► time
```

This provides more physically realistic guidance commands.

---

# Autonomous Orbit Guidance

Once the appropriate target-relative geometry is achieved, the system can generate guidance intended to maintain an orbit around the target.

The desired orbit radius is configurable:

```python
ORBIT_RADIUS = 150.0
```

with direction represented as:

```python
ORBIT_DIR = 1
```

for the configured orbit direction.

Conceptually:

```text
                  Ownship
                     ●
                 .       .
              .             .
            .                 .
           .         ●         .
            .      Target     .
              .             .
                 .       .
```

Orbit guidance combines two requirements:

1. **Tangential motion** around the target.
2. **Radial correction** toward the desired orbit radius.

---

# Orbit Error Correction

The later implementation introduces proportional and integral orbit correction terms:

```python
ORBIT_KP = 0.7
ORBIT_KI = 0.06
```

The radial error can be considered as:

```text
Orbit Error = Current Radius - Desired Radius
```

Conceptually:

```text
                Desired Orbit
             .----------------.
          .                      .
        .                          .
       .             ●              .
        .          Target          .
          .                      .
             '----------------'

                      ● Ownship
                      │
                      │ Radial Error
                      ▼
```

The controller can use this error to bias the guidance vector back toward the desired orbit.

The integral term allows persistent radial error to influence the correction over time.

---

# Guidance Modes

The system evolved around several distinct guidance and safety states.

At a high level:

```text
                  AUTONOMY
                     │
             ┌───────┴────────┐
             ▼                ▼
         INTERCEPT           ORBIT
             │                │
             └───────┬────────┘
                     │
                     ▼
                OVERRIDES
                     │
          ┌──────────┼──────────┐
          ▼          ▼          ▼
        ABORT       RTB       MANUAL
```

This separation allows autonomous behaviour to operate while preserving higher-priority pilot or safety commands.

---

# Human-in-the-Loop Control

The autonomy architecture deliberately retains human supervisory control.

RC inputs can be used to influence or override autonomous behaviour.

The later implementation contains dedicated channels for functions including:

```python
RC_ATTACK_CH = 6
RC_ABORT_CH = 7
RC_PARACHUTE_CH = 8
RC_RTB_CH = 9
```

For the public portfolio, the important architectural concept is the **priority of supervisory and safety inputs over autonomous guidance**.

```text
        Autonomous Guidance
                 │
                 ▼
          Proposed Command
                 │
                 ▼
       Safety / Pilot Checks
                 │
        ┌────────┴────────┐
        ▼                 ▼
    Continue          Override
    Autonomy              │
                          ▼
                    Pilot / RTB
```

---

# Abort Behaviour

The system includes an abort mechanism that can suppress autonomous guidance and return control to a safer autopilot/pilot-controlled state.

The implementation uses latch and edge-detection logic to prevent repeated mode commands.

Conceptually:

```text
Autonomy Active
      │
      ▼
ABORT Requested?
   /       \
 No         Yes
 │           │
 ▼           ▼
Continue   Latch Abort
Autonomy       │
               ▼
       Stop Autonomous
          Guidance
               │
               ▼
        Autopilot Mode
```

This is an important architectural feature because autonomous control is treated as **subordinate to supervisory safety control**.

---

# Return-to-Base

Later development introduced explicit return-to-base state handling.

The system stores home-position information received from the autopilot and can monitor distance to home.

Runtime state includes:

```text
Home Latitude
Home Longitude
Home Altitude
```

alongside RTB latch and edge-detection state.

The terminal display can therefore show whether return-to-base behaviour is active and the current distance from the aircraft to its home position.

---

# MAVLink Integration

Communication with the simulated or physical autopilot is performed using:

```python
from pymavlink import mavutil
```

Later simulation development uses separate MAVLink connections for the two vehicles.

Conceptually:

```text
       Ownship SITL                  Target SITL
            │                            │
            │ MAVLink                    │ MAVLink
            ▼                            ▼
     UDP Connection 1              UDP Connection 2
            │                            │
            └────────────┬───────────────┘
                         ▼
                  Python Guidance
```

Both MAVLink streams are drained during each control cycle so telemetry from one vehicle does not prevent the other vehicle from being processed.

---

# Simulation Environment

The later version was developed for software-in-the-loop testing using UDP MAVLink connections.

Example configuration:

```python
DEVICE = "udp:127.0.0.1:14541"
BAUD = None
```

This allows the autonomy software to communicate with simulated vehicles without requiring physical flight hardware.

The development workflow can therefore be represented as:

```text
             Python Guidance
                   │
                   ▼
                MAVLink
                   │
        ┌──────────┴──────────┐
        ▼                     ▼
   Ownship SITL          Target SITL
        │                     │
        └──────────┬──────────┘
                   ▼
             Gazebo / SITL
                   │
                   ▼
            Observe Behaviour
                   │
                   ▼
          Tune / Debug Guidance
```

This enables guidance logic to be tested and visualised before any consideration of physical flight testing.

---

# Software Simulation Controls

The simulation version includes keyboard-based software commands for exercising system states without requiring physical RC hardware.

Supported development commands include:

```text
attack
attack off

abort
abort off

rtb
rtb off

parachute
parachute off

status
help
```

These controls were introduced for **SITL development and testing**, allowing state transitions to be exercised directly from the terminal.

The keyboard listener runs separately from the main guidance loop so interactive input does not block telemetry processing.

---

# Multi-Vehicle Telemetry Handling

The later implementation maintains separate state objects for:

```text
Ownship / Attack UAV
Target UAV
```

Each stores the telemetry required by the guidance algorithms, including information such as:

```text
Latitude
Longitude
North velocity
East velocity
Heading
Flight mode
```

The guidance layer therefore operates on structured vehicle state rather than directly coupling every calculation to individual MAVLink messages.

This separation improves readability and provides a clearer interface between:

```text
MAVLink Reception
       │
       ▼
Vehicle State
       │
       ▼
Guidance Logic
```

---

# Live Trajectory Visualisation

Later development introduced live plotting using:

```python
matplotlib.pyplot
```

Trajectory history is retained for both aircraft and displayed during simulation.

Conceptually:

```text
North
  ▲
  │               T T T T
  │            T
  │         T
  │
  │      A A
  │    A
  │  A
  │
  └────────────────────────► East

A = Ownship trajectory
T = Target trajectory
```

The desired orbit radius can also be visualised around the target.

This makes it much easier to inspect:

- Intercept behaviour
- Pursuit geometry
- Orbit acquisition
- Orbit stability
- Relative trajectories
- Guidance tuning

---

# Terminal Telemetry Display

A separate terminal-display implementation was developed to provide a clearer real-time view of system state.

The display presents information including:

```text
OWN UAV
  Position
  Velocity
  Heading
  Mode

TARGET UAV
  Position
  Velocity
  Heading

RELATIVE GEOMETRY
  Distance
  Bearing
  Time-to-intercept

GUIDANCE
  Current mode
  Commanded speed
  Desired heading

RC / SUPERVISORY INPUTS
  Autonomous mode state
  Abort
  RTB
  Additional safety states

HOME
  Distance to home

SYSTEM STATE
  Avoidance / failsafe information
```

Separating this information into a structured display makes the autonomy system easier to monitor during simulation and debugging.

---

# Development History

The software was developed iteratively rather than as a single final script.

```text
VERSION 1.0
    │
    ├── Baseline MAVLink integration
    ├── Multi-UAV telemetry
    ├── GPS → local coordinates
    ├── Relative geometry
    ├── Intercept guidance
    ├── Orbit behaviour
    ├── Adaptive speed
    └── Basic supervisory controls
    │
    ▼
VERSION 1.1
    │
    ├── Lead-pursuit guidance
    ├── Centripetal acceleration constraint
    └── Improved physical realism
    │
    ▼
VERSION 1.2 / 1.3
    │
    ├── Expanded autonomous sequencing
    ├── Additional runtime state
    ├── Safety / override behaviour
    └── Return-to-base development
    │
    ▼
VERSION 1.4
    │
    ├── SITL / Gazebo integration
    ├── Separate MAVLink vehicle streams
    ├── Keyboard simulation controls
    ├── Live trajectory plotting
    ├── Orbit PI terms
    └── Expanded telemetry monitoring
```

Earlier implementations are retained in:

```text
legacy/
```

to show the progression of the software architecture.

---

# Guidance Mathematics

The mathematical reasoning behind the guidance system is documented separately:

```text
docs/guidance_mathematics.md
```

Topics include:

- GPS-to-local coordinate conversion
- Relative-position vectors
- Distance calculation
- Bearing calculation
- Target-relative geometry
- Adaptive speed
- Lead-pursuit guidance
- Orbit-vector construction
- Radial orbit correction
- Centripetal acceleration
- Heading-rate constraints

Keeping this material separate from the main README allows the repository overview to remain readable while preserving the engineering reasoning behind the implementation.

---

# Development Philosophy

The system follows a layered autonomy approach:

```text
MISSION / BEHAVIOUR
        │
        ▼
INTERCEPT / ORBIT LOGIC
        │
        ▼
GUIDANCE
        │
        ▼
PHYSICAL CONSTRAINTS
        │
        ▼
MAVLink COMMANDS
        │
        ▼
AUTOPILOT
        │
        ▼
LOW-LEVEL FLIGHT CONTROL
```

This separation is important.

The Python autonomy layer determines **where and how the aircraft should move**, while the autopilot determines the low-level control actions required to achieve that motion.

---

# Safety Architecture

The project was developed with supervisory control above the autonomous guidance layer.

The intended priority is:

```text
             HIGHEST PRIORITY

             Pilot / Safety
                  │
                  ▼
            Abort / RTB
                  │
                  ▼
         Autonomy Permission
                  │
                  ▼
        Guidance State Machine
                  │
                  ▼
          Guidance Commands

             LOWEST PRIORITY
```

Autonomy is therefore treated as a conditional capability rather than having unrestricted authority over the aircraft.

---

# Dependencies

The main Python implementation uses:

```text
Python 3
pymavlink
matplotlib
```

A typical installation is:

```bash
pip install pymavlink matplotlib
```

The exact simulation environment additionally depends on the autopilot/SITL configuration used for testing.

---

# Running the Software

The current implementation is intended primarily as a **development and simulation prototype**.

Before running, the MAVLink endpoints and system IDs must correspond to the configured SITL environment.

The source contains configuration for separate ownship and target MAVLink streams.

Start the required SITL vehicles before launching:

```bash
python src/autonomous_guidance.py
```

The software then waits for MAVLink heartbeats before beginning normal telemetry processing.

---

# Known Limitations

The project is an evolving autonomy prototype rather than a flight-qualified control system.

Current limitations include:

- Reliance on externally supplied target telemetry
- Simplified local Earth-coordinate approximation
- Primarily planar relative guidance
- Configurable rather than adaptive pursuit gain
- Simplified aircraft dynamic constraints
- No formal trajectory optimisation
- No probabilistic target-state estimator
- No sensor-fusion implementation
- No formal verification of guidance behaviour
- No environmental qualification
- No operational flight certification

Simulation behaviour should therefore not be interpreted as evidence of operational flight readiness.

Potential Future Development

Possible extensions to the guidance architecture include:

Kalman-filter target-state estimation

Sensor fusion

Noisy/lost telemetry handling

Target-state prediction

More advanced pursuit guidance

Model-predictive guidance

Dynamic flight-envelope modelling

Wind compensation

Altitude-aware 3D guidance

Improved orbit acquisition

Automated parameter tuning

Structured event logging

Replayable simulation logs

Automated SITL test scenarios

Unit and integration testing

ROS 2 integration

A more advanced architecture could therefore evolve toward:

             Sensors / Telemetry
                     │
                     ▼
               Sensor Fusion
                     │
                     ▼
              State Estimation
                     │
                     ▼
             Target Prediction
                     │
                     ▼
              Path / Guidance
                     │
                     ▼
            Flight Constraints
                     │
                     ▼
                 Autopilot

Skills Demonstrated

This project demonstrates experience across several areas of autonomous-systems software development.

Python

Modular control logic

Runtime state management

Mathematical computation

Threading

Interactive terminal controls

Live data visualisation

Robotics & Autonomy

Autonomous guidance

Relative navigation

Pursuit algorithms

Orbit guidance

State-based behaviour

Human supervisory control

Mathematics

Coordinate transformations

Vector geometry

Navigation bearings

Relative motion

Pursuit geometry

Circular-motion constraints

Rate limiting

UAV Systems

MAVLink

pymavlink

Autopilot integration

Multi-vehicle telemetry

SITL

Flight modes

Home-position handling

Software Engineering

Iterative development

Version progression

Separation of telemetry and guidance

Runtime-state containers

Simulation-based testing

Diagnostic interfaces

Safety-oriented override logic

Portfolio Context

This project represents a significant progression from my earlier autonomous mobile-robot projects.

Embedded Systems
       │
       ▼
Sensor-Based Robotics
       │
       ▼
Reactive Navigation
       │
       ▼
Autonomous Mobile Robots
       │
       ▼
Coordinate Geometry
       │
       ▼
Target-Relative Guidance
       │
       ▼
Multi-Vehicle Autonomy
       │
       ▼
MAVLink / Autopilot Integration
       │
       ▼
Simulation-Based UAV Guidance

Rather than controlling individual motors directly, this project operates at a higher level of abstraction:

Earlier Projects

Sensor → Controller → Motor

              ↓

This Project

Telemetry
    ↓
State
    ↓
Geometry
    ↓
Guidance
    ↓
Physical Constraints
    ↓
MAVLink
    ↓
Autopilot
    ↓
Aircraft

The project therefore demonstrates the progression from basic embedded robotic control toward multi-vehicle autonomous guidance, simulation and flight-control integration.

Disclaimer

This repository is presented as an engineering portfolio and simulation/research project demonstrating autonomous guidance software development.

It is not presented as flight-qualified, safety-certified or operationally deployable aircraft-control software.
