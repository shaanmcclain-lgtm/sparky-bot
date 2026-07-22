# SparkyBot Waypoint Follower: Modular Control System Design

**Status**: Design Phase (No hardware code yet)  
**Target Robot**: SparkyBotMini differential-drive platform  
**Last Updated**: 2026-07-22

---

## Executive Summary

This document specifies a modular software architecture for autonomous waypoint-following on a differential-drive robot using wheel encoders and an IMU. The design prioritizes **modularity, testability, and safety** following the AI Programming Guide for Robotics best practices.

**Key principle**: Build small, testable modules with clear interfaces. Test progressively from simulation → unit tests → safe hardware tests → full mission.

---

## 1. Robot Behavior Specification

### 1.1 Robot Brief

| Aspect | Details |
|--------|---------|
| **Type** | Differential-drive mobile platform (SparkyBotMini) |
| **Drive System** | 4 DC motors (2 left, 2 right) with wheel encoders |
| **Sensors** | Wheel encoders, 9-DOF IMU (accel, gyro, mag), battery voltage |
| **Controller** | Linux host (Raspberry Pi / x86) over serial link |
| **Comms** | UART serial at 115200 baud |
| **Compute Constraint** | ~50 Hz control loop, <50 ms latency tolerance |

### 1.2 Task

**Primary Task**: Autonomously navigate from start position through a sequence of GPS-free waypoints (x, y, θ) using dead-reckoning (encoders + IMU).

**Sequence**:
1. Load waypoint list from file or remote command
2. Start mission: drive to waypoint 1
3. Update odometry from encoder + IMU at 50 Hz
4. Execute motion control: steer toward waypoint
5. Detect waypoint reached (pos tolerance ±10 cm, heading ±10°)
6. Advance to next waypoint
7. Repeat until all waypoints visited
8. Return to ready/idle state

### 1.3 Inputs

| Input | Format | Rate | Units | Handling |
|-------|--------|------|-------|----------|
| **Encoder (L, R)** | Tick deltas | 50 Hz | counts | Accumulated via serial |
| **IMU yaw rate** | Raw gyro output | 50 Hz | deg/s → rad/s | Low-pass filtered |
| **IMU heading** | Magnetometer-fused | 20 Hz | rad, [-π, π] | Drift correction |
| **Battery voltage** | 1 byte ADC | 10 Hz | 0–255 → volts | Threshold warning |
| **Command (user)** | Mission start/stop/pause | On-demand | enum | Validated state transition |

**Stale data handling**: If any sensor missing >200 ms, flag as "stale" and degrade gracefully.

### 1.4 Outputs

| Output | Format | Rate | Range | Limits |
|--------|--------|------|-------|--------|
| **Motor command (L, R)** | Signed PWM | 50 Hz | [-100, 100] | Ramped; no step jumps |
| **Status (state machine)** | Enum + telemetry | 5 Hz | IDLE / READY / RUNNING / PAUSED / FAULT | JSON log |
| **Odometry pose** | (x, y, θ) | 50 Hz | meters, radians | ±drift over time |
| **Mission progress** | Waypoint index + % complete | 1 Hz | 0–100% | User feedback |

### 1.5 Constraints

- **Max linear speed**: 0.5 m/s (tuned for stable odometry)
- **Max angular speed**: 1.57 rad/s (≈90°/s; full rotation in ~4 sec)
- **Acceleration limit**: 0.2 m/s² ramp to prevent wheel slip
- **Turning radius**: Minimum ~0.15 m (differential drive geometry)
- **Battery**: Nominal 12V; minimum operational 10.5V; warning at 11.0V
- **Field size**: Indoor arena, max ~20 m × 20 m per mission
- **Compute**: Single-threaded 50 Hz loop; no blocking I/O
- **No GPS / external localization**: Pure dead-reckoning (odometry + IMU)

### 1.6 Safe State

**At any moment, the robot must be able to achieve this state in <500 ms**:

1. All motor commands → 0 (coast or brake)
2. State machine → **FAULT** (no auto-recovery)
3. Write error log with timestamp and last valid odometry
4. Operator must manually verify and re-enter **READY** state
5. Wireless emergency stop (if available) has priority over all logic

---

## 2. Modular Architecture

### 2.1 Layer Overview

```
┌─────────────────────────────────────────────────────────┐
│ Mission / Goal Layer                                     │
│  - Waypoint list management                             │
│  - Progress tracking                                    │
│  - Completion detection                                 │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ State Machine / Decision Layer                          │
│  - IDLE, READY, RUNNING, PAUSED, FAULT                 │
│  - Transition logic & safety checks                     │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Control Layer                                            │
│  - Motion controller (PD loops for heading, speed)      │
│  - Waypoint tracking (goal-seeking logic)               │
│  - Output saturation & rate limiting                    │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Estimation Layer (Sensor Fusion)                        │
│  - Odometry estimator (encoder + IMU)                   │
│  - State covariance tracking                            │
│  - Stale data detection                                 │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Hardware Interface Layer                                 │
│  - Encoder reader (via serial protocol)                 │
│  - IMU reader (via serial protocol)                     │
│  - Motor driver command builder                         │
│  - Battery monitor                                      │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Safety Supervisor (Cross-cutting)                        │
│  - Watchdog timer on loop rate                         │
│  - Output clamping (all layers)                         │
│  - Timeout enforcement                                  │
│  - Emergency stop handler                               │
└─────────────────────────────────────────────────────────┘
```

### 2.2 Module Responsibilities

| Module | Input | Output | Rate | Failure Mode |
|--------|-------|--------|------|--------------|
| **Encoder Reader** | Serial stream | (Δticks_L, Δticks_R, timestamp) | 50 Hz | Stale >200ms → flag stale |
| **IMU Reader** | Serial stream | (yaw_rate_rad/s, heading_rad, timestamp) | 50 Hz | Stale >200ms → flag stale |
| **Odometry Estimator** | Encoder + IMU deltas | Pose2D (x, y, θ), Velocity2D, covariance | 50 Hz | Drift logged; exceeds threshold → warn |
| **Waypoint Controller** | Waypoint list, current pose | Next goal Pose2D | On advance | Invalid list → reject, stay IDLE |
| **Motion Controller** | Goal pose, current pose, dt | Motor command (L_pwm, R_pwm) | 50 Hz | Goal unreachable >60s → FAULT |
| **Motor Driver** | Motor command | UART protocol packet | 50 Hz | Command not echoed → retry, then FAULT |
| **State Machine** | User input, subsystem health | Active state, mode flags | Event-driven | Timeout or fault → FAULT |
| **Safety Supervisor** | All subsystem outputs | Clamp/disable signals | 50 Hz | Enforce hard limits, emergency stop |

---

## 3. Interfaces & Data Types

### 3.1 Core Data Structures

```python
# All distances in meters, angles in radians, time in seconds

@dataclass
class Pose2D:
    """Robot pose in global frame"""
    x: float          # meters
    y: float          # meters
    theta: float      # radians, [-π, π]
    timestamp: float  # seconds since mission start

@dataclass
class Velocity2D:
    """Robot velocity"""
    linear: float     # m/s (forward)
    angular: float    # rad/s (CCW positive)
    timestamp: float  # seconds

@dataclass
class MotorCommand:
    """Command sent to motor driver"""
    left_speed: float   # m/s or PWM [-1, 1]
    right_speed: float  # m/s or PWM [-1, 1]
    timestamp: float    # seconds

@dataclass
class EncoderReading:
    """Encoder sample"""
    left_ticks: int     # absolute count
    right_ticks: int    # absolute count
    delta_left: int     # ticks since last read
    delta_right: int    # ticks since last read
    timestamp: float    # seconds

@dataclass
class IMUReading:
    """IMU sample (already fused by robot firmware)"""
    yaw_rate: float     # rad/s
    heading: float      # rad, [-π, π] from magnetometer
    ax: float           # m/s² (if needed)
    ay: float           # m/s²
    az: float           # m/s²
    timestamp: float    # seconds

@dataclass
class SensorHealth:
    """Freshness and validity of each sensor"""
    encoders_ok: bool
    encoders_age_ms: int
    imu_ok: bool
    imu_age_ms: int
    battery_voltage: float  # volts
    battery_ok: bool

@dataclass
class SystemHealth:
    """Overall system status"""
    state: str  # "IDLE" | "READY" | "RUNNING" | "PAUSED" | "FAULT"
    sensors: SensorHealth
    odometry_confidence: float  # 0.0 (lost) to 1.0 (high)
    error_log: List[str]
    last_error: str or None

@dataclass
class WaypointList:
    """Mission waypoints"""
    waypoints: List[Pose2D]  # in order
    metadata: dict  # {"name": str, "created": timestamp, ...}
```

### 3.2 Module Interfaces (Public Methods)

#### **A. Encoder Reader Interface**

```python
class EncoderReader:
    """Reads wheel encoders from serial"""
    
    def get_delta_ticks(self) -> Tuple[int, int]:
        """Return (delta_left, delta_right) since last call"""
        # Return (-1, -1) if stale (>200 ms since last sensor packet)
    
    def reset_counters(self) -> None:
        """Zero the absolute tick counts"""
    
    def get_ticks_per_meter(self) -> Tuple[float, float]:
        """Return (left_ticks/m, right_ticks/m) from calibration"""
    
    def is_healthy(self) -> bool:
        """Return False if data is stale or invalid"""
```

#### **B. IMU Reader Interface**

```python
class IMUReader:
    """Reads IMU from serial"""
    
    def get_yaw_rate(self) -> float:
        """Return yaw rate in rad/s; 0 if stale"""
    
    def get_heading(self) -> float:
        """Return heading in rad [-π, π]; NaN if uncalibrated"""
    
    def get_raw_imu(self) -> IMUReading:
        """Return raw IMU sample with timestamp"""
    
    def is_healthy(self) -> bool:
        """Return False if stale (>200 ms) or magnetometer not calibrated"""
```

#### **C. Odometry Estimator Interface**

```python
class OdometryEstimator:
    """Fuses encoders and IMU into pose estimate"""
    
    def __init__(self, wheel_radius: float, wheelbase: float):
        """
        Args:
            wheel_radius: meters
            wheelbase: distance between wheel centers (meters)
        """
    
    def update(self, encoder: EncoderReading, imu: IMUReading, dt: float) -> None:
        """Integrate new sensor data into pose estimate"""
    
    def get_pose(self) -> Pose2D:
        """Return current (x, y, theta) estimate"""
    
    def get_velocity(self) -> Velocity2D:
        """Return (linear, angular) velocity estimate"""
    
    def get_covariance(self) -> List[List[float]]:
        """Return 3x3 uncertainty matrix diag [σ_x², σ_y², σ_θ²]"""
    
    def reset_to_pose(self, pose: Pose2D) -> None:
        """Reset estimate to known pose (e.g., at mission start)"""
```

#### **D. Motion Controller Interface**

```python
class MotionController:
    """Tracks goal pose using PD control"""
    
    def __init__(self, kp_linear: float, kp_angular: float):
        """
        Args:
            kp_linear: proportional gain for linear speed (s^-1)
            kp_angular: proportional gain for angular speed (s^-1)
        """
    
    def set_goal(self, goal: Pose2D) -> None:
        """Set target pose"""
    
    def set_speed_limits(self, max_linear: float, max_angular: float) -> None:
        """Set saturation limits in m/s and rad/s"""
    
    def update(self, current_pose: Pose2D, dt: float) -> MotorCommand:
        """
        Compute motor command to track goal.
        
        Returns:
            MotorCommand with left/right speeds (m/s).
            If goal unreachable >60s, raises TimeoutError.
        """
    
    def is_goal_reached(self, current_pose: Pose2D, 
                        pos_tol: float = 0.1, 
                        ang_tol: float = 0.174) -> bool:
        """
        Check if goal is reached within tolerances.
        
        Args:
            pos_tol: position tolerance (meters, default 10 cm)
            ang_tol: angle tolerance (radians, default ~10°)
        """
```

#### **E. Waypoint Controller Interface**

```python
class WaypointController:
    """Manages waypoint sequence"""
    
    def load_waypoints(self, wp_list: WaypointList) -> bool:
        """Load waypoint sequence. Return False if invalid."""
    
    def get_current_goal(self) -> Pose2D or None:
        """Return current target waypoint; None if mission complete"""
    
    def advance_to_next_waypoint(self) -> bool:
        """Move to next waypoint. Return False if already at end."""
    
    def get_progress(self) -> Tuple[int, int]:
        """Return (current_index, total_count)"""
    
    def reset(self) -> None:
        """Reset to first waypoint"""
    
    def is_mission_complete(self) -> bool:
        """Return True if all waypoints visited"""
```

#### **F. Motor Driver Interface**

```python
class MotorDriver:
    """Sends motor commands to robot firmware"""
    
    def set_wheel_speeds(self, left_m_per_s: float, right_m_per_s: float) -> bool:
        """
        Command wheel speeds. Clamps to [-0.5, 0.5] m/s.
        
        Returns:
            True if command sent successfully, False if serial error.
        """
    
    def emergency_stop(self) -> bool:
        """Send hard stop command"""
    
    def get_battery_voltage(self) -> float:
        """Return last known voltage in volts"""
    
    def is_ready(self) -> bool:
        """Return False if serial link down or no handshake"""
```

#### **G. State Machine Interface**

```python
class StateMachine:
    """Manages mission state and transitions"""
    
    def __init__(self, config: dict):
        """Load safety limits and state parameters"""
    
    def handle_user_command(self, command: str) -> bool:
        """
        Process user command: "start", "stop", "pause", "resume", "reset".
        Return True if valid transition, False otherwise.
        """
    
    def update_health(self, health: SystemHealth) -> None:
        """Feed subsystem health into state machine"""
    
    def get_current_state(self) -> str:
        """Return current state: IDLE, READY, RUNNING, PAUSED, FAULT"""
    
    def get_active_mode(self) -> bool:
        """Return True if state permits motion (RUNNING or PAUSED)"""
    
    def is_faulted(self) -> bool:
        """Return True if state is FAULT"""
```

#### **H. Safety Supervisor Interface**

```python
class SafetySupervisor:
    """Enforces hard limits and emergency stop"""
    
    def clamp_motor_command(self, cmd: MotorCommand, 
                            max_linear: float, 
                            max_angular: float) -> MotorCommand:
        """Enforce speed limits and rate limits (slew rate)"""
    
    def request_emergency_stop(self) -> None:
        """Trigger immediate safe shutdown"""
    
    def check_watchdog(self, loop_time_ms: int) -> bool:
        """Return False if loop exceeded time budget (e.g., >20 ms for 50 Hz)"""
    
    def is_safe_to_command_motors(self) -> bool:
        """Return False if system is in FAULT or emergency stop active"""
```

---

## 4. Units & Coordinate Frames

### 4.1 Unit Table

| Quantity | Unit | Notes |
|----------|------|-------|
| Position (x, y) | meters (m) | Global frame, origin at robot start |
| Heading (θ) | radians (rad) | Range [-π, π], 0 = +x-axis, π/2 = +y-axis (CCW) |
| Linear velocity (v) | m/s | Positive = forward along robot x-axis |
| Angular velocity (ω) | rad/s | Positive = CCW (right-hand rule around z-axis) |
| Wheel radius (r) | meters (m) | From wheel center to contact point |
| Wheelbase (L) | meters (m) | Distance between left and right wheel centers |
| Encoder resolution | ticks/revolution | From calibration; multiply by 2πr to get m/tick |
| Motor command | PWM duty cycle | Range [-1, 1], -1 = full reverse, 0 = stop, 1 = full forward |
| Acceleration | m/s² or rad/s² | Rate of change of velocity |
| Timestamp | seconds (s) | Since mission start (t=0 at first odometry update) |
| Battery voltage | volts (V) | Nominal 12V, warning at 11.0V, min 10.5V |

### 4.2 Coordinate Frames

- **Global frame**: Fixed to ground; robot position and waypoints expressed here
- **Robot frame**: Fixed to robot body; x-axis forward, y-axis left (standard differential drive)
- **Transformation**: 
  ```
  [x_global]     [cos(θ) -sin(θ)] [x_robot]   [x_start]
  [y_global]  =  [sin(θ)  cos(θ)] [y_robot] + [y_start]
  ```

### 4.3 Angle Wrapping

Always wrap heading errors to [-π, π]:
```
angle_error = atan2(sin(θ_goal - θ_current), cos(θ_goal - θ_current))
```

---

## 5. Failure Cases & Recovery

### 5.1 Failure Matrix

| Failure | Root Cause | Detection | Recovery | Severity |
|---------|-----------|-----------|----------|----------|
| **Encoder stale** | Serial lag, sensor unplugged | No ticks for >200 ms | Flag `encoders_ok=False`, fall back to IMU-only | Medium |
| **IMU stale** | Calibration lost, serial lag | No heading update for >200 ms | Flag `imu_ok=False`, use encoder heading estimate | Medium |
| **Motor stalled** | Obstacle, friction, dead battery | Motor command sent, encoder unchanged >100 ms | Reduce speed 20% and retry; if persists, → FAULT | High |
| **Battery critical** | Discharge > expected | Voltage < 10.5 V | Reduce max speed to 0.3 m/s, emit warning; if <10V → stop | High |
| **Goal unreachable** | Obstacle blocking path, tuning off | Distance to goal doesn't decrease >60 s | Log obstacle, emit warning, → FAULT (wait for operator) | Medium |
| **Loop timeout** | CPU overload, blocking call | Iteration time >20 ms (for 50 Hz target) | Skip iteration, log warning, check CPU load | Medium |
| **Serial link lost** | USB disconnect, baud mismatch | No packets for >500 ms | Attempt reconnect 3×; if fails → FAULT | High |
| **Odometry divergence** | Wheel slip, encoder error, IMU drift | Pose covariance exceeds threshold (>1 m²) | Log warning, mark confidence < 0.5, continue but operator aware | Low |
| **State transition invalid** | User command in wrong state | Attempted transition FAULT → RUNNING | Reject command, log invalid transition, stay in current state | Low |
| **NaN/Inf in math** | Divide-by-zero, numeric underflow | State contains NaN values | Clamp to last valid value, log error, mark subsystem suspect | High |

### 5.2 Recovery Procedures

1. **Automatic (no human required)**:
   - Stale encoder → use IMU-only odometry
   - Stale IMU → use encoder-only odometry
   - Motor command not echoed → retry up to 3 times with backoff

2. **Human-required (operator intervention needed)**:
   - Motor stall → operator adjusts waypoints or clears obstacle
   - Battery critical → operator docks/charges
   - Goal unreachable → operator inspects environment and resumes
   - Serial link lost → operator reconnects device

3. **No auto-recovery from FAULT state**:
   - Fault must be explicitly cleared by operator command `reset` after manual inspection
   - Safety-first: assume worst case until human confirms safe

---

## 6. Test Plan

### 6.1 Unit Tests (Per Module, No Hardware)

**Test each module in isolation using mock dependencies.**

#### **EncoderReader Tests**
- ✓ Parse valid tick packets → correct delta values
- ✓ Stale detection: no packet >200 ms → `is_healthy()` returns False
- ✓ Tick counter rollover: handle 32-bit wrap-around
- ✓ Invalid packet format → skip, no crash
- ✓ Concurrent read while new data arrives → thread-safe

#### **IMUReader Tests**
- ✓ Parse yaw rate and heading from valid packet
- ✓ Stale detection >200 ms → `is_healthy()` returns False
- ✓ Uncalibrated magnetometer → heading = NaN, `is_healthy()` returns False
- ✓ Angle wrapping: heading > π wraps to negative range

#### **OdometryEstimator Tests**
- ✓ **Straight line forward**: 1 m traveled, final pose = (1, 0, 0)
- ✓ **Pure rotation**: 90° turn in place, final pose = (0, 0, π/2)
- ✓ **Arc**: 0.5 m forward + 45° left turn → pose within 5 cm and 5° of expected
- ✓ **Encoder loss**: continue with IMU-only estimate
- ✓ **IMU drift**: log pose covariance growth over 5 min (expect <1 m drift indoors)
- ✓ **Reset to known pose**: `reset_to_pose()` → next update correct
- ✓ Covariance matrix: positive definite, grows monotonically

#### **MotionController Tests**
- ✓ **Goal straight ahead**: current (0,0,0), goal (1,0,0) → output forward speed >0
- ✓ **Goal to the left**: current (0,0,0), goal (0,1,0) → output CCW angular speed >0
- ✓ **Goal reached**: current within tol of goal → output speeds → 0
- ✓ **Gain tuning**: vary Kp → observe convergence time and overshoot
- ✓ **Speed saturation**: large error doesn't exceed max_linear or max_angular
- ✓ **Rate limiting**: successive commands don't exceed acceleration limit (e.g., 0.2 m/s²)
- ✓ **Timeout**: goal not reached >60 s → raise TimeoutError

#### **WaypointController Tests**
- ✓ Load valid 3-waypoint list → stored correctly
- ✓ Invalid waypoint (NaN, out of range) → reject load, return False
- ✓ Empty waypoint list → reject, return False
- ✓ `get_current_goal()` → returns first waypoint
- ✓ `advance_to_next_waypoint()` → cycles through list
- ✓ After final waypoint, `advance_to_next_waypoint()` → returns False
- ✓ `is_mission_complete()` → True after all waypoints

#### **MotorDriver Tests** (mock serial)
- ✓ Build valid motor command packet with correct checksum
- ✓ Clamp speeds to [-1, 1]
- ✓ Serial send fails → return False, don't crash
- ✓ Emergency stop → sends special packet, returns True
- ✓ Battery voltage: parse ADC value → correct volts

#### **StateMachine Tests**
- ✓ IDLE → READY (on start) → valid transition
- ✓ READY → RUNNING (user "start") → valid transition
- ✓ RUNNING → PAUSED (user "pause") → valid transition
- ✓ PAUSED → RUNNING (user "resume") → valid transition
- ✓ RUNNING → FAULT (subsystem unhealthy) → auto-transition
- ✓ FAULT → RUNNING (user "reset" without operator check) → rejected (invalid)
- ✓ RUNNING → RUNNING (user "start") → no-op, stays RUNNING
- ✓ All states → FAULT on emergency stop command

#### **SafetySupervisor Tests**
- ✓ Clamp motor speeds to max limits
- ✓ Rate-limit acceleration: delta-speed > threshold → cap to slew rate
- ✓ Loop watchdog: 20 ms timeout for 50 Hz → `check_watchdog()` returns False if exceeded
- ✓ Emergency stop: `is_safe_to_command_motors()` → False after call
- ✓ All outputs remain valid (no NaN, Inf)

---

### 6.2 Integration Tests (Modules Together, Simulation)

**Combine modules in closed-loop simulation; no robot hardware yet.**

#### **Odometry Integration**
- ✓ Mock encoder and IMU readers with recorded trajectory data
- ✓ Run estimator at 50 Hz for 60 seconds → compare output pose to expected ground truth
- ✓ Measure drift: difference between estimated and true pose
- **Success**: Drift < 10 cm after 5 min mission in typical arena

#### **Motion Control + Odometry**
- ✓ Start at (0, 0, 0), goal at (1, 0, 0)
- ✓ Run motion controller + odometry in closed loop (simulation)
- ✓ Plot motor commands, odometry, and error over time
- **Success**: Reaches goal, no oscillation, settles in <5 sec

#### **Waypoint Following (3-point mission)**
- ✓ Waypoints: (0,0,0) → (1,0,0) → (1,1,0) → (0,1,0) (square)
- ✓ Run full stack: state machine → waypoint controller → motion controller → odometry
- ✓ Monitor each transition
- **Success**: Visits all 4 points in order, closes loop (final pose ≈ start), <20 cm error

#### **Sensor Failure Injection**
- ✓ Simulate encoder loss at t=30s → system falls back to IMU, continues to next waypoint
- ✓ Simulate IMU stale at t=45s → system continues with encoder-only, heading drifts but mission completes
- ✓ Simulate late sensor packets (200+ ms delay) → system marks stale, continues

#### **Edge Cases**
- ✓ Single waypoint (start == goal) → mission complete immediately
- ✓ Two identical consecutive waypoints → merged or skipped
- ✓ Very close waypoints (<5 cm) → no false double-advance
- ✓ Waypoint directly behind robot → large heading error, takes ~3 sec to turn
- ✓ Pause mid-mission → resume at same waypoint, same trajectory

---

### 6.3 Hardware Tests (Real Robot, Progressive Risk)

**Follow strict progression. Never skip a step.**

#### **Phase 1: Motor Command Verification (Robot Lifted, Wheels Free)**

1. **Setup**:
   - Robot on lift/stand so wheels don't touch ground
   - Operator at emergency stop switch
   - Logging enabled for motor commands and encoder ticks

2. **Tests**:
   - ✓ Send command (left=0.1 m/s, right=0.1 m/s) → both wheels spin forward
   - ✓ Send command (left=0.1, right=-0.1) → in-place CCW rotation
   - ✓ Emergency stop → wheels stop within 100 ms
   - ✓ Battery voltage read correctly (e.g., serial ADC value 200 → 12.0V)

3. **Success Criterion**:
   - Commands execute within 50 ms
   - No unexpected jerks or oscillation
   - Operator can stop robot immediately

#### **Phase 2: Odometry Verification (Straight-Line Test, Low Speed)**

1. **Setup**:
   - Robot on flat, open floor (2m × 2m area)
   - Mark start position with tape
   - Measure actual distance to verify encoders

2. **Test**:
   - Send constant forward speed 0.2 m/s for 5 seconds
   - Measure actual distance traveled (tape measure or manual odometry)
   - Compare encoder-based distance to measured distance

3. **Success Criterion**:
   - Encoder error < 5% (1 m traveled → error < 5 cm)
   - Straight-line deviation < ±3 cm (drift left/right)

#### **Phase 3: IMU Heading Verification (In-Place Rotation)**

1. **Setup**:
   - Robot stationary on ground
   - Compass handheld for reference

2. **Test**:
   - Command in-place 90° rotation (left wheel fwd, right wheel back at 0.2 m/s) for ~1.6 sec
   - Read IMU heading before and after
   - Compare to handheld compass and expected 90°

3. **Success Criterion**:
   - Heading error < 10° (so 90° command → final heading 80–100°)
   - Heading stable (no drift after rotation stops)

#### **Phase 4: Odometry Fusion (Dead-Reckoning Over 2-Min Mission)**

1. **Setup**:
   - Taped arena 3m × 3m with marked return point
   - Measure all distances and angles with tape + compass

2. **Test**:
   - Load 4-waypoint square: (0,0,0) → (1,0,0) → (1,1,0) → (0,1,0)
   - Run mission at 0.3 m/s
   - Operator watches and logs any issues
   - At mission end, measure robot final pose relative to start

3. **Success Criterion**:
   - Mission completes without operator intervention
   - Final position within 15 cm of start
   - Final heading within 10° of start (should be 0°)
   - Encoder readings match distances traveled (< 5% error)

#### **Phase 5: Closed-Loop Waypoint Following (3-Waypoint Indoor Circuit)**

1. **Setup**:
   - Cleared room, ~4m × 4m, no obstacles
   - Waypoints pre-loaded and verified in software

2. **Test**:
   - Start mission; operator has emergency stop ready
   - Robot autonomously navigates 3 waypoints
   - Log all commands, odometry, state transitions
   - Repeat 3 times with same waypoint list

3. **Success Criterion**:
   - All 3 runs complete without operator intervention
   - Each run reaches waypoints in correct order
   - Consistency: runs within 10% of each other (position, time)

#### **Phase 6: Fault Injection & Recovery**

1. **Setup**:
   - Same 3-waypoint circuit as Phase 5
   - Operator prepared to trigger faults

2. **Tests**:
   - ✓ **Simulate encoder stale** (stop encoder serial for 1 sec mid-mission) → robot continues on IMU
   - ✓ **Simulate motor stall** (gentle hand-block robot for 2 sec) → motor tries harder, then backs off or stops
   - ✓ **Low battery warning** (artificially set voltage read to 11.0V) → speed reduces, mission continues
   - ✓ **Manual pause** (operator sends pause command) → robot stops, resumes correctly

3. **Success Criterion**:
   - Robot recovers gracefully from each fault
   - No unintended motion or crash
   - Logs clearly identify fault and recovery action

---

### 6.4 Simulation & Playback Tools

**Build these to reduce real-robot testing time:**

1. **Trajectory Simulator**:
   - Mock encoder/IMU readers that replay recorded real-world data
   - Test control algorithms without hardware
   - Compare different tuning parameters

2. **Hardware-in-the-Loop (HIL) Harness**:
   - Robot connected via serial, motion controller running on host PC
   - Can pause, inspect state, inject faults, resume
   - Useful for offline debugging

3. **Logging Playback**:
   - Record complete mission: encoder deltas, IMU samples, motor commands, state transitions
   - Replay through control stack offline
   - Verify same outputs generated (regression test)

---

## 7. Implementation Roadmap

### **Week 1: Design & Core Data Structures**
- [ ] Define all data structures (Pose2D, MotorCommand, etc.)
- [ ] Create abstract base classes for modules
- [ ] Write stubs for all public interfaces
- [ ] Create unit test scaffolding

### **Week 2: Hardware Interface Layer**
- [ ] Implement EncoderReader (parse serial packets)
- [ ] Implement IMUReader (parse serial packets)
- [ ] Implement MotorDriver (build command packets, send)
- [ ] Unit tests for each (mock serial)
- [ ] Manual serial verification on real hardware (oscilloscope/terminal)

### **Week 3: Estimation Layer**
- [ ] Implement OdometryEstimator (differential-drive kinematics + sensor fusion)
- [ ] Unit tests: straight line, rotation, arc, noise robustness
- [ ] Covariance tracking
- [ ] Simulation tests with recorded sensor data

### **Week 4: Control Layer**
- [ ] Implement MotionController (PD loops for heading + speed)
- [ ] Tuning: empirically find good Kp, Kp_angular
- [ ] Rate limiting + saturation
- [ ] Unit tests + simulation closed-loop tests

### **Week 5: High-Level Layers**
- [ ] Implement WaypointController (sequence management)
- [ ] Implement StateMachine (IDLE, READY, RUNNING, etc.)
- [ ] Implement SafetySupervisor (clamping, watchdog, emergency stop)
- [ ] Unit tests for each

### **Week 6: Integration & Simulation**
- [ ] Combine all modules in simulation
- [ ] Run 3-waypoint mission sim 10× with different tuning
- [ ] Inject faults, verify recovery
- [ ] Logging and visualization

### **Week 7: Hardware Bring-Up**
- [ ] Phase 1: Motor commands (wheels free)
- [ ] Phase 2: Odometry (straight line)
- [ ] Phase 3: IMU heading (rotation)
- [ ] Phase 4: Full odometry over 2-min mission
- [ ] Document any calibration (wheel radius, wheelbase, encoder ticks/rev)

### **Week 8: Autonomous Mission**
- [ ] Phase 5: Closed-loop waypoint following
- [ ] Phase 6: Fault injection and recovery
- [ ] Tune control gains on real robot
- [ ] 3 successful repeat runs

### **Week 9: Optimization & Validation**
- [ ] Document performance: position error, heading error, repeatability
- [ ] Longer missions (10+ waypoints)
- [ ] Field test with realistic obstacles/lighting
- [ ] Code review and documentation

---

## 8. Safety Checklist

### **Before Any Hardware Test**

- [ ] Code compiles without warnings
- [ ] All motor outputs clamped to [-1, 1]
- [ ] Timeout handlers active (no infinite loops)
- [ ] Emergency stop command tested (serial)
- [ ] Operator at emergency stop switch
- [ ] Robot on lift or enclosed (wheels not on ground) for first tests
- [ ] Batteries charged, voltage measured
- [ ] Serial cable verified with oscilloscope
- [ ] Logs recording to disk
- [ ] Second person present (observer)

### **Before Full-Speed Motion**

- [ ] Low-speed tests (0.1–0.3 m/s) all passed
- [ ] Odometry verified accurate (< 5% error on straight line)
- [ ] Heading estimate within ±10° after 90° turn
- [ ] Mission completed without intervention at least once
- [ ] Recovery procedures tested (pause/resume, fault injection)
- [ ] Battery voltage monitored; mission completes before 10.5V
- [ ] Thermal checked: no overheating after 30 min run

### **Before Demonstration or Field Trial**

- [ ] Tagged release version in version control
- [ ] Known-good baseline: basic mission (2–3 waypoints) runs 5 consecutive times with <5% variation
- [ ] All subsystem health checks enabled
- [ ] Logging sent to persistent storage (SD card / USB drive)
- [ ] Manual override procedure drilled with team
- [ ] Spare batteries, cables, and USB adapter on site
- [ ] Outdoor testing: confirmed GPS/magnetometer not disrupted (if in new environment)

---

## 9. Key References & Guidelines

From **AI Programming Guide for Robotics**:

> "**Treat AI-generated code as a draft from a fast junior engineer**: useful, productive, and never trusted until reviewed and tested on the real system."

- Define clearly before coding
- Build in modules
- Test progressively (simulation → safe hardware → full mission)
- Keep safety deterministic (no AI in emergency stop)
- Log everything for debugging
- One change at a time, always have a rollback

---

## 10. Next Steps

1. **Agree on this design** with the team
2. **Review assumptions**: Do robot specs, sensor rates, and tolerances match reality?
3. **Choose implementation language** (Python, C++, Rust?)
4. **Assign module owners** (one person per layer)
5. **Implement Week 1** (data structures + interfaces)
6. **Begin unit tests** (one module at a time)
7. **After 3 weeks of integration testing**, request hardware bring-up review

---

**Document Version**: 1.0  
**Status**: Ready for Review  
**Author**: AI Robotics Design Assistant  
**Date**: 2026-07-22
