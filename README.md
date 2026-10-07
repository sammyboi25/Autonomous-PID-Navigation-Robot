# Autonomous Differential-Drive Robot — PID Control

<img width="300" height="302" alt="image" src="https://github.com/user-attachments/assets/36520ee9-6dba-481d-9efc-abb575a50418" />

<img width="300" height="302" alt="image" src="https://github.com/user-attachments/assets/06b58d12-5dfc-414e-8ee2-68be1bcf9de8" />

---

## Overview

This project is a compact ** PID controlled two-wheel differential-drive autonomous robot** with a passive caster wheel.

The robot uses an **MPU6050 gyroscope** to estimate its heading and a **TCS34725 RGB sensor** to detect colored navigation cards.

Based on the detected color, the robot can continue straight, perform a 90° turn, or stop.

|   Color   | Action            |
| :-------: | ----------------- |
|   🔴 Red  | Stop              |
| 🟡 Yellow | Turn 90° Left     |
|  🔵 Blue  | Turn 90° Right    |
|   Other   | Continue Straight |

### Key Features

* PID-based heading control
* Gyroscope-based heading estimation
* Autonomous 90° turns
* Color-based navigation
* Custom-designed wheels
* Raspberry Pi embedded control
* Sensor calibration and experimentally tuned color classification

---

## Robot

https://github.com/user-attachments/assets/7f8da84f-6d9c-4864-beaf-307f05c705fd


The robot uses two independently controlled drive wheels and a passive caster, forming a **differential-drive configuration**.

---

## System Architecture

The control loop is:

```text
Sensors
   ↓
Read Heading / Color
   ↓
Navigation Logic
   ↓
PID Controller
   ↓
Motor Commands
   ↓
Robot Motion
   ↓
Gyroscope Feedback
   ↺
```

For straight-line motion:

```text
Left Motor  = Base Speed + PID Correction
Right Motor = Base Speed - PID Correction
```

---

## Hardware

* Raspberry Pi
* MPU6050 IMU / Gyroscope
* TCS34725 RGB Color Sensor
* 2 × DC Gear Motors
* Motor Driver
* Custom-designed wheels
* Battery
* Compact custom chassis

### Custom Wheel Design

The robot had to fit inside a **very constrained compact enclosure**, making commercially available wheels unsuitable for the required dimensions.

The wheels were therefore designed and manufactured specifically for the robot.

<img width="623" height="576" alt="image" src="https://github.com/user-attachments/assets/ed67cd29-06de-48c2-b472-20f161306318" />


The custom wheels allowed the design to:

* Fit within the limited chassis volume
* Maintain the required wheel dimensions
* Interface correctly with the motors
* Minimize unnecessary mechanical space

This turned the wheel design into part of the mechanical engineering problem rather than simply selecting an off-the-shelf component.

---

## PID Heading Control

The robot initially used proportional control:

```text
Correction = KP × Error
```

This was functional, but could leave persistent heading error and produce overshoot.

The controller was upgraded to PID:

```text
u = KP × error + KI × ∫error dt + KD × d(error)/dt
```

where:

* **P** responds to the current heading error
* **I** reduces persistent error
* **D** responds to how quickly the error is changing

Initial controller values:

```python
KP = 0.015
KI = 0.0008
KD = 0.004
```

The implementation also includes:

* Integral windup protection
* Derivative filtering
* Correction limits
* Motor speed limits
* PID state reset between movements

---

## Heading Estimation

The MPU6050 provides angular velocity rather than an absolute heading.

The robot estimates its heading by integrating the gyroscope:

```text
Heading = Heading + Angular Velocity × Δt
```

At startup, **500 stationary gyro measurements** are used to estimate the sensor bias.

A small deadzone is also applied to reduce the effect of low-level gyro noise.

During normal driving, the estimated heading is continuously compared with the target heading and the PID controller adjusts the relative motor speeds.

---

## Autonomous Navigation

The navigation logic is intentionally simple:

```text
Read Color
    │
    ├── Red ───────→ Stop
    │
    ├── Yellow ────→ Turn 90° Left
    │                    ↓
    │               Drive Forward
    │
    ├── Blue ──────→ Turn 90° Right
    │                    ↓
    │               Drive Forward
    │
    └── Other ────→ Drive Straight
                         ↓
                    PID Correction
```

### Turning

When a blue or yellow card is detected, the robot sets a new target heading:

```text
Blue   → Current Heading + 90°
Yellow → Current Heading - 90°
```

The robot then turns using PID feedback.

The turn is considered complete when:

```text
|Heading Error| < 2°
```

After the turn, the robot drives forward for approximately **1.2 seconds** to clear the navigation card before continuing.

### Red Stop

A red card acts as the termination condition:

```text
Red detected
      ↓
Stop both motors
      ↓
End navigation
```

---

## Color Sensor Challenge

One of the main engineering challenges was the behavior of the **TCS34725**.

The sensor did not produce the RGB relationships expected from the actual card colors. However, the readings were **consistent and repeatable rather than random**.

Instead of replacing the sensor, its behavior was experimentally characterized and incorporated into the detection algorithm.

The final color detection uses:

* Black/white calibration
* Channel scaling
* Relative RGB comparisons
* Experimentally determined thresholds

> **Key lesson:** A sensor does not necessarily need to produce textbook values. If its output is repeatable, its behavior can be characterized and mapped to the required result.

This allowed the robot to reliably identify the required colors using the existing hardware.

---

## Engineering Challenges

### 1. Compact Mechanical Design

The limited enclosure required custom wheels and careful component placement.

### 2. Unusual Sensor Behavior

The TCS34725 produced incorrect but repeatable RGB values. The sensor was characterized experimentally rather than replaced.

### 3. Gyroscope Drift

Gyroscopes naturally contain bias and noise. Startup calibration, a deadzone, and feedback control were used to reduce their effect.

### 4. Motor Differences

Even nominally identical DC motors can behave differently. Differential motor correction allows the controller to compensate for these differences while driving.

---

## Software

The project is written in **Python**.

### Libraries

* `gpiozero` — GPIO and motor control
* `adafruit_tcs34725` — RGB sensor
* `mpu6050` — IMU communication
* `board` — I²C interface
* `time` — control-loop timing

### Main Functions

```text
set_speeds()
      ↓
calculate_pid()
      ↓
get_angle()
      ↓
get_color()
      ↓
turn_90_smart()
      ↓
drive_with_pid_timed()
      ↓
Navigation Loop
```

---

## Safety & Constraints

Motor commands are constrained to keep the robot within its operating range:

```python
MAX_CORRECTION = 0.15
MAX_MOTOR_SPEED = 0.60
MIN_MOTOR_SPEED = 0.15
```

The controller includes:

* Maximum PID correction
* Motor speed limits
* Integral windup protection
* Controlled turning
* Motor shutdown on program interruption

---

## Future Improvements

Potential improvements include:

* Wheel encoders for accurate velocity measurement
* Encoder + gyro sensor fusion
* Improved heading estimation
* Automatic PID tuning
* HSV-based color classification
* Closed-loop velocity control
* Position and trajectory tracking

---

## What This Project Demonstrates

This project combines:

**Mechanical Design · Electronics · Embedded Programming · Sensors · Control Theory · Autonomous Navigation**

The main engineering feedback loop is:

```text
Sense → Understand → Control → Act → Measure → Correct
```

Rather than relying entirely on predetermined motor commands, the robot continuously measures its state and adjusts its behavior using feedback.

---

## Author

**Swayam Jogani**
Robotics & AI Engineering

**Interests:** Robotics · Autonomous Systems · Embedded Systems · Control · AI/ML
