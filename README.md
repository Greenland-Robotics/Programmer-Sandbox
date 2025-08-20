# What is this repo?
This repository is meant to be a space for programmers to work on code and experiment with using the Greenland Robotics Framework without having to worry about breaking or cluttering up their competition repo. I recommend using this repo to play with the basic robots that you have and learn things before the robot is built.

# Greenland Robotics Framework: Documentation

## Notes
- If you are using the PlayStation controllers, use the button map below
  | PlayStation    | Gamepad(in code) |
  | -------------- | ---------------- |
  | X              | A                |
  | O              | B                |
  | △              | Y                |
  | □              | X                |
- In TeleOp, the following buttons are in use by the `implementDriveLogic()` method(gamepad1 only)
  - `left_stick_x`
  - `left_stick_y`
  - `right_stick_x`
  - `right_bumper`
  - `left_trigger`
  - `right_trigger`<br>
  by using these in your teleop code, that one action will control both the driving and whatever you mapped it to

# Method Documentation
This documentation covers **all classes and methods** in `TeamCode/src/main/java/gcsrobotics/framework` for the GreenlandRoboticsFramework.  
It includes explanations for:  
- **What each method does**
- **How and why to use them**
- **Common usage scenarios**

---

## Table of Contents

- [`AutoBase`](#autobase)
- [`Constants`](#constants)
- [`DcMotorEnhanced`](#dcmotorenhanced)
- [`GoBildaPinpointDriver`](#gobildapinpointdriver)
- [`OpModeBase`](#opmodebase)
- [`TeleOpBase`](#teleopbase)

---

## AutoBase

**Purpose:**  
`AutoBase` is an abstract class for autonomous robot operation modes.  
It extends `OpModeBase`, providing accurate movement utilities and control structures for writing robust autonomous routines.

### Common Usage

- Extend `AutoBase` in your own autonomous OpMode.
- Override `initSequence()` (optional) for initialization logic.
- Override `runSequence()` for your autonomous sequence.
- Use movement methods like `path()`, `chain()`, `simpleDrive()`.

### Key Methods

#### `protected void initSequence()`
Override to add code you want to run in the `init` phase.

#### `protected abstract void runSequence()`
Override to define the autonomous actions (your main logic).

#### `protected void simpleDrive(Axis direction, double power, int time)`
Moves the robot simply in the specified `direction` ("vertical" or "horizontal") at a set power for a duration.
- **Use:** For short, direct movements (e.g., nudging into position).
- **Example:** `simpleDrive(Axis.X, 0.5, 1000);`

#### `protected void setPowers(double power)`
Sets all drive motors to the same power.
- **Use:** To move all wheels together, usually straight.

#### `protected void wait(int milliseconds)`
Pauses execution for a number of milliseconds, updating odometry and telemetry.
- **Use:** Preferable to `sleep()` in FTC, as it keeps robot feedback alive during the wait.

#### `protected void path(int targetX, int targetY, Axis forgiveAxis = Axis.NONE)`
Accurate movement to a coordinate (`targetX`, `targetY`) with optional axis forgiveness.
- **Use:** For precise autonomous positioning.
- **`forgiveAxis`:** `Axis.X` or `'Axis.Y'` if you want to ignore error on that axis (e.g., just get close horizontally).

#### `protected void chain(int targetX, int targetY, char forgiveAxis = ' ')`
Fast movement to a coordinate (less accurate than `path`).
- **Use:** When speed is more important than precision.

#### `protected void waitUntil(@NonNull Supplier<Boolean> condition)`
Waits until a given condition is true.
- **Use:** For waiting on asynchronous events or sensor thresholds.

#### **Private Utility Methods**  
**You don't need to worry about these, they are internal**
- `pidDrivePower(double error, boolean isX)`  
  Calculates PID-like drive power for pathing routines.
- `setMotorPowers(double xPower, double yPower, double headingCorrection)`  
  Sets individual wheel powers for advanced movement.
- `sendTelemetry(String label, ...)`
  Sends detailed telemetry for debugging pathing.
- `stopMotors()`
  Stops all drive motors.
- `notStuck(double targetX, double targetY)`
  Detects if the robot is stuck (not making progress).
- `getX(), getY(), getAngle()`
  Return current robot position/heading from odometry.

---

## Constants

**Purpose:**  
Central location for tunable constants (motor positions, PID values, setpoints, etc).

### Key Fields
  **Example fields that many teams may use, feel free to delete them if they are unnecessary**
- `clawClose`, `clawOpen`: Servo positions for the claw.
- `armUp`, `armMiddle`, `armDown`: Encoder positions for arm levels.
- `wristUp`, `wristDown`: Servo positions for wrist.
<br><br>
  **These fields are required, and are needed for basic functions. Do NOT delete these.**
- `ENCODER_TOLERANCE`: How close an encoder must be to target to count as "there".
- `KpDrive`, `KdDrive`, `KpTurn`, `KdTurn`: PID coefficients for drive and turn control.
- `autoMaxPower`: Max drive power for autonomous.
- **All constants are static and can be tuned live with FTC Dashboard.**

---

## DcMotorEnhanced

**Purpose:**  
A wrapper around FTC's `DcMotor` providing easier position control, power management, and utility operations.

### Constructors

- `public DcMotorEnhanced(DcMotor motor)`

### Key Methods

#### Position Control

- `setPosAndWait(int targetPosition, OpModeBase opmode)`
  - Moves to a position at default speed, waits until there.
- `setPosAndWait(int targetPosition, double speed, OpModeBase opmode)`
  - Moves to a position at given speed, waits until there.
- `setPosition(int targetPosition)`
  - Go to position at default speed (doesn’t wait).
- `setPosition(int targetPosition, double speed)`
  - Go to position at specified speed.

#### Speed & Power

- `setDefaultSpeed(double speed)`, `getDefaultSpeed()`
- `setPower(double power)`, `getPower()`

#### Encoder & State

- `reset()`  
  Resets encoder, restoring previous mode.
- `isAtTarget()`  
  Returns true if within `ENCODER_TOLERANCE` of target.
- `getCurrentPosition()`
- `isBusy()`

#### Run Modes & Directions

- `setMode(DcMotor.RunMode mode)`, `getMode()`
- `setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)`, `getZeroPowerBehavior()`
- `setDirection(DcMotorSimple.Direction direction)`, `getDirection()`

#### Base Access

- `getBaseMotor()`  
  Returns the raw underlying `DcMotor`.

---

## GoBildaPinpointDriver

**Purpose:**  
Driver for the goBILDA® Pinpoint Odometry Computer.  
Handles communication, configuration, and reading robot pose/velocity.

### Instantiation

- Constructed by FTC SDK’s hardwareMap (`hardwareMap.get(GoBildaPinpointDriver.class,"odo")`).

### Key Methods

#### Data Update

- `update()`
  - Reads *all* odometry data (should be called each loop).
- `update(ReadData data)`
  - Reads only the heading (for performance).

#### Offsets & Encoder Setup

- `setOffsets(double xOffset, double yOffset)`
  - Set pod offsets in mm (deprecated: see overload with `DistanceUnit`).
- `setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit)`
- `setEncoderDirections(EncoderDirection x, EncoderDirection y)`
- `setEncoderResolution(GoBildaOdometryPods podType)`
- `setEncoderResolution(double ticks_per_mm)`
- `setEncoderResolution(double ticks_per_unit, DistanceUnit distanceUnit)`

#### IMU

- `recalibrateIMU()`
  - Zero the IMU (robot must be still).
- `resetPosAndIMU()`
  - Zero position and IMU.

#### Pose and Heading

- `setPosition(Pose2D pos)`
- `setPosX(double posX, DistanceUnit unit)`
- `setPosY(double posY, DistanceUnit unit)`
- `setHeading(double heading, AngleUnit unit)`
- `getPosition()`
- `getAngle()`, `getAngle(AngleUnit)`, `getAngle(UnnormalizedAngleUnit)`
- `getX()`, `getY()`, `getAngle()`

#### Status

- `getDeviceID()`, `getDeviceVersion()`
- `getDeviceStatus()`
- `getLoopTime()`, `getFrequency()`
- `getEncoderX()`, `getEncoderY()`
- `getYawScalar()`

#### Velocity

- `getVelX()`, `getVelX(DistanceUnit)`
- `getVelY()`, `getVelY(DistanceUnit)`
- `getHeadingVelocity()`, `getHeadingVelocity(UnnormalizedAngleUnit)`
- `getVelocity()`

#### Miscellaneous

- `getXOffset(DistanceUnit)`, `getYOffset(DistanceUnit)`
- `getPinpoint()`

---

## OpModeBase

**Purpose:**  
Abstract base for all OpModes (autonomous or teleop).  
Handles hardware initialization and provides access to motors, servos, and odometry.

### Key Properties

- `fl`, `fr`, `bl`, `br` — Drivetrain motors (`DcMotorEnhanced`)
- `arm` — Arm motor (`DcMotorEnhanced`)
- `claw` — Claw servo
- `odo` — Odometry computer (`GoBildaPinpointDriver`)

### Main Methods

#### `protected abstract void runInit()`
Override for code to run in `init` phase.

#### `protected abstract void run()`
Override for code to run once `start` is pressed.

#### `private void initHardware()`
Initializes all hardware.  
- Configures motor/servo objects, odometry, directions.

#### `public void runOpMode()`
Main entrypoint for OpMode.  
- Sets up telemetry, hardware, runs `runInit()`, waits for start, then runs `run()`.

---

## TeleOpBase

**Purpose:**  
Base for teleop OpModes.  
Implements drive logic and framework for teleop control.

### Key Methods

#### `protected void runInit()`
- Sets drivetrain motors to `RUN_WITHOUT_ENCODER` (faster for teleop).
- Calls `inInit()` for your custom init code.

#### `protected void run()`
- Repeatedly calls `runLoop()` while OpMode is active.

#### `protected abstract void runLoop()`
Override to add the main teleop loop logic.

#### `protected abstract void inInit()`
Override to add code for the `init` phase of teleop.

#### `protected void setSpeed(double speed)`
- Set the drive speed multiplier.

#### `protected void implementDriveLogic()`
- Implements full mecanum drive logic using gamepads.
- Handles horizontal locking, slow mode, and trigger overrides.
- **Use:** Call this in your `runLoop()` to handle all drive movement.

---

# How to Use This Framework

1. **Create an OpMode** (autonomous: extend `AutoBase`; teleop: extend `TeleOpBase`)
2. **Override** the required abstract methods (`runSequence`, `runInit`, `runLoop`, etc.)
3. **Call movement and hardware methods** to control your robot.
4. **Tune constants** in `Constants.java` as needed (dashboard compatible).
5. **Use odometry and drive utilities** for accurate and efficient robot control.

---

## Questions?

- Each class and method is documented with purpose, usage, and scenarios.
- For detailed code reference, see the .java files.
- For FTC SDK integration, see [FTC documentation](https://ftc-docs.firstinspires.org/).
- If nothing else works, **ask Josh**. 

