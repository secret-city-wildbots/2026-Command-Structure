# Team 4265 - Command Structure Examples

## Overview

This FRC robot project is structured using the **Command-Based framework** recommended by WPILib. It is designed to be modular, readable, and easily extensible for both teleoperated and autonomous modes. The robot currently includes two core mechanisms: an `intake` and a `pivot`.

- The **IntakeSubsystem** controls a single motor used to collect or eject game pieces.

- The **PivotSubsystem** drives a pivoting arm using a motor and an `absolute encoder` for precise angular positioning. The pivot uses `Motion Magic` for smooth motion profiling, and supports software-defined soft limits to prevent over-rotation.

The project includes example commands to operate both mechanisms:

- Commands to run the intake / pivot motosr at fixed speeds.
- A specific command that moves the pivot to a user defined angle using Motion Magic and finishes when within a configurable error tolerance.

All control bindings are defined in `RobotContainer`, using the `onTrue()` and `whileTrue()` trigger methods to map controller inputs to subsystem behaviors. This project follows the standard directory and class layout defined by FIRST and WPILib for command-based robots.

Additionally, **simulation** support has been implemented for both the intake and pivot subsystems. This allows for hardware-free testing of motor behavior and sensor feedback within WPILib’s simulation framework, which can be run via VS Code or other simulation tools.

This codebase serves as a clean and practical example of a command-based robot architecture, complete with real-time control, encoder feedback, and simulation support.

---

<details>
  <summary>📁 <strong>Code Structure Overview</strong> (Click to expand)</summary>

---

### 📄 **Key Files**

#### `Main.java`

* **Purpose**: Entry point for the program.
* **Details**: Contains the `main()` method that starts the robot using WPILib’s `RobotBase.startRobot()`. You typically won’t need to modify this.

#### `Robot.java`

* **Purpose**: Controls the robot's mode transitions and periodic updates.
* **Details**: Defines methods like `robotInit()`, `teleopPeriodic()`, and `autonomousInit()`. Delegates actual logic to `RobotContainer` and the CommandScheduler.

#### `RobotContainer.java`

* **Purpose**: Central wiring for your robot’s components.
* **Details**: Instantiates subsystems, commands, and controller bindings. Returns the autonomous command during auto mode. Keeps setup code organized and separate from runtime logic.

---

### 📁 **Key Folders**

#### `/actors/subsystems`

* **Purpose**: Encapsulates control of specific robot mechanisms.
* **Details**: Each subsystem class (e.g. `IntakeSubsystem`, `PivotSubsystem`) manages motors, encoders, and control logic for one part of the robot. Exposes public methods used by commands (like `runIntake()` or `moveToAngle()`).

#### `/commands`

* **Purpose**: Defines robot behaviors using subsystems.
* **Details**: Each command implements a task or behavior (e.g. `PivotToPositionCommand`). Commands use `initialize()`, `execute()`, `end()`, and `isFinished()` to control the robot in response to driver input or autonomous instructions.

#### `/utils`

* **Purpose**: Stores shared utility classes and helper functions.
* **Details**: May include math helpers, logging utilities, custom PID wrappers, or unit converters. Keeps common logic centralized and reusable across subsystems and commands.

</details>
