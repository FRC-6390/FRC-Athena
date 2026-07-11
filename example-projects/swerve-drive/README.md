# Swerve Drive

This is a standalone Athena/WPILib robot project for a four-module field-oriented swerve drive.

## What it demonstrates

- `HardwareBus.rio()` declarations for the motors, CANcoders, and Pigeon 2.
- `SwerveModules` templates with brake-mode slot configuration.
- Per-module absolute encoder offsets. Replace the zero values in `Constants.ModuleOffsets` with measured rotations.
- `SwerveKinematics.rectangular(...)` for module targets, desaturation, optimization, and odometry-compatible geometry.
- Field-oriented and robot-oriented driving from processed `DoubleSupplier` axes.
- Current `ControlSignal` bindings: Y selects field-oriented, A selects robot-oriented, and Start resets the relative heading.
- A teleop-only drive action through `Events.teleopPeriodic()`.

Athena reads the absolute steering angle on every target update. The module chooses the shortest steering path, reverses wheel direction when appropriate, and holds its previous angle at zero speed. `steerPid(...)` is voltage-based.

## Discovery and simulation

`Robot` has no `configure()` method. Athena discovers the drivetrain, controls, devices, hooks, and the `SwerveKinematics` simulation model from the mechanism graph. Running WPILib simulation uses the same declarations and actions as the real robot; no simulated motor kind or parallel drivetrain runtime is required.

## Run

```text
./gradlew build
./gradlew simulateJava
```
