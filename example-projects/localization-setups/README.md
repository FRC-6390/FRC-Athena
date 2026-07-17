# Localization Setups

This standalone Athena/WPILib project shows inspectable pose pipelines built from swerve odometry and three camera APIs.

It also shows target streams from Limelight and PhotonVision, a supplier-bound team camera kind, a custom
`CameraAdapter`, and target-driven drivetrain Actions for aiming and approaching while maintaining standoff range.

## Pipeline

The example deliberately keeps each stage as a public field so robot code and telemetry can inspect every result:

```text
camera PoseSignals -> filteredVision -> fusedVision
swerve odometry -----------------------> estimatedFieldPose
```

- `VisionSources` configures Limelight, PhotonVision, and Helios camera mounts and per-source standard deviations.
- `TargetingExamples` reads the newest typed target sample and turns yaw/range error into live swerve Actions.
- `ExampleCameraAdapter` is the minimal service-provider shape for a team-owned camera library.
- `filteredVision` rejects invalid, stale, ambiguous, distant, and off-field measurements.
- `fusedVision` uses covariance intersection for camera samples close in time while rejecting disagreement.
- `weightedFieldPose` and `latestCameraPose` show alternative reusable localization stages.
- `estimatedFieldPose` uses Athena's WPILib-backed timestamp-aware swerve pose estimator, per-source covariance, process covariance, and an innovation gate.

`LocalizationExamples.pose()` exposes the final result as `Pose2d`. Start preserves the current translation and resets the complete localization chain to zero heading. Back resets it to the example field pose `(2 m, 4 m, 0 rad)`.

## Drive and hardware

The drivetrain uses `HardwareBus.rio()`, filled SDS module slots, absolute CANcoder offsets, brake mode, current `ControlSignal` bindings, and a teleop-only `Events.teleopPeriodic()` drive action. Replace the zero module offsets before using real hardware. Athena steering PID and drive feedforward outputs are volts.

## Discovery and simulation

There is no `configure()` or manual registration method. Athena discovers the mechanisms, devices, actions, hooks, odometry, localization chain, and `SwerveKinematics` simulation model from `Robot`'s fields. The same real declarations drive simulation. Camera simulation becomes available when the PhotonVision vendor dependency/provider is present; no camera-specific simulation code is required in robot code.

## Run

```text
./gradlew build
./gradlew simulateJava
```
