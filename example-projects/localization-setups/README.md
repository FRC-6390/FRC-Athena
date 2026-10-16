# Localization Setups

This standalone Athena/WPILib project shows inspectable pose pipelines built from swerve odometry and three camera APIs.

It also shows target streams from Limelight and PhotonVision, a supplier-bound team camera kind, a custom
`CameraAdapter`, and target-driven drivetrain Actions for aiming and approaching while maintaining standoff range.

## Pipeline

The example deliberately keeps each stage as a public field so robot code and telemetry can inspect every result:

```text
camera PoseSignal -> per-camera stage (limelightStage, photonStage, heliosStage) --\
                                                                                  +-> estimatedFieldPose
swerve odometry -------------------------------------------------------------------/
camera PoseSignals -> filteredVision -> fusedVision   (kept for comparison and telemetry)
```

- `VisionSources` configures Limelight, PhotonVision, and Helios camera mounts and per-source standard deviations.
- `TargetingExamples` reads the newest typed target sample and turns yaw/range error into live swerve Actions.
- `ExampleCameraAdapter` is the minimal service-provider shape for a team-owned camera library.
- `filteredVision` rejects invalid, stale, ambiguous, distant, and off-field measurements.
- `fusedVision` uses covariance intersection for camera samples close in time while rejecting disagreement.
- `weightedFieldPose` and `latestCameraPose` show alternative reusable localization stages.
- `limelightStage`, `photonStage` and `heliosStage` are the per-camera filter chains. Each one feeds the Kalman node directly.
- `estimatedFieldPose` uses Athena's WPILib-backed timestamp-aware swerve pose estimator, per-source covariance, process covariance, and an innovation gate.

## Recommended setup

For the Kalman node, give each camera its own input. Covariance intersection assumes the cameras may be correlated, so it
keeps the variance of one camera and throws away the gain from the others. Feeding each camera straight into
`Localizations.kalman()` lets independent frames add their information. The
`twoCamerasFeedingTheKalmanDirectlyConvergeFasterThanCovarianceIntersection` test in
`athena-localization` checks this on the same samples.

```java
estimatedFieldPose = Localizations.kalman()
        .input(odometry, limelightStage, photonStage, heliosStage)   // one input per camera
        .defaultVisionStdDevs(0.7, Math.toRadians(20.0))
        .visionStdDevSpeedScaling(0.5, 0.25)            // trust frames less while driving or turning
        .maxVisionAngularVelocity(3.0)                  // drop frames taken while spinning faster than 3 rad/s
        .seedFromVisionWhileDisabled(DriverStation::isDisabled)   // multi-tag pose snaps in while disabled
        .publishNetworkTables()
        .name("estimatedFieldPose");
```

- **Speed scaling** multiplies a frame's standard deviations by `1 + a*|v| + b*|omega|`, using the frame's own speeds
  when the camera reports them and odometry otherwise.
- **Spin gate** rejects frames taken above the rotation limit. The reason reads `spinning 4.1 rad/s > 3.0`.
- **Seeding while disabled** resets the estimate to a multi-tag frame, translation and heading, at most once per 0.25 s.
  Single-tag frames still take the normal update. The node has no mode of its own, so pass the supplier that tells it the
  robot is disabled.

## Camera diagnostics

Every Kalman localization reports a status per camera, refreshed each cycle: `visionStatus()` in code, and the
`athena/vision/<camera>/...` paths on NetworkTables and ARCP (see [docs/arcp.md](../../docs/arcp.md#vision-channels)).
Filters name their rejection, so the status says which one dropped a frame:

| State | Meaning | Example reason |
| --- | --- | --- |
| `OK` | The newest frame was accepted. | empty |
| `REJECTED` | A filter or the Kalman node dropped the newest frame. | `ambiguity 0.31 > 0.20`, `latency 0.210 s > 0.150 s`, `filter glare` |
| `RECOVERING` | The innovation or heading gate holds the frame until the camera is self-consistent for a few frames. | `innovation 41.0 > 9.0 (recovering 2/3)` |
| `UNSYNCED` | The camera's timestamps are ahead of robot time, so its clock is not synced yet. | `frame 3.2 s in the future` |
| `STALE` | No new frame for more than 0.5 s. | `no frame for 0.8 s` |

Custom filters that do not implement `rejectionReason(...)` are reported as `filter <name>`, or `filter #<position>`
when unnamed. Rates count over the last second. Covariance intersection stages report the name of the fused node, not
the camera, so per-camera diagnostics need per-camera Kalman inputs.

`LocalizationExamples.pose()` exposes the final result as `Pose2d`. Start preserves the current translation and resets the complete localization chain to zero heading. Back resets it to the example field pose `(2 m, 4 m, 0 rad)`.

## Alliance geometry

`LocalizationExamples.ALLIANCE` is an `AllianceTransform.rotate180(...)` over the field size. `blueStartPose` is a
blue-side pose that resolves to the red side on red. `resetHeadingForAlliance()` zeroes heading relative to the
alliance, so forward is toward the far wall for each alliance. `resetHeading()` keeps its field-zero behavior.

## Drive and hardware

The drivetrain uses `HardwareBus.rio()`, filled SDS module slots, absolute CANcoder offsets, brake mode, current `ControlSignal` bindings, and a teleop-only `Events.teleopPeriodic()` drive action. Replace the zero module offsets before using real hardware. Athena steering PID and drive feedforward outputs are volts.

## Discovery and simulation

There is no `configure()` or manual registration method. Athena discovers the mechanisms, devices, actions, hooks, odometry, localization chain, and `SwerveKinematics` simulation model from `Robot`'s fields. The same real declarations drive simulation. Camera simulation becomes available when the PhotonVision vendor dependency/provider is present; no camera-specific simulation code is required in robot code.

## Run

```text
./gradlew build
./gradlew simulateJava
```
