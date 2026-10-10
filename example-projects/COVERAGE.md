# Example coverage

This matrix tracks team-facing Athena features that should remain represented by compile-checked robot code.
`./gradlew.bat compileExamples` publishes the current Athena snapshot locally and runs `build` in all eight projects.
The two projects with tests (`mechanism-examples`, `tank-drive`) also run them with `./gradlew test`.

| Feature area | Primary example | Coverage |
| --- | --- | --- |
| WPILib lifecycle and graph discovery | all projects | complete |
| Motors, encoders, IMUs, DIO, followers, neutral modes | `tank-drive`, `mechanism-examples` | complete |
| CAN, PWM, quadrature, analog, SPI, I2C, serial, USB, named CAN buses | `mechanism-examples` / `HardwareConnections` | declaration coverage |
| CTRE, REV, and Studica-specific configuration | `mechanism-examples` / `HardwareConnections` | declaration coverage |
| Open-loop and closed-loop mechanisms | `mechanism-examples`, `custom-controls` | complete |
| PID, feedforward, profiles, constraints, and custom loops | `mechanism-examples`, `custom-controls`, `rule-examples` | complete |
| Interpolated controls | `mechanism-examples` / `VelocityShooter` | complete |
| Stall detection and stall events | `mechanism-examples` / `IndexedIntake` | complete |
| Sequences, parallel/race/deadline actions, cycles, conditions, and timeouts | `mechanism-examples`, `rule-examples`, `auto-following` | complete |
| Templates and device slots | `mechanism-examples` / `TemplateRoller` | complete |
| SysId | `mechanism-examples` / `Turret` | complete |
| Annotation/custom telemetry and live tuning | `mechanism-examples` / `VelocityShooter`, `FieldTelemetry` | complete |
| Trace telemetry profiles | `mechanism-examples` / `Robot` | complete |
| Field geometry telemetry | `mechanism-examples` / `FieldTelemetry` | complete |
| Declarative simulation models | `tank-drive`, `swerve-drive`, `mechanism-examples`, `rule-examples` | complete |
| Explicit simulation session, handles, field targets, and custom physics | `mechanism-examples` / `SimulationHarness` | harness coverage |
| Inline runtime workers and failure handling | `mechanism-examples` / `RuntimeWorkerExamples` | harness coverage |
| Tank drive | `tank-drive`, `controller-bindings` | complete |
| Swerve drive and odometry | `swerve-drive`, `localization-setups` | complete |
| Choreo paths, markers, splits, and branches | `auto-following` | complete |
| PathPlanner AutoBuilder and path provider | `auto-following` | complete |
| Generated/custom path provider and preview geometry | `auto-following` / `GeneratedPathProvider` | complete |
| Auto chooser and automatic preview publishing | `auto-following` | complete |
| WPILib command adaptation and Athena command arbitration | `auto-following`, `controller-bindings` | complete |
| Camera pose estimation | `localization-setups` | complete |
| Localization filtering and fusion | `localization-setups` | complete |
| Camera targets and target-driven actions | `localization-setups` / `TargetingExamples` | complete |
| Supplier-bound custom camera and adapter extension | `localization-setups` / `VisionSources`, `ExampleCameraAdapter` | complete |
| Basic axes, buttons, toggles, and signal composition | `controller-bindings` | complete |
| Clicks, holds, repeats, debounce, thresholds, chords, and sequences | `controller-bindings` / `AdvancedControls` | complete |
| Typed units: angle and distance encoders, typed bindings, targets, tolerances, constraints | `mechanism-examples` / `SingleJointArm` | complete |
| Characterization, model-based feedforward, identified simulation plant | `mechanism-examples` / `SingleJointArm` | complete |
| Checks: blocked, load, feedback stall, slip | `mechanism-examples` / `SingleJointArm`, `StagedArm` | complete |
| Motion policies: retarget, release, interrupt holds, bounded commitments, attempt chains, faults | `mechanism-examples` / `SingleJointArm` | complete |
| Power profiles, endgame profile updates, battery floor | `mechanism-examples` / `VelocityShooter`, `Robot` | complete |
| Thermal derating and temperature estimates | `mechanism-examples` / `StagedArm` | complete |
| Staging enums with effort, speed, and acceleration caps | `mechanism-examples` / `StagedArm` | complete |
| Persisted telemetry, lock groups, and `tuningLock(...)` | `mechanism-examples` / `VelocityShooter`, `StagedArm`, `Robot` | complete |
| Interpolation recording and persisted tables | `mechanism-examples` / `VelocityShooter` | complete |
| Robot-level replay, ARCP, and power policy declarations | `mechanism-examples` / `Robot` | complete |
| Replay recording and replay in simulation | `mechanism-examples` / `ReplayRecordingTest` | test coverage |
| Seeded simulation exploration (no findings on a clean robot) | `tank-drive` / `ExplorationTest` | test coverage |
| Binding sets (`in(condition, ...)`) | `controller-bindings` / `AdvancedControls` | complete |
| Alliance transforms for geometry and path mirroring | `localization-setups` / `LocalizationExamples`, `auto-following` / `AutoContext` | complete |

## Intentionally not represented as robot examples

Backend registries, device handles, scheduler internals, runtime contexts, adapter registries, and concrete
vendor backend implementations are implementation SPI. Their coverage belongs in focused module tests rather
than team robot examples. The ARCP JNI binding is compile-checked through the `ArcpPolicy` declaration in
`mechanism-examples`; the Rust protocol server (`athena-arcp/rust`) and the `athena-arcp/host-dashboard` application
are not robot APIs and have no robot example. The deleted/placeholder telemetry, superstructure, and HeliOS modules
remain outside the active Gradle deliverable; add examples only if those deliverables are restored.
