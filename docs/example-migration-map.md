# Example Migration Map

This map connects the evicted legacy example catalog to the replacement
example project. Rows marked `Implemented` have a root example, test, and
catalog entry covering the retained behavior. Rows marked `Boundary` point to
work that intentionally belongs outside this Java robot workspace. Rows marked
`Deferred` intentionally wait for the deferred annotation/state DSL pass.

| Legacy Example | Root Coverage | Status | Notes |
| --- | --- | --- | --- |
| `MovementCommandExamples.java` | `RobotCommands`, `DriveCommandExample`, `WpilibBoundaryExample` | Implemented | Distance and velocity command descriptors write into `RobotSpeeds`, adapt to real WPILib commands with named requirements, and can drive a WPILib differential runtime adapter. |
| `TankDriveCommandExamples.java` | `RobotCommands`, `DriveCommandExample`, `DriveExample`, `WpilibBoundaryExample` | Implemented | Differential drivetrain declarations and tank-drive speed commands bind to a WPILib differential drive adapter. |
| `VisionCommandExamples.java` | `RobotCommands`, `DriveCommandExample`, `VisionExample`, `VisionVendorAdapterExample` | Implemented | Vision observations, target-alignment feedback commands, and camera-frame-backed `VisionTurnAssist` are covered; PhotonVision and Limelight adapters produce the same frame model. |
| `AutoExecutionExamples.java` | `AutoExample` | Implemented | Chooser preparation and selected command execution view are covered. |
| `AutoProgramInputHandoffExamples.java` | `AutoExample` | Implemented | Scoped string, boolean, and numeric auto input handoff between prepared routines is covered. |
| `AutoRegistryExamples.java` | `AutoExample`, `AutoVendorAdapterExample` | Implemented | Source-backed chooser syntax and source registration are covered. |
| `DiagnosticsExamples.java` | `DiagnosticsExample`, `DashboardBridgeExample` | Implemented | Diagnostics snapshots and dashboard publication through the optional TCP transport are covered. |
| `ExampleRobotCoreConfig.java` | `WpilibBoundaryExample` | Implemented | Lifecycle hooks adapt to a real WPILib `TimedRobot`. |
| `LocalizationExamples.java` | `LocalizationExample`, `WpilibBoundaryExample` | Implemented | Vision weighting, slip detection, bounds, pose aliases, and WPILib pose-estimator measurement weights are covered. |
| `MotionLimitsExamples.java` | `MotionLimitsExample`, `ShooterExample`, `SwerveExample`, `LocalizationExample` | Implemented | Dedicated `MotionLimits` and `TimedRunner` utilities retain conservative limit aggregation and periodic runner behavior. |
| `RegistryAndBackendExamples.java` | `VendorOptionsExample`, `SimulationExample`, `StudicaImuAdapterExample` | Implemented | Backend registration and typed vendor boundary examples are covered through simulation, vendor option, and IMU adapter examples. |
| `RobotCoreHookExamples.java` | `WpilibBoundaryExample` | Implemented | Lifecycle hooks adapt to a real WPILib `TimedRobot`. |
| `RobotCoreHooksExamples.java` | `WpilibBoundaryExample` | Implemented | Lifecycle hooks adapt to a real WPILib `TimedRobot`. |
| `RobotCoreLifecycleExamples.java` | `WpilibBoundaryExample` | Implemented | Mode-specific init/periodic hooks are covered through `AthenaTimedRobot`. |
| `RobotNetworkTablesExamples.java` | `RobotTelemetry`, `WpilibBoundaryExample` | Implemented | Telemetry publishing can write through a real WPILib `NetworkTableInstance`. |
| `RobotSpeedsExamples.java` | `RobotSpeedsExample`, `DriveExample`, `SwerveExample`, `ControlUtilityExample` | Implemented | Source blending, output clamping, heading assist override, and field-relative conversion are retained without WPILib dependencies. |
| `ServiceLoaderRegistryExamples.java` | `SimulationExample`, `VendorOptionsExample`, `StudicaImuAdapterExample` | Implemented | Service-loaded backend behavior is represented by simulation and vendor backend tests/examples. |
| `TelemetryRegistryExamples.java` | `RobotTelemetry`, `DashboardBridgeExample` | Implemented | Telemetry snapshots, sinks, dashboard packets, and optional TCP publishing are covered. |
| `TypedInputResolverExamples.java` | `IntakeExample`, `ShooterExample`, `SensorExample` | Implemented | Digital, runtime number, and sensor wrapper inputs are covered. |
| `VendorModuleExamples.java` | `VendorOptionsExample`, `VisionVendorAdapterExample`, `AutoVendorAdapterExample`, `StudicaImuAdapterExample` | Implemented | CTRE TalonFX/Kraken, REV Spark Max/Flex, CTRE/REV integrated encoders, REV attached absolute encoders, REV through-bore encoders, CTRE CANcoder, CTRE Pigeon2, Studica/NavX, PhotonVision, Limelight, PathPlanner, and Choreo adapters call real vendor APIs. Phoenix 5 motor controllers are intentionally outside this pass because they require a separate optional dependency. |
| `ArcpControlExamples.java` | `DashboardBridgeExample` | Implemented | Explicit dashboard control messages can arrive over the optional TCP transport. |
| `ArcpLayoutExamples.java` | `DashboardBridgeExample` | Boundary | Dashboard packet and TCP transport exist here; layout/UI schema belongs to the separate dashboard workspace. |
| `ArcpReadExamples.java` | `DashboardBridgeExample`, `RobotTelemetry` | Implemented | Telemetry/diagnostics reads publish over newline-delimited dashboard JSON. |
| `ArcpRobotCoreExamples.java` | `DashboardBridgeExample`, `WpilibBoundaryExample` | Implemented | Dashboard transport and robot lifecycle boundaries are covered without putting dashboard dependencies in default robot runtime. |
| `ArcpSignalExamples.java` | `DashboardBridgeExample`, `RobotTelemetry` | Implemented | Dashboard signal payloads are represented by telemetry packets and control messages over the TCP transport. |
| `DifferentialDrivetrainExamples.java` | `DriveExample`, `WpilibBoundaryExample` | Implemented | Differential drivetrain syntax, validation, and WPILib runtime output binding are covered. |
| `SwerveDrivetrainExamples.java` | `SwerveExample`, `WpilibBoundaryExample` | Implemented | Swerve module syntax, geometry, gains, validation, and WPILib module-state output binding are covered. |
| `ControllerHelperExamples.java` | `ControlUtilityExample`, `WpilibBoundaryExample` | Implemented | Runtime controller helpers and WPILib-backed bindings are covered. |
| `FilterExamples.java` | `ControlUtilityExample` | Implemented | Scalar and pose filtering examples are covered. |
| `HardwareFactoryExamples.java` | `RobotHardware`, `VendorOptionsExample` | Implemented | Hardware aliases and typed vendor options are covered. |
| `ImuEncoderEdgeExamples.java` | `RobotHardware`, `ShooterExample`, `SimulationExample`, `StudicaImuAdapterExample` | Implemented | IMU and encoder specs plus real CTRE/REV integrated encoder reads, REV attached absolute encoder reads, REV through-bore duty-cycle reads, CTRE CANcoder reads, CTRE Pigeon2 yaw reads, and Studica/NavX yaw reads exist. |
| `StateMachineExamples.java` | `IntakeExample`, `ShooterExample`, `SuperstructureExample`, `CompositeSuperstructureExample`, `TurretSuperstructureExample` | Implemented | Mechanism and superstructure state declarations are covered. |
| `StateDslExamples.java` | `docs/api-v3-direction.md` | Deferred | The annotation/DSL state layer is intentionally deferred until the fluent Java API and spec model settle. |
| `StateSeedExamples.java` | `ShooterExample`, `SuperstructureExample`, `CompositeSuperstructureExample`, `TurretSuperstructureExample` | Implemented | Named state targets are covered by mechanism and superstructure examples. |
| `StateSetExamples.java` | `IntakeExample`, `ShooterExample` | Implemented | Plain Java state sets are represented by fluent mechanism state declarations and named setpoints. |
| `StateDslPluginExamples.java` | `docs/api-v3-direction.md` | Deferred | Annotation/state DSL work is intentionally deferred until fluent specs settle. |
| `MechanismInfrastructureExamples.java` | `IntakeExample`, `ShooterExample`, `VendorOptionsExample` | Implemented | Mechanism declarations, validation, and vendor option boundaries are covered. |
| `ArmMechanismExamples.java` | `MechanismV2Example`, `VendorOptionsExample` | Implemented | Angular position mechanism syntax is covered fluently; annotation loading is tracked by the deferred DSL rows. |
| `ElevatorMechanismExamples.java` | `MechanismV2Example`, `SensorExample`, `VendorOptionsExample` | Implemented | Linear position, typed input, and vendor option patterns are covered fluently; annotation loading is tracked by the deferred DSL rows. |
| `FlywheelMechanismExamples.java` | `MechanismV2Example`, `ShooterExample` | Implemented | Velocity mechanism states, encoder requirements, PID, and feedforward are covered fluently; annotation loading is tracked by the deferred DSL rows. |
| `SimpleMotorMechanismExamples.java` | `MechanismV2Example`, `IntakeExample` | Implemented | Percent-output mechanism syntax is covered fluently; annotation loading is tracked by the deferred DSL rows. |
| `TurretMechanismExamples.java` | `MechanismV2Example`, `TurretSuperstructureExample`, `VendorOptionsExample` | Implemented | Turret-style angular states and target inputs are covered fluently; annotation lifecycle callbacks are tracked by the deferred DSL rows. |
| `ExampleSuperstructure.java` | `SuperstructureExample` | Implemented | Basic multi-mechanism state coordination is covered. |
| `ExampleCompositeSuperstructure.java` | `CompositeSuperstructureExample` | Implemented | Nested superstructure propagation syntax, validation, planning, and runtime application are covered. |
| `ExampleTurretSuperstructure.java` | `TurretSuperstructureExample` | Implemented | Turret, hood, and shooter assembly coordination is covered. |
| `AthenaNTExamples.java` | `RobotTelemetry`, `WpilibBoundaryExample` | Implemented | NetworkTables publishing is backed by real WPILib entries through `WpilibNetworkTableWriter`. |
| `SensorWrapperExamples.java` | `SensorExample` | Implemented | Limit switch, beam-break, and camera target wrappers are covered. |
| `InputUtilityExamples.java` | `ControlUtilityExample`, `SensorExample` | Implemented | Input shaping and typed input/sensor examples are covered. |
| `MotorSimExamples.java` | `SimulationExample` | Implemented | Motor, IMU, and vision simulation stepping are covered. |
