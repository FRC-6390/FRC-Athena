# Migration Matrix

This matrix keeps the frozen repository root connected to the 2027 replacement
workspace. It is deliberately status-oriented: every legacy feature area should
either point at implemented 2027 evidence or at the next module that needs work.

Status values:

- `Implemented`: the 2027 module has code, tests, and docs for the replacement
  shape.
- `Boundary`: the 2027 module has a compile-tested API or adapter boundary, but
  the complete implementation belongs to a different workspace or optional
  artifact.
- `Deferred`: intentionally not part of the first fluent Java V3 pass.

| Legacy Area | Legacy Evidence | 2027 Target | Status | 2027 Evidence | Next Step |
| --- | --- | --- | --- | --- | --- |
| Hardware keys and aliases | `HardwareFactoryExamples.java`, `VendorModuleExamples.java` | `athena-api`, `athena-hardware`, `athena-plugin` | Implemented | `RobotHardware`, `BackendRegistryTest`, `verifyVendorMetadata` | Stable for V3. |
| Mechanism declarations, state targets, PID/feedforward, and runtime state application | `StateMachineExamples.java`, `StateSeedExamples.java`, mechanism docs | `athena-mechanisms` | Implemented | `IntakeExample`, `ShooterExample`, `MechanismController`, `MechanismConfigTest`, `MechanismV2Example` | Stable for the fluent V3 pass. |
| Superstructure coordination | `ExampleSuperstructure.java`, composite/turret examples | `athena-superstructure`, `example-project` | Implemented | `SuperstructureExample`, `CompositeSuperstructureExample`, `TurretSuperstructureExample`, `SuperstructureController`, `SuperstructureCommands`, `SuperstructureConfigTest` | Stable for the fluent V3 pass. |
| Advanced composite/turret superstructure parity examples | `ExampleCompositeSuperstructure.java`, `ExampleTurretSuperstructure.java` | `example-project`, `athena-superstructure` | Implemented | `CompositeSuperstructureExample`, `TurretSuperstructureExample`, `SuperstructurePlanner`, `SuperstructureController`, `SuperstructureCommands`, `SuperstructureConfigTest`, `docs/example-migration-map.md` | Stable for the fluent V3 pass. |
| Differential drivetrain | `DifferentialDrivetrainExamples.java` | `athena-drivetrain`, `athena-wpilib` | Implemented | `DriveExample`, `DifferentialDrivetrainConfigTest`, `WpilibDifferentialDriveAdapter`, `WpilibAdapterBoundaryTest`, `WpilibBoundaryExample` | Stable for V3. |
| Swerve drivetrain | `SwerveDrivetrainExamples.java` | `athena-drivetrain`, `athena-wpilib` | Implemented | `SwerveExample`, `DriveCommandExample`, `SwerveDrivetrainConfigTest`, `WpilibSwerveDriveAdapter`, `WpilibAdapterBoundaryTest`, `WpilibBoundaryExample` | Stable for V3. |
| Real WPILib command adapter | `TankDriveCommandExamples.java`, `MovementCommandExamples.java`, `VisionCommandExamples.java` | `athena-commands`, `athena-vision`, `athena-wpilib` | Implemented | `RobotCommands`, `DriveCommandExample`, `VisionTurnAssist`, `RobotDriveCommands`, `CommandGroups`, `CommandSpecTest`, `RobotDriveCommandsTest`, `WpilibBoundaryExample`, `WpilibCommandAdapter`, `WpilibCommandScheduler`, `WpilibTriggerBindings`, `WpilibDifferentialDriveAdapter`, `WpilibAdapterBoundaryTest` | Stable for V3. |
| Autonomous chooser/source registry/input handoff | `AutoRegistryExamples.java`, `AutoExecutionExamples.java`, `AutoProgramInputHandoffExamples.java` | `athena-auto` | Implemented | `AutoExample`, `AutoChooserConfigTest`, `AutoRegistryTest`, `AutoInputStore`, `AutoInputScope` | Stable for V3. |
| WPILib NetworkTables writer adapter | `TelemetryRegistryExamples.java`, `AthenaNTExamples.java`, `RobotNetworkTablesExamples.java` | `athena-telemetry`, `athena-wpilib` | Implemented | `RobotTelemetry`, `NetworkTablesTelemetrySinkTest`, `WpilibNetworkTableWriter`, `WpilibNetworkTableWriterTest`, `WpilibBoundaryExample`, `WpilibAdapterBoundaryTest` | Stable for V3. |
| Generic vision observations, command feedback, and pose estimates | vision command examples, localization examples | `athena-vision`, `athena-localization`, `example-project` | Implemented | `VisionExample`, `VisionTurnAssist`, `VisionPoseEstimator`, `VisionFrameTest`, `VisionPoseEstimatorTest`, `DriveCommandExample`, `LocalizationExample` | Stable for V3. |
| Simulation motor/IMU/world state | `MotorSimExamples.java` | `athena-simulation` | Implemented | `SimulationExample`, `SimWorldTest`, `SimMechanismTest`, `SimDrivetrainTest`, `SimImuBackendTest` | Stable for V3. |
| CTRE/REV motor percent, voltage, closed-loop target calls, integrated encoder reads, REV attached absolute encoder reads, REV through-bore reads, CTRE CANcoder, CTRE Pigeon2, and Studica/NavX IMU device calls | vendor module contracts, hardware examples | `athena-vendor-ctre`, `athena-vendor-rev`, `athena-vendor-studica` | Implemented | `CtreMotorBackendTest`, `CtreEncoderBackendTest`, `CtreImuBackendTest`, `RevMotorBackendTest`, `RevEncoderBackendTest`, `StudicaImuBackendTest`, `VendorOptionsExample`, `StudicaImuAdapterExample` | CTRE support is scoped to Phoenix 6 TalonFX/Kraken keys; Phoenix 5 Talon SRX/Victor SPX would require a separate optional adapter artifact and dependency. |
| PhotonVision camera library calls plus Limelight NetworkTables reads | PhotonVision/Limelight usage in legacy vision flows | `athena-vendor-photonvision`, `athena-vendor-limelight`, `athena-vision` | Implemented | `VisionVendorAdapterExample`, `PhotonVisionCameraAdapterTest`, `LimelightCameraAdapterTest`, `VisionTurnAssist`, `DriveCommandExample` | Stable for V3. |
| PathPlanner auto command calls plus Choreo trajectory loading and AutoFactory command wrappers | auto source/provider examples | `athena-vendor-pathplanner`, `athena-vendor-choreo` | Implemented | `AutoVendorAdapterExample`, `PathPlannerAutoSourceTest`, `ChoreoAutoSourceTest`, `ChoreoAutoFactoryAdapterTest` | Stable for V3. |
| Controller helpers, filters, motion limits, and robot speeds | `ControllerHelperExamples.java`, `FilterExamples.java`, `MotionLimitsExamples.java`, `RobotSpeedsExamples.java` | `athena-runtime`, `athena-wpilib` | Implemented | `ControlUtilityExample`, `MotionLimitsExample`, `RobotSpeedsExample`, `ControllerHelperTest`, `FilterPipelineTest`, `MotionLimitsTest`, `RobotSpeedsTest`, `WpilibDriverStationProfile`, `WpilibAdapterBoundaryTest` | Stable for V3. |
| Diagnostics and event logs | `DiagnosticsExamples.java` | `athena-runtime`, `athena-dashboard` | Implemented | `DiagnosticsExample`, `BoundedEventLogTest`, `DiagnosticsChannelTest`, `DashboardBridgeExample`, `DashboardTcpServer` | UI schema belongs to the separate dashboard workspace. |
| Robot lifecycle, controller bindings, and WPILib adapters | RobotCore lifecycle/hook examples and contracts, `ControllerHelperExamples.java` | `athena-wpilib` | Implemented | `AthenaTimedRobot`, `WpilibControllerBindings`, `WpilibDriverStationProfile`, `WpilibBoundaryExample`, `WpilibAdapterBoundaryTest` | Stable for V3. |
| Localization and pose estimation | `LocalizationExamples.java` | `athena-localization`, `athena-wpilib` | Implemented | `LocalizationExample`, `LocalizationConfigTest`, `VisionPoseEstimatorTest`, `WpilibPoseEstimatorAdapter`, `WpilibAdapterBoundaryTest`, `WpilibBoundaryExample` | Stable for V3. |
| Sensors/wrappers | `SensorWrapperExamples.java`, typed input docs | `athena-hardware`, `athena-vision` | Implemented | `SensorExample`, `SensorConfigTest`, `CameraTargetViewTest` | Stable for V3. |
| Real ARCP/dashboard transport | `athena-arcp`, ARCP examples | `athena-dashboard` or separate dashboard/control workspace | Implemented | `DashboardBridgeExample`, `DashboardWireCodec`, `DashboardTcpServer`, `DashboardBridgeTest` | Keep ARCP/UI dependencies outside the default robot runtime. |
| Dashboard layout/UI schema | `ArcpLayoutExamples.java` | separate dashboard/control workspace | Boundary | `DashboardBridgeExample`, `DashboardPacket`, `DashboardTcpServer` | The Java robot workspace owns packets and transport; interactive layout rendering belongs outside the robot runtime. |
| Annotation and state DSL plugin | state DSL plugin contract docs | none for V3 fluent pass | Deferred | `docs/api-v3-direction.md` | Revisit after fluent Java declarations and specs settle. |
| Behavioral parity pass for all legacy examples | `docs/coverage/examples-catalog.md` | `example-project`, `docs/example-migration-map.md` | Implemented | `docs/examples.md`, `docs/example-migration-map.md`, `verifyExampleDocs`, `verifyExampleMigrationMap` | Deferred annotation rows and separate dashboard UI work are explicitly out of the fluent Java V3 pass. |

## Current Priority

The V3 direction and fluent Java replacement pass are ready for review inside
`2027/`. Future work should start from a concrete new adapter, robot use case,
or dashboard workspace requirement instead of reopening the architecture shape.
