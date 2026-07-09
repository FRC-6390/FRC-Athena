# Athena 2027 API Landscape

This document is the pruning map for the 2027 API. Rebuilt 2027 is the main usage proof, not the only feature whitelist. API is removed when it is an old concept, duplicate wiring, a V3 migration artifact, and unsupported by the new architecture. Adjacent feature implementations stay when they fit the final API with the same shape as a used feature.

Status words:

- `KEEP`: keep as public API with the listed final name.
- `RENAME`: keep as public API, but rename to the listed final name.
- `KEEP INTERNAL`: keep only the implementation needed by a kept API. This is not a hiding place for old public API.
- `REMOVE`: delete from public API because the concept is old, duplicated, and unsupported by the final architecture.
- `ADD`: missing public API needed by the 2027 architecture.
- `IMPLEMENT`: feature should exist, but must be rebuilt behind the final API instead of preserving old wiring.

## Definitive Naming Rules

| Name | Meaning |
| --- | --- |
| `Kind` | One option from a category list. |
| `Device` | Robot-facing hardware declaration. |
| `Handle` | Runtime-bound live hardware object. |
| `Signal` | Typed readable stream that emits one kind of value. |
| `Binding` | Relationship between declarations, triggers, controls, hooks, and followers. |
| `State` | Behavior request passed to a runtime. |
| `Config` | Removed V3 builder shape. |
| `Graph` | Internal lowered robot structure built from declarations. |
| `Runtime` | Executor that owns mutation, scheduling, simulation, and estimation. |
| `Context` | Runtime read/write surface passed to behavior code. |
| `Backend` | Vendor and sim factory that creates handles. |
| `Adapter` | Bridge to WPILib and vendor frameworks. |
| `Source` | External input boundary. |
| `Sink` | External output boundary. |
| `Catalog` | Static user entry point that creates declarations and states. |

## Rebuilt 2027 Used Public Surface

This is the current public API Rebuilt actually imports.

- Hardware kinds: `AthenaMotor`, `AthenaEncoder`, `AthenaImu`.
- Hardware declarations: `MotorRef`, `EncoderRef`, `ImuRef`, `DigitalInputRef`, `DigitalInputs`, `HardwareBus`.
- Hardware values: `GearRatioRef`, `RangeRef`, `EncoderUnit`.
- Simulation declarations: `Sim`, `SimRef`.
- Mechanisms: `Mechanism`, `MechanismState`, `States`, `Events`, `HookRef`, `HookRunner`, `EventContext`, `LifecycleMode`, `LifecyclePhase`, `MechanismRegistry`, `PathRef`, `Paths`.
- Control values: `PidRef`, `FeedforwardRef`.
- Runtime helpers: `ModifiedAxis`, `RobotVelocity`, `PoseSnapshot`, `MeasurementRef`, `Measurements`, `MeasurementStdDevs`.
- Localization: `Localizations`, `FieldBounds`, `LocalizationFilters`, `LocalizationRef`.
- Vision: `Cameras`, `HeliOSRef`, `LimelightRef`, `CameraMountPose`.
- Commands: `CommandSpec`.
- Drivetrain geometry: `WheelBase`, `TrackWidth`.

Everything else is required implementation behind these APIs. Old public surface is deleted.

## Final Public Shape

```mermaid
flowchart TD
    Robot["Robot code"]
    Kinds["Kind catalogs"]
    Devices["Device declarations"]
    Signals["Signals"]
    Bindings["Bindings"]
    States["States"]
    Graph["Runtime graph"]
    Runtime["RobotRuntime"]
    Handles["Handles"]
    Backend["Backends"]
    Adapter["WPILib and vendor adapters"]

    Robot --> Kinds
    Robot --> Devices
    Robot --> Signals
    Robot --> Bindings
    Robot --> States
    Devices --> Graph
    Signals --> Graph
    Bindings --> Graph
    Graph --> Runtime
    States --> Runtime
    Runtime --> Handles
    Backend --> Handles
    Runtime --> Adapter
```

## Kinds

`Kind` stays public. `Athena*` catalog names do not.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `HardwareKind` | `HardwareKind` | KEEP | Shared key contract for hardware categories. |
| `MotorKind` | `MotorKind` | KEEP | Used through motor declarations and vendor backends. |
| `EncoderKind` | `EncoderKind` | KEEP | Used through encoder declarations and swerve. |
| `ImuKind` | `ImuKind` | KEEP | Used by drivetrain and localization. |
| `CameraKind` | `CameraKind` | KEEP | Used by camera declarations. |
| `AthenaMotor` | `MotorKinds` | RENAME | Built-in motor kind catalog. |
| `AthenaEncoder` | `EncoderKinds` | RENAME | Built-in encoder kind catalog. |
| `AthenaImu` | `ImuKinds` | RENAME | Built-in IMU kind catalog. |
| `AthenaCamera` | `CameraKinds` | RENAME | Built-in camera kind catalog. |
| `EncoderUnit` | `EncoderUnit` | KEEP | Rebuilt uses it for swerve angle units. |
| `InputSourceKind` | removed | REMOVE | Old config/spec lowering detail. |
| `InputType` | removed | REMOVE | Old config/spec lowering detail. |
| `EncoderSignalType` | removed | REMOVE | Old config/spec lowering detail. |
| `NeutralMode` | `MotorNeutralMode` | RENAME | Hardware behavior option belongs with motor declarations. |
| `MotorCapability` | removed | REMOVE | Old public backend-validation shape. |
| `SensorKind` | removed | REMOVE | Rebuilt uses direct devices and signals, not sensor configs. |
| `BlockDirection` | removed | REMOVE | Old sensor-config API. |
| `AxisKind` | removed | REMOVE | Axis API is not used by Rebuilt. |
| `ControlMode` in `mechanism.core` | removed | REMOVE | Old public control-mode surface. |
| `ControlMode` in `mechanism.spec` | removed | REMOVE | Duplicate old spec surface. |
| `BlockPolicy` | removed | REMOVE | Rule API is removed. |
| `Direction` | removed | REMOVE | Old output resolver public surface. |
| `EventActivation` | removed | REMOVE | Replace with `Events` and `HookBinding` factories. |
| `HookTrigger` | removed | REMOVE | Replace with `Events` and `HookBinding` factories. |
| `LifecycleMode` | `LifecycleMode` | KEEP | Rebuilt robot runtime uses it. |
| `LifecyclePhase` | `LifecyclePhase` | KEEP | Rebuilt robot runtime uses it. |
| `RobotMode` | removed | REMOVE | `LifecycleMode` is the final lifecycle enum. |
| `TelemetryType` | removed | REMOVE | Public telemetry API is not used by Rebuilt. |
| `DiagnosticLevel` | removed | REMOVE | No public diagnostics API yet. |
| `AthenaFeature` | removed | REMOVE | Old public plugin feature surface. |

## Devices And Handles

`Device` is the robot-facing declaration. `Handle` is the live runtime object. This resolves the current name collision.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `MotorRef` | `MotorDevice` | RENAME | Rebuilt declares motors everywhere. |
| `EncoderRef` | `EncoderDevice` | RENAME | Rebuilt declares encoders for mechanisms and swerve. |
| `ImuRef` | `ImuDevice` | RENAME | Rebuilt declares drivetrain IMU. |
| `DigitalInputRef` | `DigitalInputDevice` | RENAME | Rebuilt uses limit switches. |
| `DigitalInputs` | `DigitalInputs` | KEEP | Catalog for digital input devices. |
| `AnalogInputRef` | removed | REMOVE | Not used by Rebuilt. |
| `AnalogInputs` | removed | REMOVE | Not used by Rebuilt. |
| `ControllerRef` | removed | REMOVE | Rebuilt uses a local controls wrapper, not Athena controller refs. |
| `Controllers` | removed | REMOVE | Not used by Rebuilt. |
| `ButtonRef` | removed | REMOVE | Not used by Rebuilt. |
| `ControllerAxisRef` | removed | REMOVE | Not used by Rebuilt. |
| `CameraRef` | `CameraDevice` | RENAME | Base public camera declaration. |
| `LimelightRef` | `LimelightDevice` | RENAME | Rebuilt uses Limelight camera declarations. |
| `HeliOSRef` | `HeliosDevice` | RENAME | Rebuilt uses HeliOS camera declarations. |
| `PhotonRef` | `PhotonVisionDevice` | RENAME | Adjacent camera backend using the same camera-device API. |
| `GenericCameraRef` | removed | REMOVE | Generic placeholder is not a concrete device family. |
| `MotorId` | removed | REMOVE | Fold fields into `MotorDevice`. |
| `EncoderId` | removed | REMOVE | Fold fields into `EncoderDevice`. |
| `CameraId` | removed | REMOVE | Fold fields into `CameraDevice`. |
| `HardwareBus` | `HardwareBus` | KEEP | Rebuilt uses named CAN bus constants. |
| backend `MotorDevice` | `MotorHandle` | RENAME | Live object returned by a backend. |
| backend `EncoderDevice` | `EncoderHandle` | RENAME | Live object returned by a backend. |
| backend `ImuDevice` | `ImuHandle` | RENAME | Live object returned by a backend. |
| `SimMotorDevice` | `SimMotorHandle` | RENAME | Sim live handle. |
| `SimImuDevice` | `SimImuHandle` | RENAME | Sim live handle. |
| `CtreMotorDevice` | `CtreMotorHandle` | RENAME | CTRE live handle. |
| `CtreEncoderDevice` | `CtreEncoderHandle` | RENAME | CTRE live handle. |
| `CtrePigeon2Device` | `CtrePigeon2Handle` | RENAME | CTRE live handle. |
| `RevMotorDevice` | `RevMotorHandle` | RENAME | REV live handle. |
| `RevThroughBoreEncoderDevice` | `RevThroughBoreEncoderHandle` | RENAME | REV live handle. |
| `StudicaImuDevice` | `StudicaImuHandle` | RENAME | Studica live handle. |

## Signals

Signals are typed streams. The final name must say what value comes out of the stream. `Signal` is not a generic replacement for old `Ref` names.

Final signal names:

- `BooleanSignal`: emits a boolean.
- `NumberSignal`: emits a double.
- `PoseSignal`: emits pose measurements.
- `TargetSignal`: emits vision targets.
- `MeasurementSignal`: emits mixed measurement types and is only for generic measurement composition.

Localization is not a signal. It is a pipeline that consumes pose/measurement signals and produces a pose estimate.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `MeasurementRef` | `MeasurementSignal` | RENAME | Rebuilt uses generic measurement composition. |
| `Measurements` | `Measurements` | KEEP | Catalog for typed measurement signals and values. |
| `MeasurementStdDevs` | `MeasurementStdDevs` | KEEP | Rebuilt uses vision weighting. |
| `CameraPoseRef` | `PoseSignal` | RENAME | Camera pose streams emit pose measurements. |
| `CameraTargetRef` | `TargetSignal` | RENAME | Target streams are a valid camera signal family. |
| `LocalizationRef` | `LocalizationPipeline` | RENAME | Rebuilt composes odometry, vision, and weighted field pose. |
| `LocalizationEstimatorRef` | removed | REMOVE | Old ref surface; `LocalizationPipeline` owns estimator composition. |
| `LocalizationFilterRef` | `LocalizationFilter` | RENAME | Rebuilt uses filters publicly. |
| `BooleanRef` | `BooleanSignal` | REMOVE | Name is valid, but Rebuilt uses direct digital input devices now. |
| `Booleans` | removed | REMOVE | Not used by Rebuilt. |
| `NumberRef` | `NumberSignal` | REMOVE | Name is valid, but Rebuilt does not use number streams now. |
| `Numbers` | removed | REMOVE | Not used by Rebuilt. |
| `RuntimeBoolean` | removed | REMOVE | Old runtime-ref surface. |
| `RuntimeNumber` | removed | REMOVE | Old runtime-ref surface. |
| `RuntimeEncoder` | removed | REMOVE | Old runtime-ref surface. |
| `RuntimeMotor` | removed | REMOVE | Old runtime-ref surface. |

## Values

Plain values stay public only when robot code uses them. Declaration-critical values stay with the declaration that owns them.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `RangeRef` | `Range` | RENAME | Rebuilt uses mechanism travel ranges. |
| `GearRatioRef` | `GearRatio` | RENAME | Rebuilt uses gearing for encoders and sim. |
| `PidRef` | `PidGains` | RENAME | Rebuilt uses PID constants. |
| `FeedforwardRef` | `FeedforwardGains` | RENAME | Rebuilt uses feedforward constants. |
| `CrtRef` | removed | REMOVE | Not used by Rebuilt. |
| `WheelBase` | `WheelBase` | KEEP | Rebuilt drivetrain uses it. |
| `TrackWidth` | `TrackWidth` | KEEP | Rebuilt drivetrain uses it. |
| Old module location ref | removed | REMOVE | Old graph/introspection helper; Rebuilt-style swerve uses mechanism-owned geometry. |
| `CameraMountPose` | `CameraMountPose` | KEEP | Rebuilt uses camera placement. |
| `ImuMountPose` | removed | REMOVE | Not used by Rebuilt. |
| `PoseSnapshot` | `PoseSnapshot` | KEEP | Rebuilt localization uses it. |
| `RobotVelocity` | `RobotVelocity` | KEEP | Rebuilt localization uses it. |
| `RobotSpeeds` | removed | REMOVE | Rebuilt uses WPILib `ChassisSpeeds`; public Athena drive runtime will define its own state. |
| `MotionLimits` | removed | REMOVE | Not used by Rebuilt. |
| `FilteredPose` | removed | REMOVE | Not used by Rebuilt. |
| `FilteredValue` | removed | REMOVE | Not used by Rebuilt. |
| measurement records | removed | REMOVE | `Measurements` should expose factories, not public record classes. |
| `VisionObservation` | removed | REMOVE | Not used directly by Rebuilt. |
| `VisionFrame` | removed | REMOVE | Not used directly by Rebuilt. |
| `CameraTargetView` | removed | REMOVE | Old target view DTO; target streams use `TargetSignal`. |
| `VisionPoseEstimate` | removed | REMOVE | Old public estimator output shape. |
| `LocalizationResult` | removed | REMOVE | Old public pipeline output shape. |
| telemetry/dashboard packet values | removed | REMOVE | Rebuilt has no public dashboard/telemetry API usage. |
| diagnostics values | removed | REMOVE | No public diagnostics API yet. |
| plugin feature values | removed | REMOVE | Old public plugin feature surface. |

## Bindings

Bindings connect robot declarations to runtime behavior.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `HookRef` | `HookBinding` | RENAME | Rebuilt uses hooks for lifecycle and limit switches. |
| `Events` | `Events` | KEEP | Rebuilt uses event catalog. |
| `HookRunner` | removed | REMOVE | Final robot code should use `RobotRuntime`; do not preserve the prototype runner API. |
| `EventContext` | `EventContext` | KEEP | Hook callbacks need context. |
| `PathRef` | `PathState` | RENAME | Rebuilt uses paths as mechanism states. |
| `Paths` | `Paths` | KEEP | Rebuilt uses path state catalog. |
| missing path provider binding | `PathProvider` | ADD | Common provider interface for PathPlanner, Choreo, and inline/test paths. |
| missing path marker binding | `PathMarkerBinding` | ADD | Marker-to-state/action binding for path following. |
| `ActionRef` | `ActionBinding` | RENAME | Used by `States.doOnce` and `States.run`; keep if those states stay. |
| `MotorFollowerRef` | `MotorFollowerBinding` | RENAME | Follower behavior is a hardware relationship. |
| `ControlRef` | `ControlBinding` | RENAME | Needed for drivetrain composite controls. |
| `Control` | `Controls` | RENAME | Public catalog for `ControlBinding`. |
| `ControlLoopRef` | `ControlLoop` | RENAME | Public control-loop declaration. |
| `ControlLoopBinding` | `ControlLoopBinding` | KEEP | Correct name. |
| `ConstraintRef` | removed | REMOVE | Not used by Rebuilt. |
| `RuleRef` | removed | REMOVE | Not used by Rebuilt. |
| `Rules` | removed | REMOVE | Not used by Rebuilt. |
| `OutputTarget` | removed | REMOVE | Old resolver public surface. |
| `SwerveModuleOrder` | removed | REMOVE | Runtime graph construction should not depend on public annotation API. |
| `InitialState` | removed | REMOVE | Rebuilt declares states directly. |

## States

States are the main public behavior API and stay public.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `MechanismState` | `State` | RENAME | Public behavior request type. |
| `States` | `States` | KEEP | Rebuilt uses it heavily. |
| nested records in `States` | removed | REMOVE | Keep factory methods public; remove public nested record classes. |
| nested records in `MotorRef` | removed | REMOVE | Device declarations must not own state classes. |
| `AxisState` | removed | REMOVE | Rebuilt does not use public axis API. |
| `AxisStateSource` | removed | REMOVE | Rebuilt does not use public axis API. |
| `MechanismStateSpec` | removed | REMOVE | Old spec path, not used by Rebuilt. |
| `SuperstructureStateSpec` | removed | REMOVE | Superstructure API is not used by Rebuilt. |
| `SimMotorState` | removed | REMOVE | Sim state should be owned by `SimRuntime`, not public. |
| `SimImuState` | removed | REMOVE | Sim state should be owned by `SimRuntime`, not public. |
| `SuperstructureTransitionPlan` | removed | REMOVE | Superstructure API is not used by Rebuilt. |
| `SuperstructureMechanismTarget` | removed | REMOVE | Superstructure API is not used by Rebuilt. |

## Removed DTO And Builder Layer

The old DTO/builder layer is not a 2027 API category. Rebuilt uses declarations and states, not config builders.

`Spec` does not survive as an Athena 2027 architecture layer. Current `*Spec` types do mechanical work in the old code: they hold normalized data, run validation, and feed a few backends, WPILib adapters, and sim helpers. That behavior moves into `Device`, `RobotGraph`, `RobotRuntime`, backend handles, and focused value types. Public `*Spec` records are deleted.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `MotorSpec` | removed | REMOVE | Replace with `MotorDevice` fields and runtime lowering. |
| `EncoderSpec` | removed | REMOVE | Replace with `EncoderDevice` fields and runtime lowering. |
| `ImuSpec` | removed | REMOVE | Replace with `ImuDevice` fields and runtime lowering. |
| `CameraSpec` | removed | REMOVE | Replace with `CameraDevice` fields and runtime lowering. |
| `LocalizationSpec` | removed | REMOVE | Replace with `LocalizationPipeline`. |
| `SwerveModuleSpec` | removed | REMOVE | Replace with swerve graph/runtime. |
| `SwerveDrivetrainSpec` | removed | REMOVE | Replace with swerve graph/runtime. |
| `DifferentialDrivetrainSpec` | removed | REMOVE | Rebuilt is swerve-only. |
| `CommandSpec` | `CommandState` | RENAME | Rebuilt experimental controls import it as command behavior descriptor. |
| `AutoChooserSpec` | removed | REMOVE | Old chooser DTO. |
| `AutoRoutineSpec` | removed | REMOVE | Old routine DTO; final autos are named `State` factories. |
| `RobotLifecycleSpec` | removed | REMOVE | `AthenaRobot` and `RobotRuntime` own lifecycle. |
| all `*Config` builders | removed | REMOVE | Old V3 builder public API, not used by Rebuilt. |
| `HeliOSConfig` | removed | REMOVE | Use `Cameras.helios` and `HeliosDevice` declarations. |

## Runtime Graph

`Definition` as a public-facing concept is out of date. The final system needs an internal runtime graph built from declarations, signals, bindings, states, controls, camera devices, path providers, and sim models. Robot code should not author `Definition` objects.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `MechanismDefinition` | `MechanismNode` | RENAME | Internal graph node behind `RobotRuntime`. |
| Old swerve drive/module definition API | removed | REMOVE | Swerve is mechanism composition, not an inspected graph. |
| missing root graph | `RobotGraph` | ADD | Internal root graph for mechanisms, drivetrain, controls, localization, autos, sim, and hardware. |
| missing hardware graph | `HardwareGraph` | ADD | Internal graph that binds device declarations to handles. |
| missing vision graph | `VisionGraph` | ADD | Internal graph for `CameraDevice` declarations and pose/target signals. |
| missing path graph | `PathGraph` | ADDED | Runtime marker graph for validated autonomous marker bindings. |

## Runtimes

Runtimes execute. Robot code should touch one public runtime host, not a collection of low-level runtime parts.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `MechanismRegistry` | `RobotRuntime` | RENAME | Rebuilt wraps it as `AthenaRobot`; make that library-owned. |
| `MechanismRuntime` | `MechanismRuntime` | KEEP INTERNAL | Execution detail required by `RobotRuntime`. |
| `MechanismController` | removed | REMOVE | Not used by Rebuilt. |
| `ControlLoopRuntime` | `ControlLoopRuntime` | KEEP INTERNAL | Execution detail required by `ControlLoop`. |
| `PathRuntime` | `PathRuntime` | KEEP INTERNAL | Execution detail required by `PathState`. |
| `AthenaTimedRobot` | `AthenaRobot` | RENAME | WPILib runtime host should own lifecycle and `RobotRuntime`. |
| `CommandRunner` | removed | REMOVE | Old command execution surface. |
| `WpilibCommandScheduler` | removed | REMOVE | Rebuilt controls use WPILib commands directly. |
| `AutoExecution` | `AutoRuntime` | IMPLEMENT | Needed for selected `State` autos, but old chooser/spec execution is removed. |
| `AutoRegistry` | removed | REMOVE | Old global registry. |
| `AutoInputStore` | removed | REMOVE | Old auto input store. |
| `AutoInputScope` | removed | REMOVE | Old auto input scope. |
| `VisionPoseEstimator` | `VisionPoseEstimator` | KEEP INTERNAL | Estimator implementation behind `LocalizationPipeline`. |
| `VisionTurnAssist` | removed | REMOVE | Not used by Rebuilt. |
| superstructure runtimes | removed | REMOVE | Rebuilt uses state composition instead of superstructure module. |
| `SimWorld` | `SimRuntime` | RENAME | Simulation runtime root. |
| `SimMechanism` | removed | REMOVE | Replace with `SimRuntime` model execution. |
| `SimModelRunner` | `SimModelRunner` | KEEP INTERNAL | Required by `SimRuntime` until replaced. |
| Temporary swerve sim bridge | removed | REMOVE | Temporary bridge duplicated mechanism behavior and direct target mutation. |
| `SimDifferentialDrive` | removed | REMOVE | Rebuilt is swerve-only. |
| `SimVisionCamera` | removed | REMOVE | Rebuilt does not use vision simulation. |
| telemetry/dashboard runtimes | removed | REMOVE | Not used by Rebuilt. |
| diagnostics runtimes | removed | REMOVE | No public diagnostics API yet. |
| `ModifiedAxis` | `ModifiedAxis` | KEEP | Rebuilt controls use it. |
| `TimedRunner` | removed | REMOVE | Not used by Rebuilt. |
| `Debouncer` | removed | REMOVE | Not used by Rebuilt. |
| `DelayedOutput` | removed | REMOVE | Not used by Rebuilt. |
| `ManualClock` | removed | REMOVE | Test helpers should not be public API. |

## Contexts

Contexts are runtime plumbing. Only event context stays public.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `EventContext` | `EventContext` | KEEP | Rebuilt hook callbacks use it. |
| `ActionContext` | `ActionContext` | KEEP INTERNAL | Runtime hardware surface required by `RobotRuntime`. |
| `MappedActionContext` | `MappedActionContext` | KEEP INTERNAL | Runtime implementation required by tests and sim. |
| `MechanismContext` | removed | REMOVE | Replace with `RobotRuntime` context and move behavior off hardware. |
| `ControlLoopContext` | `ControlLoopContext` | KEEP INTERNAL | Required by `ControlLoopRuntime`. |
| `RuleContext` | removed | REMOVE | Rules public API is removed. |
| `LocalizationFilterContext` | removed | REMOVE | Filters should receive typed pose/measurement data directly. |
| `LocalizationEstimateContext` | removed | REMOVE | `LocalizationPipeline` should own estimator context. |
| `AthenaValidationContext` | removed | REMOVE | Old validation context shape. |

## Backends

Backends stay implementation-facing. They should not shape robot code.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `BackendRegistry` | `BackendRegistry` | KEEP INTERNAL | Owned by `RobotRuntime`. |
| `MotorBackend` | `MotorBackend` | KEEP INTERNAL | Vendor adapter SPI required by motor handles. |
| `EncoderBackend` | `EncoderBackend` | KEEP INTERNAL | Vendor adapter SPI required by encoder handles. |
| `ImuBackend` | `ImuBackend` | KEEP INTERNAL | Vendor adapter SPI required by IMU handles. |
| `SimMotorBackend` | `SimMotorBackend` | KEEP INTERNAL | Sim adapter required by `SimRuntime`. |
| `SimImuBackend` | `SimImuBackend` | KEEP INTERNAL | Sim adapter required by `SimRuntime`. |
| `CtreMotorBackend` | `CtreMotorBackend` | KEEP INTERNAL | Vendor adapter required by CTRE motors. |
| `CtreEncoderBackend` | `CtreEncoderBackend` | KEEP INTERNAL | Vendor adapter required by CTRE encoders. |
| `CtreImuBackend` | `CtreImuBackend` | KEEP INTERNAL | Vendor adapter required by CTRE IMUs. |
| `RevMotorBackend` | `RevMotorBackend` | KEEP INTERNAL | Vendor adapter required by REV motors. |
| `RevEncoderBackend` | `RevEncoderBackend` | KEEP INTERNAL | Vendor adapter required by REV encoders. |
| `StudicaImuBackend` | `StudicaImuBackend` | KEEP INTERNAL | Vendor adapter required by Studica IMUs. |

## Adapters, Sources, And Sinks

Only adapters needed by Rebuilt stay public. Most source/sink APIs are not currently justified as public.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `WpilibControllerBindings` | removed | REMOVE | Rebuilt uses local controls wrapper. |
| `WpilibDriverStationProfile` | removed | REMOVE | Not used by Rebuilt. |
| `WpilibTriggerBindings` | removed | REMOVE | Rebuilt uses local controls wrapper. |
| `WpilibCommandAdapter` | removed | REMOVE | Rebuilt local controls use WPILib commands directly. |
| `WpilibCommandPathRuntime` | removed | REMOVE | Replace with Athena `PathRuntime` and provider adapters. |
| `WpilibDifferentialDriveAdapter` | removed | REMOVE | Rebuilt is swerve-only. |
| `WpilibSwerveDriveAdapter` | removed | REMOVE | Replace with mechanism-composed swerve control. |
| `WpilibPoseEstimatorAdapter` | `WpilibPoseEstimatorAdapter` | KEEP INTERNAL | Required behind `LocalizationPipeline`. |
| `WpilibNetworkTableWriter` | removed | REMOVE | Telemetry not used by Rebuilt. |
| `WpilibNetworkTableSink` | removed | REMOVE | Telemetry not used by Rebuilt. |
| `AutoSource` | `PathProvider` | RENAME | Provider concept stays, auto/source naming goes away. |
| `ChoreoAutoSource` | `ChoreoPathProvider` | RENAME | Choreo support stays behind `PathState`. |
| `PathPlannerAutoSource` | `PathPlannerPathProvider` | RENAME | PathPlanner support stays behind `PathState`. |
| `ChoreoAutoFactoryAdapter` | `ChoreoPathAdapter` | RENAME | Choreo adapter rebuilt for `PathProvider`. |
| `ChoreoAutos` | removed | REMOVE | Old catalog name. Use `Paths.choreo`. |
| `PathPlannerAutos` | removed | REMOVE | Old catalog name. Use `Paths.pathPlanner`. |
| `PhotonVisionCameraAdapter` | `PhotonVisionCameraAdapter` | KEEP INTERNAL | Backend for `PhotonVisionDevice`. |
| `LimelightCameraAdapter` | `LimelightCameraAdapter` | KEEP INTERNAL | Backend for `LimelightDevice`. |
| `HeliOSCameraAdapter` | `HeliOSCameraAdapter` | KEEP INTERNAL | Backend for `HeliosDevice`. |
| `HeliOSCameraProvider` | removed | REMOVE | Old provider surface. |
| `HeliOSProvider` | removed | REMOVE | Old provider surface. |
| `TelemetrySink` | removed | REMOVE | Not used by Rebuilt. |
| `NetworkTablesTelemetrySink` | removed | REMOVE | Not used by Rebuilt. |
| `NetworkTableWriter` | removed | REMOVE | Not used by Rebuilt. |
| `InMemoryNetworkTableWriter` | removed | REMOVE | Telemetry API is removed. |
| `DashboardSink` | removed | REMOVE | Dashboard API not used by Rebuilt. |

## Catalogs

Catalogs are public only when they create the kept robot API.

| Current API | Final API | Status | Reason |
| --- | --- | --- | --- |
| `DigitalInputs` | `DigitalInputs` | KEEP | Rebuilt uses it. |
| `Sim` | `SimModels` | RENAME | Rebuilt uses sim declarations. |
| `States` | `States` | KEEP | Rebuilt uses it heavily. |
| `Events` | `Events` | KEEP | Rebuilt uses it. |
| `Paths` | `Paths` | KEEP | Rebuilt autos use it. |
| `Cameras` | `Cameras` | KEEP | Rebuilt localization uses it. |
| `FieldBounds` | `FieldBounds` | KEEP | Rebuilt localization uses it. |
| `LocalizationFilters` | `LocalizationFilters` | KEEP | Rebuilt localization uses it. |
| `Localizations` | `Localizations` | KEEP | Rebuilt localization uses it. |
| `Measurements` | `Measurements` | KEEP | Rebuilt localization uses it. |
| `AnalogInputs` | removed | REMOVE | Not used by Rebuilt. |
| `Booleans` | removed | REMOVE | Not used by Rebuilt. |
| `Numbers` | removed | REMOVE | Not used by Rebuilt. |
| `Controllers` | removed | REMOVE | Rebuilt local controls wrapper replaces it. |
| `Sensors` | removed | REMOVE | Not used by Rebuilt. |
| `Axes` | removed | REMOVE | Rebuilt does not use public axis API. |
| `Rules` | removed | REMOVE | Rebuilt does not use public rules. |
| `Hooks` | removed | REMOVE | Rebuilt uses `Events` directly. |
| `Mechanisms` | removed | REMOVE | Old config catalog. |
| `Drivetrains` | removed | REMOVE | Rebuilt uses direct swerve module declarations. |
| `SwerveModules` | `SwerveModules` | KEEP | Public module catalog must be expanded. |
| `Photon` | removed | REMOVE | Old standalone catalog. Use `Cameras.photonVision`. |
| `Limelight` | removed | REMOVE | `Cameras.limelight` is the public entry point. |
| `HeliOS` | removed | REMOVE | `Cameras.helios` is the public entry point. |
| `Autos` | `Autos` | IMPLEMENT | Final catalog creates named auto `State` routines, not specs/registries. |
| `CommandGroups` | removed | REMOVE | Not used by Rebuilt. |
| `RobotDriveCommands` | removed | REMOVE | Not used by Rebuilt. |
| `Superstructures` | removed | REMOVE | Rebuilt uses state composition. |

## Whole Modules Needing Public API Pruning

These modules need old public surface removed. Feature support stays when it can be expressed through the final API.

- `athena-superstructure`: remove public API. Rebuilt composes subsystem states directly with `States.set`.
- `athena-dashboard`: remove public API. Rebuilt does not use dashboard transport.
- `athena-telemetry`: remove public API. Rebuilt does not publish through Athena telemetry yet.
- `athena-arcp`: keep source only as isolated tooling/transport. It is absent from `settings.gradle`, `athenaPublishedArtifacts`, and the vendordep, so ARCP is not part of the active 2027 library deliverable. Reintroduce only as an optional transport behind a final `Signal`/`Sink` extension point.
- `athena-auto`: remove public chooser/spec/registry API. Add named auto `State` routines and selected-auto runtime support.
- `athena-commands`: keep only the renamed `CommandState` concept if the local controls wrapper still needs it.
- `athena-vendor-photonvision`: keep camera backend support behind `PhotonVisionDevice`.
- `athena-vendor-pathplanner`: keep path provider support behind `Paths.pathPlanner`.
- `athena-vendor-choreo`: keep path provider support behind `Paths.choreo`.
- `athena-plugin`: move public classes internal to Gradle plugin implementation.
- `example-project`: keep absent from the build. Future examples should return as final API acceptance fixtures, not as compatibility examples for removed V3/spec/ref APIs.

## Current Package Boundaries

- `ca.frc6390.athena.hardware.device`: hardware declarations and physical value objects such as `MotorDevice`, `EncoderDevice`, `ImuDevice`, `DigitalInputDevice`, `GearRatio`, `Range`, and neutral/follower metadata.
- `ca.frc6390.athena.hardware.runtime`: runtime hardware access surfaces such as `HardwareGraph`, `ActionContext`, and `MappedActionContext`.
- `ca.frc6390.athena.hardware.sim`: simulation declarations such as `SimModel`, `SimModels`, `SimLimit`, and `SimProfile`.
- `ca.frc6390.athena.hardware.backend`: backend SPI and live runtime handles such as `MotorBackend`, `MotorHandle`, and `HardwareIdentity`.

## Additions Needed Before Robot Testing

- `RobotRuntime`: root runtime in `athena-robot` that owns cross-module lifecycle composition for hardware context binding, mechanisms, drivetrain, vision refresh, localization evaluation, autos, path markers, commands, and simulation.
- `AthenaRobot`: WPILib-facing runtime host that wraps `RobotRuntime`.
- `HardwareGraph`: lowered hardware graph from device declarations.
- `RobotGraph`: root runtime graph for mechanisms, drivetrain, controls, localization, sim, and autos.
- `VisionGraph`: camera device and signal graph.
- `PathGraph`: path marker graph for validated autonomous marker bindings.
- Swerve control is owned by mechanisms and normal state/output resolution, matching the Rebuilt 2027 pattern.
- `ControlBinding`: library-owned version of the Rebuilt local `experimental.controls.ControlRef`.
- `SimRuntime`: runs `SimModel` updates through the same device handles as real hardware.
- `PathProvider`: common provider interface for PathPlanner, Choreo, and inline/test paths.
- `AutoRuntime`: selected-auto state execution without the old chooser/spec registry.

## First Removal Order

1. Remove public config/spec builders from student-facing docs and examples.
2. Rename kind catalogs from `Athena*` to `*Kinds`.
3. Rename used refs to `Device`, `Signal`, `Binding`, `State`, and value names.
4. Move runtime handles and backend interfaces out of the public declaration surface.
5. Remove unused catalogs: analog inputs, booleans, numbers, sensors, axes, rules, hooks, drivetrain configs, command groups, superstructure, dashboard, telemetry, and old auto registry.
6. Move mechanism runtime primitives out of `athena-hardware`.
7. Add `RobotRuntime` and `AthenaRobot`, then port Rebuilt local runtime/control/swerve code into the library.
