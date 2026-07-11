# Athena 2027 Module Notes

## athena-api

- [x] Code optimization: current module is marker interfaces and enum key catalogs; no meaningful logic optimization needed.
- [x] Runtime optimization: no runtime code here. Key strings are static enum data, which is appropriate.
- [x] Test surfaces needed: stable key values, uniqueness across built-in catalogs, and marker-interface contracts are covered by `KindCatalogTest`.
- [x] Upkeep: current Java sources are moved off old `Athena*` catalogs and old `*Id` DTOs. Watch for these names when examples are rebuilt.
- [x] Architecture: public generic `Kind` contracts stay, built-in kind lists use definitive names, and identity DTOs are no longer treated as the hardware declaration layer.

## athena-runtime

- [x] Overall task: make `athena-runtime` the low-level value/signal layer that can be consumed by a graph-owned robot runtime.
- [x] Code optimization: replaced stream-based `MeasurementSignal.latestMeasurement()` with a direct loop and added `SingleMeasurementSignal` so common single-sample factories can answer latest-sample reads without list scanning.
  - [x] Update default latest-sample reduction to avoid stream allocation.
  - [x] Route `Measurements.poses`, `poseAndSpeeds`, `speeds`, and `measurement` through a single-sample signal implementation.
  - [x] Add `MeasurementSnapshot` so runtime-owned graph cycles can cache measurement lists and latest-sample reduction once per refresh.
- [x] Runtime optimization: latest-measurement reduction is now centralized on the signal contract for runtime consumers.
  - [x] Keep generic list signals compatible.
  - [x] `RobotRuntime` now owns localization pipeline refresh through `MeasurementSnapshot` instead of directly pulling raw lists during periodic.
- [x] Test surfaces needed: `MeasurementSignal` policy chaining, latency/max-age filtering, disabled signals, standard deviation application, `ModifiedAxis`, `RobotVelocity.fieldToRobot`, and `RobotVelocity.clamp` are covered by `MeasurementSignalPolicyTest`.
  - [x] Cover `MeasurementSnapshot` list/latest caching until explicit refresh.
- [x] Upkeep: current Java sources are moved off deleted names such as `MeasurementRef`, `RobotSpeeds`, and `ValidationReport`.
  - [x] Focused stale-name scan found no deleted runtime names in `athena-runtime/src/main/java`.
- [x] Architecture: `athena-runtime` is now the low-level value/signal layer; the root runtime lives in `athena-robot` instead of this module.
  - [x] Final ownership converges through `athena-robot`, which composes mechanism runtime, hardware graph, drivetrain runtime, localization scheduling, paths, autos, command graph, and sim graph.
  - [x] `MechanismScheduler` can now be constructed with a runtime-owned `ActionContext` such as `HardwareGraph`, so mechanism Action dispatch can resolve hardware through the graph boundary.

## athena-hardware

- [x] Overall: finish hardware declaration/runtime boundary cleanup.
- [x] Code optimization: `MotorDevice`, `EncoderDevice`, and `SimModel` remain small declaration types without the old DTO/spec layer.
- [x] Package cleanup: removed `ca.frc6390.athena.hardware.ref`; declarations now live in `ca.frc6390.athena.hardware.device`, action contexts in `ca.frc6390.athena.hardware.runtime`, and sim declarations in `ca.frc6390.athena.hardware.sim`.
- [x] Runtime ownership: added `HardwareGraph` as the runtime-owned resolver/cache for motor, encoder, and IMU handles.
- [x] Handle identity: added `HardwareIdentity` so runtime and simulation maps include category, kind, bus, id, and integrated-encoder detail instead of relying on display/default names.
- [x] Backend discovery: `BackendRegistry.global()` now discovers lazily, and `setGlobal(null)` clears the override without eager `ServiceLoader` work.
  - [x] Service discovery skips unavailable provider classes so the single Athena vendordep can include dormant vendor adapter jars without requiring every real vendor library.
- [x] Integrated encoder behavior: `HardwareGraph` resolves integrated motor encoders through a cached encoder-handle facade backed by the motor handle.
- [x] Test surfaces needed: device validation defaults, integrated encoder behavior, digital input inversion/binding, sim model composition, backend lookup, and default handle exception behavior are covered by `HardwareGraphTest`.
  - [x] `HardwareGraphTest` covers `HardwareIdentity`, graph handle caching/backend lookup, integrated motor encoder facade behavior, missing-backend failures, default handle exceptions, basic device validation failures, digital input binding/inversion, and `SimModel` composition.
- [x] Upkeep: moved `VendorOptions` to `ca.frc6390.athena.hardware.vendor` and documented it as typed vendor declaration metadata, not a spec layer.
- [x] Architecture: `HardwareGraph` now implements `ActionContext`, so mutation can be owned by runtime graph plumbing instead of direct map contexts.
- [x] Architecture follow-up: `ActionContext` is the public runtime hardware boundary for cross-module graph plumbing, and `MappedActionContext` remains a focused public test/manual-handle helper. The final `athena-robot` runtime owns when those contexts are used.

## athena-mechanisms

- [x] Overall task: make `athena-mechanisms` the mechanism graph/Action execution slice of the final robot runtime.
- [x] Code optimization: moved mechanism graph discovery into an internal `RobotGraph` and made internal `MechanismRuntime` consume a prebuilt `MechanismNode`.
  - [x] Add internal `RobotGraph` for mechanism node caching, declaration collection, and sim model discovery.
  - [x] Use cached `MechanismNode` when constructing `MechanismRuntime`.
  - [x] Cache class field lists in package-private `MechanismIntrospector`, `HookIntrospector`, and `PathIntrospector`.
  - [x] Cache per-node `PathRuntime` resolution after first path-Action entry.
  - [x] Lower `StateScheduler` into a cached scheduler-node tree so static child Action nodes are built once and runtime Action no longer uses generated string path keys.
- [x] Runtime optimization: hook discovery is no longer performed every periodic tick.
  - [x] Capture hook bindings from `MechanismNode` at runtime construction.
  - [x] Reuse graph-owned declaration/simulation discovery in `RobotRuntime.bindInMemoryRuntime()` and simulation refresh.
  - [x] Rename old `HookRunner` plumbing to package-private `HookRuntime`.
- [x] Test surfaces needed: Action sequencing, timeout and conditional behavior, hook edge behavior, path runtime lifecycle, child mechanism output routing, sim model stepping, and control output application to `MotorHandle` are covered by `MechanismRuntimeTest`.
  - [x] Added focused `MechanismRuntimeTest` coverage for sequence timing, timeout/conditional transitions, rising-edge hook behavior, path runtime lifecycle, child mechanism output routing, output application through fake `MotorHandle`s, `MechanismScheduler` dispatch through a `HardwareGraph`, and in-memory sim model stepping.
  - [x] Action arbitration is resource-scoped: disjoint motor requests coexist, newer conflicting activations win without per-tick renewal, releasing a held request restores older active intent, and newly unclaimed motors neutralize independently.
  - [x] Scheduler periodic runs hooks before action evaluation so cross-mechanism bindings take effect deterministically in the same cycle; constraints remain downstream of arbitration.
  - [x] Cover lowered scheduler subtree reset behavior with nested sequence coverage.
  - [x] Cover `Action.request()` dispatch and registered child-mechanism owner inference.
- [x] Upkeep: `Action`, `Output`, and `MechanismContext` now live in this module, and current Java sources are moved off old mechanism names.
  - [x] Removed `HookRunner` from the mechanism source/API surface.
- [x] Architecture: the old public mechanism `RobotRuntime` name is removed. The final public robot runtime is `ca.frc6390.athena.robot.RobotRuntime`; the mechanism slice exposes `MechanismScheduler` for root-runtime composition and keeps lower runtime/introspection/application classes package-private.
  - [x] Add graph ownership for mechanism nodes, declarations, and sim model discovery.
  - [x] Keep lifecycle/Action dispatch in `RobotRuntime` instead of separate runners.
  - [x] Allow `RobotRuntime` to use a graph-backed hardware context while preserving explicit registered test handles.
  - [x] Add request dispatch so robot code can call `action.request()` while runtime routing remains graph-owned.
  - [x] Expand composable `SimModel` and `SimModel.Source` declarations through the graph, including automatically contributed production kinematics models.
  - [x] Shared action instances across child mechanisms now become ambiguous direct request targets instead of blocking registration; declaration-targeted requests still route normally.
  - [x] Final cross-module root ownership moved to `athena-robot`.
  - [x] Replace public `MutableBoolean` with package-private `SimDigitalInput`; public digital input binding now accepts `BooleanSupplier`.

## athena-drivetrain

- [x] Architecture correction: removed the swerve reflection/introspection path. Swerve follows the Rebuilt model: drive code is a `Mechanism`, modules are mechanisms, and drivetrain behavior composes normal Actions instead of mounting a separate drive runtime.
- [x] Runtime cleanup: removed the separate swerve runtime, module node graph, root drivetrain mount, and direct `MotorHandle` write path. Swerve control belongs in mechanism Action composition.
- [x] Template API: restored reusable module presets through generic mechanism-core `MechanismTemplate` and `Slot` APIs. `SwerveModules` now consumes that pattern, but slots are not a drivetrain-only concept.
- [x] Test surfaces needed: production surfaces now exist for drivetrain behavior tests to target after the repo-wide test reset.
  - [x] `TrackWidth`, `WheelBase`, and `SwerveModuleModel` validate finite/positive conversion inputs.
  - [x] `SwerveModulesTest` covers preset availability and filling template slots into runtime controls.
  - [x] WPILib sim tests cover root-discovered `SwerveKinematics` moving `SimulationSession.pose()` through `RobotRuntime.simulationPeriodic(...)` without a swerve-specific sim descriptor.
  - [x] Runtime graph/introspection tests were removed with the deleted runtime path.
- [x] Upkeep: the old module location ref, swerve drive marker/definition/node API, `SwerveModuleOrder`, `Drivetrains`, all drivetrain `*Config`, all drivetrain `*Spec`, and differential drivetrain public API were removed. The module dropped its stale Gradle API dependency on `athena-commands`.

## athena-vision

- [x] Code optimization: the camera device/signal layer remains small immutable declaration wrappers, and hot-path reads can now be routed through `VisionGraph` instead of every consumer pulling suppliers directly.
- [x] Runtime optimization: added `VisionGraph` with per-camera cached pose and target signal views so robot-period code can refresh once and reuse cached measurements.
  - [x] Add `VisionGraph` runtime cache for camera declarations.
  - [x] Add cached `PoseSignal` and `TargetSignal` views from graph-owned camera runtimes.
  - [x] Preserve source signal metadata through cached signal wrappers.
  - [x] `athena-robot` owns when `VisionGraph.refresh()` runs during the root periodic cycle.
- [x] Test surfaces needed: camera kind and name defaults, mount immutability, supplier null handling, and `VisionGraph` cached pose/target signal behavior are covered by focused JUnit tests.
  - [x] `RobotRuntimeTest` covers root graph mounting/refresh ownership for vision graphs.
  - [x] `RobotRuntimeTest` covers hardware vendor `ServiceLoader` discovery from root test runtime classpath descriptors and root mounting/refresh of Limelight/PhotonVision camera declarations.
- [x] Upkeep: current Java sources are moved off old camera `*Ref` types and frame DTOs. Future examples should stay on `*Device`, `PoseSignal`, `TargetSignal`, and graph-cached signals.
- [x] Package cleanup: camera declarations now live in `vision.device`; pose and target streams now live in `vision.signal`; the old `vision.ref` package was removed.
- [x] Architecture: vision now declares camera devices and typed signals, and has a `VisionGraph` cache boundary for runtime-owned measurements.
  - [x] Add pose/target signal metadata hooks for vendor strategy/runtime policy.
  - [x] Graph mounting is covered through `RobotRuntime.vision(...)` and `RobotRuntime.cameras(...)`.
  - [x] Add `CameraAdapter`/`CameraAdapters` ServiceLoader SPI for opt-in automatic vendor camera binding; Limelight and PhotonVision publish descriptors and root tests verify discovery.
  - [x] Camera adapter discovery skips unavailable provider classes so absent PhotonVision/Limelight runtime libraries do not break projects that only installed the Athena vendordep.

## athena-localization

- [x] Code optimization: localization no longer reflects into package-private measurement records; pose extraction uses the typed runtime `PoseMeasurementSample` contract.
  - [x] Add runtime-level typed pose measurement sample accessors.
  - [x] Convert `PoseSamples` to direct `instanceof PoseMeasurementSample` extraction.
  - [x] Confirm owned source has no reflection-based pose extraction left.
- [x] Runtime optimization: `athena-robot` owns when localization pipelines evaluate during the robot periodic cycle.
  - [x] Pose measurement filtering now uses typed sample access instead of reflective calls.
  - [x] Root runtime scheduling now decides when pipelines evaluate.
- [x] Test surfaces needed: input chaining, nested pipeline estimates, filter rejection, field-bounds filtering, weighted-average behavior, latest-valid behavior, reset actions, and behavior when no pose-capable measurements exist are covered by focused JUnit tests.
  - [x] Root `RobotRuntime` now owns localization max-age filtering and disabled-mode refresh policy for localization snapshots.
- [x] Upkeep: removed the old public config/spec layer, old localization contexts, `LocalizationRef`, `LocalizationFilterRef`, `LocalizationEstimatorRef`, `FieldBoundsRef`, and stale vision frame estimator files. Current Java sources use uniform `PoseSignal -> Localization -> Localization` composition with `Localizations`, `LocalizationFilter`, and `FieldBoundsFilter`.
- [x] Package cleanup: localization APIs now live in `localization.pipeline`; the old `localization.ref` package was removed.
- [x] Architecture: localization now consumes a runtime-level pose sample contract instead of reflecting into concrete measurement records.
  - [x] Replace temporary reflective bridge with `PoseMeasurementSample`.
  - [x] Final graph ownership keeps localization inputs as generic `MeasurementSignal`s with typed pose-capable samples for now.

## athena-simulation

- [x] Overall: finish simulation runtime integration with the final hardware graph.
- [x] Code optimization: sim handles are small mutable live objects and the old public sim DTO layer remains removed.
- [x] Runtime identity: simulated handles key motor and IMU reuse by `HardwareIdentity`, so CAN bus/category/kind/id collisions do not collapse into default-name lookups.
- [x] Runtime handle reuse: `SimulationSession` coordinates cached handles for the same stable identity.
- [x] HardwareGraph integration: `SimulationSession` supplies a `HardwareGraph` backed by the same simulated handles that model stepping advances.
- [x] Test surfaces needed: motor percent/voltage/position/velocity behavior, timestep integration, IMU yaw-rate stepping, backend kind matching, and runtime handle reuse are covered by simulation tests.
  - [x] Simulation tests cover runtime handle reuse, registered model materialization, motor percent/voltage/position/velocity behavior, timestep integration, IMU yaw-rate stepping, backend kind matching through `HardwareGraph`, and graph-backed integrated encoder reads.
  - [x] `SimulationSessionTest` verifies every supported real sim motor, encoder, and IMU kind resolves through `SimulationSession.hardwareGraph()` with no public `SIM` kind.
  - [x] Root runtime tests verify mechanism-declared `SimModel` fields are registered into `SimulationSession` and stepped through `RobotRuntime.simulationPeriodic(...)`.
- [x] Upkeep: standalone sim backend classes remain ServiceLoader SPI entries for explicit simulation backend registration. Session-owned simulation uses private nested backends through the simulation hardware graph.
- [x] Architecture: narrowed `athena-simulation` Gradle dependencies by removing unused direct dependencies on `athena-mechanisms` and `athena-vision`.

## athena-vendor-ctre

- [x] Code optimization: CTRE handle code is direct and small after the `*Handle` rename.
- [x] Runtime optimization: Phoenix status reads are now snapshot-backed through `refreshInputs()`, and `RobotRuntime` calls `HardwareGraph.refreshInputs()` once per periodic cycle before graph consumers read handles.
- [x] Test surfaces needed: CTRE fakeable backend tests cover kind support and option sanitization.
  - [x] Cover supported motor kinds and equivalent keys.
  - [x] Cover CANcoder encoder kind support and non-CTRE rejection.
  - [x] Cover Pigeon2 IMU kind support and non-CTRE rejection.
  - [x] Cover CTRE vendor option sanitization.
  - [x] Cover graph-activation timing for neutral-mode setup.
  - [x] Cover cached CTRE motor, CANcoder, and Pigeon input snapshots with fake controllers.
  - [x] Remaining real neutral mode/current behavior, closed-loop calls, CANcoder reads, and Pigeon reset/yaw behavior are robot-hardware test gates, not local fakeable unit tests.
- [x] Upkeep: CTRE options are public declaration metadata with focused sanitization coverage; final package policy is documented as declaration metadata.
- [x] Architecture: backend interfaces remain public intentionally as vendor ServiceLoader SPI; runtime ownership is in `HardwareGraph`, which now owns activation, refresh, caching, and close policy for created handles.

## athena-vendor-rev

- [x] Code optimization: Spark handle logic is contained, but repeated encoder accessor calls should be reviewed once real hardware behavior is covered.
- [x] Runtime optimization: Spark configuration moved from construction to `MotorHandle.activate()`, and `HardwareGraph` owns one-time activation for cached handles.
- [x] Test surfaces needed: REV fakeable backend tests cover kind support and option sanitization.
  - [x] Cover Spark MAX/Flex and equivalent motor kind support.
  - [x] Cover through-bore encoder kind support.
  - [x] Cover idle/ramp/current option sanitization inputs that can be tested without hardware.
  - [x] Cover through-bore absolute-position delegation and unsupported velocity behavior with a fake controller.
  - [x] Cover graph-activation timing for Spark configuration.
  - [x] Remaining real idle mode/current/ramp application and integrated/absolute encoder unit behavior are robot-hardware test gates, not local fakeable unit tests.
- [x] Upkeep: through-bore now uses `EncoderDevice.dioChannel()` as an explicit roboRIO DIO declaration boundary instead of treating generic `id()` as a channel.
- [x] Architecture: motor-attached absolute encoders are now modeled explicitly as `MotorDevice.absoluteEncoder()` declarations resolved by `HardwareGraph` through the cached motor handle.

## athena-vendor-studica

- [x] Code optimization: Studica IMU handle is simple and matches `ImuHandle`.
- [x] Runtime optimization: real AHRS construction is lazy and graph-activated through `ImuHandle.activate()`, with `HardwareGraph` owning cached handle lifecycle and close policy.
- [x] Test surfaces needed: Studica fakeable backend tests cover kind support and injected handle creation.
  - [x] Cover NavX kind support and rejection of non-Studica IMU kinds.
  - [x] Cover backend handle factory injection for non-hardware tests.
  - [x] Cover named NavX port declarations through `StudicaNavxPort` and `StudicaImus.navx(...)`.
  - [x] Cover activation plus yaw/angle/reset/zero/close delegation through a fake NavX controller.
  - [x] Remaining real AHRS construction and physical port behavior are robot-hardware test gates, not local fakeable unit tests.
- [x] Upkeep: NavX port selection now has `StudicaNavxPort` and `StudicaImus.navx(...)` helpers instead of requiring raw numeric ids.
- [x] Architecture: common IMU extras are represented on the generic `ImuHandle` contract (`angleDegrees`, `zeroYaw`, `reset`) and Studica implements them through that contract.

## vendor dependency model

- [x] The single `FRC6390-Athena.json` vendordep includes the core Athena stack and all Athena vendor adapter artifacts.
- [x] Athena vendor adapter artifacts declare third-party vendor libraries as compile-only, so Athena does not transitively install CTRE, REV, PhotonVision, PathPlanner, Choreo, Studica, or Limelight runtime libraries.
- [x] Teams keep using the normal third-party vendordeps for hardware/tools they actually use; Athena adapter discovery activates only when those libraries are present.

## athena-commands

- [x] Code optimization: the module is now two small immutable behavior-declaration types, `CommandAction` and `CommandActionBuilder`; the deleted drive helpers and command grouping utilities no longer carry duplicated command composition logic.
- [x] Runtime optimization: command execution is centralized in `CommandGraph`, so lifecycle and requirement checks are owned by one small graph instead of repeated by each adapter.
- [x] Test surfaces needed: command descriptor behavior and local requirement arbitration are covered by `CommandActionTest`.
  - [x] Cover default callback behavior.
  - [x] Cover requirement de-duplication and trimming.
  - [x] Cover finish condition behavior and null handling.
  - [x] Cover `CommandGraph` lifecycle ownership, command replacement, conflict cancellation, finish cleanup, and requirement release.
- [x] Upkeep: current Java sources are moved off `CommandSpec`, `CommandGroups`, and `RobotDriveCommands`. Rebuilt examples should use `CommandAction` plus `CommandGraph` only where Athena needs behavior descriptors and local arbitration.
- [x] Architecture: `athena-commands` now exposes small behavior descriptors plus a requirement-arbitration graph. It does not own drivetrain command helpers or WPILib adapter semantics.

## athena-auto

- [x] Code optimization: the module is now a small indexed routine set instead of the old chooser/config/registry chain; this removes global lookup and duplicate DTO construction.
  - [x] Keep routine indexing normalized and duplicate-checked in `AutoRuntime`.
  - [x] Keep path-backed routine creation lazy through `Autos.path(...)`.
- [x] Runtime optimization: `AutoRuntime` creates the selected `CommandAction` lazily and reuses it until selection/reset, and `athena-robot` mounts it into the root lifecycle.
  - [x] Add lifecycle ownership methods: `initialize()`, `execute()`, `periodic()`, `isFinished()`, `end(...)`, and `reset()`.
  - [x] End the active command when selection/reset interrupts the current routine.
  - [x] Add `PathGraph` marker-command lifecycle ownership for path events.
  - [x] `CommandGraph` now owns local command requirement arbitration and cancellation policy.
  - [x] Final robot runtime mounts `AutoRuntime`, `PathGraph`, and `CommandGraph` into one lifecycle.
- [x] Test surfaces needed: routine name normalization, duplicate detection, selected routine fallback, reset behavior, provider-backed path routines, marker binding validation, marker dispatch, and lifecycle behavior when integrated with `RobotRuntime`.
  - [x] Added `AutoRuntimeTest` coverage for routine normalization, duplicate detection, first-routine selection, provider-backed routine loading/reset, marker validation, local command lifecycle dispatch, `PathGraph` marker dispatch, duplicate marker rejection across routines, and active marker ending.
  - [x] `athena-robot` adds integrated root scheduling coverage for autos and command cancellation.
- [x] Upkeep: old `AutoChooserConfig`, `AutoChooserSpec`, `AutoRoutineConfig`, `AutoRoutineSpec`, `AutoRegistry`, `AutoSource`, `AutoExecution`, input store/scope, and missing-source exception surfaces are gone from disk. Vendor path modules now use `PathProvider` and `Paths.*` entry points.
- [x] Architecture: autos now express selected autonomous work as named command Actions, path providers, validated marker metadata, and root-mounted runtime lifecycle.
  - [x] Add `PathMarkerBinding` validation through `AutoRoutine`.
  - [x] Widen `PathProvider` so providers expose `PathAction`, `CommandAction`, and `PathRuntime` from one boundary.
  - [x] Add `PathGraph` as the executable marker-dispatch boundary for routine marker bindings.
  - [x] `AutoRuntime` is mounted into the final root `RobotRuntime` in `athena-robot`.

## athena-dashboard

- [x] Code optimization: removed the public dashboard packet, codec, publisher, TCP server, and control registry surface instead of retaining unused transport code.
- [x] Runtime optimization: no runtime dashboard work remains. Future dashboard support should attach as a `Sink` owned by runtime plumbing, not as a separate student-facing API.
- [x] Test surfaces needed: no dashboard tests are needed while the module is absent from the public 2027 API.
  - [x] Future coverage is intentionally deferred unless dashboard support returns through a final sink boundary.
- [x] Upkeep: old dashboard examples and module sources are deleted.
- [x] Architecture: dashboard is not part of the current 2027 functional API. Reintroduce it only through the final source/sink boundary after `RobotRuntime` exists.

## athena-telemetry

- [x] Code optimization: removed the public telemetry registry, key/value DTOs, sink, and NetworkTables writer wrappers instead of maintaining a second output system beside runtime signals.
- [x] Runtime optimization: no telemetry publication loop remains. Future telemetry should consume runtime-owned graph data and publish through a sink boundary.
- [x] Test surfaces needed: no telemetry tests are needed while the module is absent from the public 2027 API.
  - [x] Future coverage is intentionally deferred unless telemetry returns through a final sink boundary.
- [x] Upkeep: old telemetry examples and WPILib NetworkTables adapters are deleted.
- [x] Architecture: telemetry is intentionally absent from the public 2027 surface. The correct replacement is a runtime `Sink`, not a standalone registry module.

## athena-superstructure

- [x] Code optimization: removed the public superstructure config/spec/controller/planner stack. That eliminates duplicated Action planning over mechanisms while `Actions` and `RobotRuntime` become the composition model.
- [x] Runtime optimization: no superstructure planner/runtime work remains. Coordinated behavior should become Action composition inside the runtime graph rather than a second scheduler.
- [x] Test surfaces needed: no superstructure tests are needed for the deleted API.
  - [x] Future replacement coverage belongs around composed `Action` execution, path/Action sequencing, and runtime conflict resolution after the runtime graph is final.
- [x] Upkeep: old superstructure examples are deleted. Rebuild coordination examples through composed `Action` behavior only.
- [x] Architecture: superstructure is no longer a module-level public API. Rebuilt-style subsystem coordination should be expressed through `Actions`, bindings, and the future `RobotRuntime`.

## athena-robot

- [x] Worker boundary: `RobotRuntime` now has optional `RuntimeWorker` / `RuntimeWorkers` configuration. The default path starts no background work; inline workers run from the main periodic loop and async workers require a caller-owned scheduler.
- [x] Signal sampling: mechanism runtimes now cache declared digital inputs and sample them once before hook evaluation. Manual `sampleSignals()` can be used by a faster worker to latch short pulses between normal periodic ticks.
- [x] Control execution: `ControlBinding` loop runtimes execute through the normal Action output path, receive runtime `dtSeconds`, reset on first use and target changes, and compose PID/feedforward outputs into one applied request.
- [x] Hardware fault visibility: `RobotRuntime.hardwareRefreshFailures()` exposes the latest `HardwareGraph` refresh failures so root-level tests and robot code can observe vendor read faults without aborting the runtime loop.
- [x] Vision sim field metadata: `SimulationSession.visionField(...)` now feeds discovered vision simulation providers through Athena-owned `VisionSimulationField` / `VisionSimulationTarget` declarations, and PhotonVision simulation binds both target and pose measurements without exposing WPILib or Photon classes to robot code.
- [x] Runtime safety: simulation stepping rejects non-finite or non-positive timesteps before physics engines run, worker failure callbacks run outside the worker lock, and hardware refresh snapshots handles before vendor calls so graph mutation is not blocked by slow I/O.
- [x] Naming cleanup: public command/path behavior declarations are now `CommandAction` and `PathAction`; the previous public state-named command/path declarations were removed.
- [x] Digital input runtime scoping: `SimulationSession` owns a scoped DIO reader/signal runtime so sessions can simulate the same real DIO declaration independently, while manual `DigitalInputDevice.bindRuntime(...)` remains available for non-session tests.
- [x] Digital edge durability: latched edge cleanup now clears only the edge generation observed by hooks, so fast samples arriving during hook execution are not erased by end-of-hook cleanup.
- [x] Hot-path allocation pass: root mechanism runtime now appends into one output list instead of allocating one output list per mechanism, hook event/source maps are reused per `HookRuntime`, grouped scheduler nodes reuse per-node output scratch lists, and control request reset tracking no longer allocates key records per resolved control output.
- [x] Benchmarking: `:athena-benchmarks:jmhGc` runs the existing JMH suite with GC allocation profiling for allocation-regression checks.
- [x] Control slots: `ControlBinding.slot(...)` now carries the motor-controller slot through the normal action/control path into vendor-neutral closed-loop requests.
- [x] Package cleanup: control gains now live in `mechanism.control`; the old `mechanism.ref` package was removed.

- [x] Code optimization: added the root `RobotRuntime` as a thin composition layer instead of forcing cross-module ownership into `athena-mechanisms`.
- [x] Runtime optimization: root periodic owns vision refresh, localization evaluation, command graph scheduling, autonomous lifecycle dispatch, disabled cancellation, and mechanism output application; simulation periodic advances physics without evaluating mechanisms again.
- [x] Test surfaces needed: `RobotRuntimeTest` covers root command/autonomous lifecycle ownership, disabled cancellation, and simulation-backed hardware graph exposure.
- [x] Upkeep: `athena-robot` is included in settings and published artifacts so WPILib and robot projects can depend on the final root boundary; it is intentionally not in the default minimal artifact set because it composes optional modules.
- [x] Architecture: `athena-robot` is the cross-module runtime owner for hardware, mechanisms, drivetrain, vision, localization, autos, path markers, commands, and simulation.

## athena-wpilib

- [x] Code optimization: the module contains the WPILib robot host, controller signals, command bridging, and simulation physics adapter; localization estimation lives in `athena-localization` behind `Localizations.kalman()`.
- [x] Runtime optimization: `AthenaRobot` dispatches robot behavior through normal WPILib lifecycle callbacks and uses simulation callbacks only to step the `SimulationSession`. Package-private lifecycle dispatch keeps deterministic tests out of the public API.
- [x] Test surfaces needed: WPILib lifecycle-to-Athena lifecycle mapping, dt calculation/reset behavior, simulation callback behavior, controller signals, and runtime host construction policy are covered or explicitly scoped.
  - [x] Added focused `AthenaRobotTest` coverage for lifecycle mapping, simulation callback flags, and negative/non-finite dt clamping through the package-private lifecycle delegate.
  - [x] Added direct WPILib parity coverage in `athena-localization` for odometry, vision correction, delayed samples, and resets.
  - [x] Direct `TimedRobot` host construction is intentionally scoped to robot-style integration fixtures because WPILib robot-base construction terminates the Gradle test worker in this environment.
- [x] Upkeep: `AthenaRobot` now imports `RobotRuntime` from `athena-robot`, not the mechanism slice.
- [x] Architecture: WPILib is now an adapter/host boundary, not a place for command scheduling, telemetry, controller binding, or drivetrain runtime logic.

## athena-vendor-photonvision

- [x] Code optimization: the module is down to PhotonVision target values and a camera adapter that feeds the new vision device/signal layer.
- [x] Runtime optimization: target conversion now uses a small adapter buffer for the live target signal, and `VisionGraph` can cache the resulting measurements for robot-period consumers.
  - [x] Add buffered PhotonVision target signal path for configured adapters.
  - [x] Emit typed `TargetMeasurementSample` target records for downstream target parsing.
  - [x] Add buffered PhotonVision pose signal path for adapters configured with a PhotonLib `PhotonPoseEstimator`.
- [x] Test surfaces needed: target field mapping, ambiguity handling, pose-signal metadata, empty/null target conversion, timestamp propagation through the fakeable result seam, pose production, and unconfigured adapter behavior are covered by focused JUnit tests.
  - [x] PhotonLib result conversion now flows through adapter-owned result/pose values so timestamp and pose handling can be faked without constructing PhotonLib results in tests.
- [x] Upkeep: old tests are removed; rebuild them around `PhotonVisionDevice`, `TargetSignal`, and future `PoseSignal` support rather than `PhotonRef`/frame DTOs.
- [x] Architecture: PhotonVision is an implementation adapter behind `Cameras.photonVision`, not a standalone public catalog.
  - [x] Target measurements now use the shared runtime target-sample contract.
  - [x] PhotonVision pose measurements now use the shared runtime pose-sample contract through the pose-estimator adapter path.

## athena-vendor-limelight

- [x] Code optimization: the module keeps Limelight target values and adapter logic only; old ref/config assumptions are gone.
- [x] Runtime optimization: NetworkTables reads now go through a `LimelightFrame` snapshot path, and the common `bind(...)` path shares a short-lived frame cache between pose and target suppliers.
  - [x] Centralize Limelight target and botpose reads into one frame snapshot.
  - [x] Add shared frame-backed pose/target supplier for bound devices.
  - [x] Emit typed `PoseMeasurementSample` and `TargetMeasurementSample` records.
  - [x] `VisionGraph`/root runtime owns the exact refresh cadence; the bound adapter now reuses the pose-read frame for the paired target read without a time-window cache.
- [x] Test surfaces needed: target parsing/conversion, latency handling, no-target behavior, pose conversion, and Limelight pose metadata are covered by focused JUnit tests without real NetworkTables.
  - [x] Bound-device frame sharing through `VisionGraph.refresh()` is covered with a fake Limelight client.
  - [x] Real NetworkTables table parsing is scoped to robot/integration coverage because the local adapter logic is covered through `LimelightFrame` conversion tests.
- [x] Upkeep: old tests are removed and need to be rebuilt against `LimelightDevice`, `PoseSignal`, and `TargetSignal`.
- [x] Architecture: Limelight support belongs behind the vision graph and camera device boundary, not as a separate robot-facing ref layer.
  - [x] Limelight pose and target outputs now use shared runtime measurement contracts.
  - [x] Root runtime mounts bound Limelight devices through `VisionGraph`; adapter construction stays behind device binding, not robot-facing runtime code.

## athena-vendor-pathplanner

- [x] Code optimization: the old `PathPlannerAutoSource`/`PathPlannerAutos` public surface has been replaced by `PathPlannerPathProvider`, which implements the widened `PathProvider`.
  - [x] Cache normalized `PathAction` objects by provider path name.
  - [x] Cache discovered PathPlanner auto names after the first vendor query.
- [x] Runtime optimization: the provider caches active WPILib commands inside its path runtime by `PathAction.key()`.
  - [x] Keep command creation scoped to command Action/runtime lifecycle instead of a global auto registry.
  - [x] Keep active runtime commands keyed by `PathAction.key()`.
  - [x] Added focused fake-command coverage proving `runtime()` reuses the active command until `end(...)`.
  - [x] Remaining lifecycle ownership is now tracked under the final root `RobotRuntime` mounting item rather than this provider-local cache.
- [x] Test surfaces needed: missing path handling, provider name normalization, `Paths.pathPlanner` conversion, `CommandAction` lifecycle wrapping, marker bindings, and interaction with `PathRuntime`.
  - [x] `athena-auto` provider-backed routine tests cover the shared `PathProvider` routine boundary and `Paths.pathPlanner` conversion through a fake provider.
  - [x] `PathPlannerPathProviderTest` covers provider name normalization, path-Action caching, PathPlanner name discovery caching, missing-path failure propagation, `CommandAction` lifecycle wrapping, provider-backed marker bindings through `PathGraph`, and `PathRuntime` fake-command interaction through a fake `PathPlannerClient`.
  - [x] Real PathPlannerLib missing-file behavior is intentionally delegated to `AutoBuilder.buildAuto`; the provider boundary is covered for propagation, and executable marker ownership is covered at `PathGraph`.
- [x] Upkeep: the old lowercase `Paths.pathplanner` entry point was removed; downstream code must use `Paths.pathPlanner`.
- [x] Architecture: PathPlanner is a `PathProvider` adapter. It does not own autonomous registry or chooser semantics.
  - [x] `PathPlannerPathProvider` exposes `path(...)`, `load(...)`, and `runtime()` through the common provider boundary.
  - [x] Marker command dispatch is owned by `PathGraph`; provider-specific marker extraction remains a vendor-fixture test gate.

## athena-vendor-choreo

- [x] Code optimization: the old Choreo auto source/catalog/factory names are replaced by `ChoreoPathProvider` and `ChoreoPathAdapter`.
  - [x] Cache normalized `PathAction` objects by provider path name.
  - [x] Cache trajectory lookup results and discovered trajectory names in `ChoreoPathProvider`.
- [x] Runtime optimization: `ChoreoPathAdapter` caches active WPILib commands inside its path runtime by `PathAction.key()`.
  - [x] `ChoreoPathAdapter` now implements the widened `PathProvider` through `path(...)`, `load(...)`, and `runtime()`.
  - [x] Keep trajectory command runtime and routine command runtime separate.
  - [x] Added focused fake-command coverage proving trajectory and routine runtimes reuse active commands until `end(...)`.
  - [x] Remaining marker extraction is a vendor-fixture test gate; executable marker dispatch is owned by `PathGraph`.
- [x] Test surfaces needed: trajectory lookup, marker conversion, missing trajectory behavior, command lifecycle, split trajectory delegation, routine command delegation, and path Action integration.
  - [x] Focused compile verification covers the widened provider contract against current ChoreoLib/WPILib APIs.
  - [x] `ChoreoPathProviderTest` covers provider path normalization/caching, trajectory lookup caching, discovered-name caching, marker extraction/conversion into `PathMarkerBinding`, missing trajectory marker behavior, fake `FactoryClient` delegation for split trajectory/reset/routine/warmup commands, `CommandAction` wrapping, and path runtime behavior.
  - [x] Marker command dispatch stays owned by `PathGraph`; Choreo conversion only extracts trajectory event names and binds supplied command Actions.
  - [x] Real Choreo filesystem-loader missing-path coverage is not kept in the unit suite because it crashes the Gradle test worker in this environment; provider-boundary missing behavior is covered with the injectable client.
- [x] Upkeep: the old `ChoreoAutos` public catalog is gone; entry points run through `Paths.choreo` and the provider/adapter pair.
- [x] Architecture: Choreo support is a provider/adapter behind the final path API, not an autonomous subsystem.
  - [x] Executable Choreo paths now use the common `PathProvider` boundary.
  - [x] Pose reset and marker extraction remain provider contract details; marker command dispatch is owned by `PathGraph`.

## athena-helios

- [x] Code optimization: deleted the old uncompiled HeliOS camera provider/config/adapter source because it depended on removed `ca.frc6390.athena.sensors.camera` APIs.
- [x] Runtime optimization: no HeliOS-specific runtime code remains in the active build. HeliOS camera declarations now belong to `athena-vision` through `HeliosDevice`.
- [x] Test surfaces needed: none in this excluded module; future HeliOS support should test `HeliosDevice` binding and pose/target signals through the same vendor-adapter pattern as Limelight and PhotonVision.
- [x] Upkeep: keep the old module out of `settings.gradle`; reintroduce only as `athena-vendor-helios` if final `CameraDevice`/`PoseSignal` support is needed.
- [x] Architecture: old HeliOS config/provider API is gone. The final shape should be a vision adapter behind `Cameras.helios`, not a parallel camera registry.

## athena-arcp

- [x] Code optimization: ARCP still contains useful Java JNI, Rust server/dashboard, and host-dashboard source, but it is not included by `settings.gradle`, not listed in `athenaPublishedArtifacts`, and not present in the vendordep. Keep the source as isolated tooling while the robot API cleanup continues.
- [x] Runtime optimization: no ARCP runtime participates in the active 2027 Java build. Re-evaluate realtime signal handling only if a future `Signal`/`Sink` boundary mounts ARCP as an optional transport.
- [x] Test surfaces needed: no Gradle test surface applies while ARCP is outside the active 2027 deliverable. If it is reintroduced, restore focused checks for JNI load behavior, protocol encode/decode, realtime loop timing, dashboard integration, and transport reconnects.
- [x] Upkeep: Markdown docs were removed from the active tree. Do not rebuild ARCP docs for the 2027 library deliverable unless ARCP is intentionally reintroduced as a published optional tooling artifact.
- [x] Architecture: ARCP is isolated tooling/transport, not a student-facing robot API layer. It should stay outside `RobotRuntime` until the final runtime exposes a deliberate external sink/transport extension point.

## example-projects

- [x] Code optimization: old example Java sources were removed because they preserved deleted V3/spec/ref APIs.
- [x] Runtime optimization: examples now run as a standalone GradleRIO workspace instead of an Athena root submodule.
- [x] Gradle status: `example-projects` contains `blank`, `tank-drive`, and `swerve-drive`.
- [x] Test surfaces needed: `./gradlew.bat -p example-projects build` compiles the example workspace against the published local Athena snapshot.
- [x] Upkeep: keep examples focused on current final API usage.
- [x] Architecture: examples should demonstrate final API usage, not act as a compatibility layer for removed concepts.

## Overall API Issues

- [x] Overall task: re-evaluate API-wide issues after each module finishes its checklist.
- [x] Compile health: owned focused Gradle module checks compile around final names.
  - [x] `:athena-runtime:compileJava` passed with rerun.
  - [x] `:athena-mechanisms:compileJava` passed.
  - [x] `:athena-vision:compileJava`, `:athena-localization:compileJava`, `:athena-vendor-photonvision:compileJava`, and `:athena-vendor-limelight:compileJava` passed with rerun after the vision/localization pass.
  - [x] `:athena-auto:test`, `:athena-vendor-pathplanner:compileJava`, and `:athena-vendor-choreo:compileJava` passed with rerun after the auto/path provider pass.
  - [x] Full `./gradlew.bat check` passes after completing the module checklist and overall API pass.
- [x] Naming/package health: package names now match the major concept boundaries covered in this pass.
  - [x] Split the old `hardware.ref` package into `hardware.device`, `hardware.runtime`, and `hardware.sim`.
- [x] Runtime ownership: cross-module runtime ownership now has a root in `athena-robot`.
  - [x] Mechanism slice now has an internal `RobotGraph` and cached `MechanismNode` ownership.
  - [x] Mechanism lifecycle/Action dispatch stays behind `RobotRuntime`.
  - [x] Vision slice now has a `VisionGraph` cache boundary and typed pose/target sample contracts for localization/vendor adapters.
  - [x] `athena-robot` binds hardware graph, drivetrain runtime, localization scheduling, paths, autos, commands, sim, and lifecycle together.
- [x] Reflection/discovery: reflection remains the declaration discovery mechanism, but periodic loops are not discovery-heavy.
  - [x] Mechanism and hook reflection results are cached at runtime/graph construction.
  - [x] Field-list reflection is cached for mechanism, hook, and path introspection.
  - [x] Localization pose extraction no longer uses reflection.
  - [x] Mechanism `StateScheduler` now lowers Action trees into cached scheduler nodes and no longer uses generated string path keys for runtime node identity.
- [x] Path, command, and auto integration has a working local boundary and root graph ownership.
  - [x] `PathProvider` now exposes path Action creation, command-Action loading, and path runtime creation from one boundary.
  - [x] `AutoRuntime` owns selected command-Action lifecycle locally.
  - [x] `PathGraph` owns executable marker-command dispatch for validated routine marker bindings.
  - [x] PathPlanner and executable Choreo adapters implement the common provider boundary.
  - [x] Auto routines validate marker binding metadata.
  - [x] `athena-robot` mounts auto/path/command lifecycle ownership into the final `RobotRuntime`.
- [x] Tests need to be rebuilt from behavior outward.
  - [x] Added focused auto behavior tests for routine indexing, provider-backed routines, marker validation, and local lifecycle dispatch.
  - [x] Added focused mechanism runtime, drivetrain fake-handle/runtime-construction, and WPILib pose-estimator adapter behavior tests.
  - [x] Added stable declaration/kind tests, graph lowering tests, fake-handle runtime behavior tests, vendor adapter contract tests, root `RobotRuntime` integration tests, and WPILib lifecycle host tests.
- [x] The remaining cleanup risk is rebuilding enough real behavior on top of the cleaned surface.
  - [x] `athena-helios` is removed from the active build; `example-projects` is a separate acceptance workspace instead of a compatibility layer.
  - [x] Full `./gradlew.bat check` covers architecture boundaries, legacy eviction, public API Javadocs, vendordep/release metadata, vendor metadata, vendor ServiceLoader descriptors, and all active module tests.
