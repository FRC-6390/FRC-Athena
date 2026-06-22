# Athena API V3 Direction

Status: accepted V3 direction, implementation active in the root workspace

V3 makes Athena feel like one library to students while keeping the codebase
split into clean feature artifacts internally.

The desired user experience is:

```text
Install one Athena vendordep.
Install the vendor libraries the robot actually uses.
Athena detects those vendors.
Only matching Athena feature artifacts are added to the robot project.
```

Java does not provide conditional imports, so V3 uses Gradle dependency
selection as the conditional-compilation boundary.

## Accepted Direction

These decisions are the V3 baseline. Remaining work is implementation and
coverage, not unresolved API direction:

- Athena is installed through one human-facing vendordep:
  `FRC6390-Athena.json`.
- That vendordep does not require every supported vendor library.
- The repo splits the current large core surface into tiered artifacts.
- Vendor integrations live in separate `athena-vendor-*` artifacts.
- The Athena Gradle plugin detects installed vendor dependencies and adds
  matching Athena vendor artifacts.
- Public Java declarations expose `toSpec()` as the normal lowering API.
- Validation supports both `spec.validate()` and `spec.validate(context)`.
- Annotation authoring waits until the regular Java API and spec model are
  settled.
- Vendor-specific escape hatches are allowed when the matching vendor artifact
  is present.
- The WPILib project template/bootstrap applies the Athena Gradle plugin;
  `FRC6390-Athena.json` provides Athena runtime/plugin artifacts but does not
  rely on vendordeps applying Gradle plugins automatically.
- `FRC6390-Athena.json` includes the default student-facing tiers:
  `athena-api`, `athena-runtime`, `athena-hardware`, `athena-mechanisms`,
  `athena-commands`, `athena-telemetry`, and `athena-plugin`.
- Drivetrain, superstructure, auto, vision, simulation, and vendor tiers are
  selected by plugin features or auto-detection.
- Vendor-specific options use typed Athena option extension classes,
  configured by lambdas. Direct vendor config objects are not part of the
  public generic API.
- `AthenaMotor` and similar common hardware enums include common built-in
  device keys even when the matching vendor artifact is absent.
- Third-party vendors publish Athena feature metadata as a resource in
  their adapter artifact and may also be configured explicitly in Gradle.
- Real WPILib glue lives in `athena-wpilib`, not in the default core tiers.
- Localization is a first-class optional artifact: `athena-localization`.
- ARCP/dashboard control does not move into the default robot runtime. It is a
  separate optional dashboard/control workspace with an adapter boundary back
  into telemetry and diagnostics.

## Implementation Scope

The V3 API direction is accepted. The the root workspace now owns the fluent Java
API, spec model, validation, plugin selection, default vendordep shape, WPILib
boundary, and current vendor adapter pass. Work outside that scope is not an
open architecture question:

| Area | Decision |
| --- | --- |
| Phoenix 5 motor controllers | Separate optional adapter artifact if needed; not part of the Phoenix 6 CTRE artifact. |
| Dashboard layout/UI schema | Separate dashboard/control workspace; the Java robot workspace owns packets, telemetry, diagnostics, and TCP transport only. |
| Annotation/state DSL | Deferred until fluent Java specs and runtime contracts settle. |
| Additional one-off vendor devices | Add as optional `athena-vendor-*` artifacts with metadata resources, not default dependencies. |

## Settled Follow-Up Decisions

These are direction-level decisions with implementation work tracked in the root workspace.

- GradleRIO bootstrap:
  Athena ships one vendordep and a Gradle plugin. The WPILib project template,
  Athena setup task, or user build file applies `id 'ca.frc6390.athena'`.
  Vendordeps are not expected to apply Gradle plugins.
- Vendor metadata loading:
  built-in and third-party adapters publish JSON metadata under
  `META-INF/athena/vendors/*.json`. The plugin loads all visible resources,
  merges them by feature key, rejects conflicting duplicate definitions, and
  lets explicit Gradle configuration override detection.
- Local conditional builds:
  repo-local feature builds are a developer convenience only. Student projects
  use normal Gradle dependency selection from the plugin; they do not need
  conditional Java source imports or conditional module inclusion.
- Vendor adapter packaging:
  every direct vendor integration lives in a separate publishable
  `athena-vendor-*` artifact. Generic Athena modules may expose stable vendor
  keys, but may not import vendor classes.
- WPILib adapter boundary:
  `athena-commands` and `athena-telemetry` own Athena-facing specs. Concrete
  WPILib command, scheduler, robot lifecycle, controller binding, and
  NetworkTables glue lives in `athena-wpilib` so default Athena artifacts stay
  free of WPILib implementation dependencies.
- Localization artifact:
  pose estimation, vision measurement weighting, slip detection, field bounds,
  and named autonomous pose aliases live in `athena-localization`. Vendor camera
  adapters feed observations through `athena-vision`; localization consumes
  those observations without importing vendor APIs.
- Dashboard/control boundary:
  ARCP remains outside the default robot runtime. A future
  dashboard/control workspace may consume telemetry and diagnostics APIs, but
  robot projects should not get dashboard server dependencies by installing the
  default Athena vendordep.
- Third-party vendor API:
  resource metadata is the preferred path. Explicit Gradle registration exists
  for local/private adapters and uses the same fields as the JSON resource.
- Annotation authoring:
  annotations are deferred. V3 first stabilizes fluent Java declarations,
  immutable specs, validation, and backend contracts; annotations can later
  lower into the same specs without changing the runtime model.

## Target Install Shape

Students install one Athena vendordep:

```text
FRC6390-Athena.json
```

That vendordep provides the default Athena runtime coordinates and the
Athena Gradle plugin artifact. The robot project template or Athena bootstrap
applies the Gradle plugin in `build.gradle`; the design must not depend on
WPILib vendordeps applying Gradle plugins automatically.

`FRC6390-Athena.json` does not force CTRE, REV, PhotonVision, PathPlanner,
Studica, or any smaller vendor dependency onto every robot.

Robot projects still install the actual vendor vendordeps they use:

```text
FRC6390-Athena.json
Phoenix6-frc2026-latest.json
REVLib-2026.json
```

The Athena Gradle plugin sees Phoenix and REV, then adds:

```text
ca.frc6390.athena:athena-vendor-ctre
ca.frc6390.athena:athena-vendor-rev
```

If a robot only installs Phoenix, only the CTRE adapter is added.

## Artifact Tiers

`athena-core` is too large to be the only base artifact. V3 splits it into
tiered artifacts with clear package boundaries.

Target Maven artifacts:

```text
ca.frc6390.athena:athena-api
ca.frc6390.athena:athena-runtime
ca.frc6390.athena:athena-hardware
ca.frc6390.athena:athena-mechanisms
ca.frc6390.athena:athena-superstructure
ca.frc6390.athena:athena-drivetrain
ca.frc6390.athena:athena-commands
ca.frc6390.athena:athena-auto
ca.frc6390.athena:athena-telemetry
ca.frc6390.athena:athena-vision
ca.frc6390.athena:athena-localization
ca.frc6390.athena:athena-simulation
ca.frc6390.athena:athena-wpilib
ca.frc6390.athena:athena-plugin

ca.frc6390.athena:athena-vendor-ctre
ca.frc6390.athena:athena-vendor-rev
ca.frc6390.athena:athena-vendor-studica
ca.frc6390.athena:athena-vendor-photonvision
ca.frc6390.athena:athena-vendor-limelight
ca.frc6390.athena:athena-vendor-pathplanner
ca.frc6390.athena:athena-vendor-choreo
```

Dependency graph:

```text
athena-api
  no WPILib vendor deps
  public declarations and common value types

athena-runtime
  depends on athena-api
  lifecycle, registries, backend discovery, validation

athena-hardware
  depends on athena-api, athena-runtime
  generic device specs, capabilities, hardware registries

athena-mechanisms
  depends on athena-api, athena-runtime, athena-hardware
  mechanism declarations, lowering, state/control behavior

athena-superstructure
  depends on athena-mechanisms

athena-drivetrain
  depends on athena-hardware, athena-commands

athena-commands
  depends on athena-runtime
  WPILib command integration

athena-auto
  depends on athena-commands, athena-runtime
  autonomous chooser declarations and auto source registry

athena-telemetry
  depends on athena-runtime

athena-vision
  depends on athena-api, athena-runtime
  generic camera declarations and target observations

athena-localization
  depends on athena-runtime, athena-vision
  pose estimation declarations, field bounds, vision weighting, slip detection

athena-simulation
  depends on athena-runtime, athena-hardware

athena-wpilib
  depends on athena-commands, athena-telemetry, athena-hardware
  WPILib command, scheduler, lifecycle, controller, and NetworkTables adapters

athena-vendor-*
  depends only on the Athena artifacts it implements plus matching vendor libs
```

The one vendordep references the default student-facing Athena artifacts that
every robot can safely use. The Gradle plugin adds optional feature artifacts
after it detects vendor libraries and enabled Athena features.

## Package Boundaries

The artifact split is visible in packages.

```text
ca.frc6390.athena.api
ca.frc6390.athena.runtime
ca.frc6390.athena.hardware
ca.frc6390.athena.mechanism
ca.frc6390.athena.superstructure
ca.frc6390.athena.drivetrain
ca.frc6390.athena.commands
ca.frc6390.athena.auto
ca.frc6390.athena.telemetry
ca.frc6390.athena.vision
ca.frc6390.athena.localization
ca.frc6390.athena.sim
ca.frc6390.athena.wpilib

ca.frc6390.athena.vendor.ctre
ca.frc6390.athena.vendor.rev
ca.frc6390.athena.vendor.studica
ca.frc6390.athena.vendor.photonvision
```

Rules:

```text
api must not depend on runtime.
runtime must not depend on vendor packages.
hardware must not import CTRE, REV, Studica, PhotonVision, etc.
mechanism code may depend on generic hardware specs, not vendor classes.
wpilib code may depend on WPILib, but default Athena tiers may not.
localization code may depend on Athena vision observations, not camera vendor classes.
vendor modules may depend on their vendor library and Athena backend contracts.
```

Additional vendor adapters follow the same `athena-vendor-*` shape when they
are added. A vendor is considered part of the current implementation set
only after a concrete adapter module, metadata file, tests, and example coverage
exist.

## Student Build Examples

Default robot build:

```groovy
plugins {
    id 'edu.wpi.first.GradleRIO'
    id 'ca.frc6390.athena'
}

athena {
    autoDetectVendors = true
}
```

Manual feature selection:

```groovy
athena {
    autoDetectVendors = false

    features {
        mechanisms()
        telemetry()
        commands()
    }

    vendors {
        ctre()
        rev()
    }
}
```

Minimal project:

```groovy
athena {
    features {
        hardware()
        telemetry()
    }
}
```

Default generated dependency result:

```text
athena-api
athena-runtime
athena-hardware
athena-mechanisms
athena-commands
athena-telemetry
athena-plugin
```

Generated dependency result for a CTRE-only robot:

```text
athena-api
athena-runtime
athena-hardware
athena-mechanisms
athena-commands
athena-telemetry
athena-plugin
athena-vendor-ctre
```

Generated dependency result for a no-vendor simulation/vision test project:

```text
athena-api
athena-runtime
athena-hardware
athena-mechanisms
athena-commands
athena-telemetry
athena-plugin
athena-auto
athena-vision
athena-simulation
```

## One Vendordep, Many Artifacts

`FRC6390-Athena.json` can be the only Athena vendordep while still exposing many
artifacts.

Conceptual vendordep shape:

```json
{
  "fileName": "FRC6390-Athena.json",
  "name": "FRC6390 Athena",
  "version": "2026.3.0",
  "javaDependencies": [
    {
      "groupId": "ca.frc6390.athena",
      "artifactId": "athena-api",
      "version": "2026.3.0"
    },
    {
      "groupId": "ca.frc6390.athena",
      "artifactId": "athena-runtime",
      "version": "2026.3.0"
    },
    {
      "groupId": "ca.frc6390.athena",
      "artifactId": "athena-hardware",
      "version": "2026.3.0"
    },
    {
      "groupId": "ca.frc6390.athena",
      "artifactId": "athena-mechanisms",
      "version": "2026.3.0"
    },
    {
      "groupId": "ca.frc6390.athena",
      "artifactId": "athena-commands",
      "version": "2026.3.0"
    },
    {
      "groupId": "ca.frc6390.athena",
      "artifactId": "athena-telemetry",
      "version": "2026.3.0"
    },
    {
      "groupId": "ca.frc6390.athena",
      "artifactId": "athena-plugin",
      "version": "2026.3.0"
    }
  ]
}
```

Then the plugin adds conditional dependencies:

```groovy
dependencies {
    implementation "ca.frc6390.athena:athena-vendor-ctre:2026.3.0"
}
```

The project template or Athena bootstrap applies the plugin:

```groovy
plugins {
    id 'edu.wpi.first.GradleRIO'
    id 'ca.frc6390.athena'
}
```

This avoids relying on vendordeps to mutate Gradle plugin configuration.

## Vendor Detection Examples

Detection table:

```java
VendorFeature CTRE = VendorFeature.create("ctre")
        .vendordepUuid("e995de00-2c64-4df5-8831-c1441420ff19")
        .dependency("com.ctre.phoenix6", "wpiapi-java")
        .athenaArtifact("ca.frc6390.athena", "athena-vendor-ctre");

VendorFeature REV = VendorFeature.create("rev")
        .dependency("com.revrobotics.frc", "REVLib-java")
        .athenaArtifact("ca.frc6390.athena", "athena-vendor-rev");
```

Plugin behavior:

```java
for (VendorFeature feature : AthenaFeatures.vendors()) {
    if (projectVendors.has(feature) || athenaExtension.vendors().enabled(feature)) {
        project.getDependencies().add(
                "implementation",
                feature.athenaCoordinate(athenaVersion));
    }
}
```

Command-line override:

```shell
./gradlew build -Pathena.vendors=ctre,rev
./gradlew build -Pathena.features=hardware,mechanisms,telemetry
```

## Public Hardware API Examples

Simple explicit hardware:

```java
MotorConfig roller = MotorConfig.create()
        .hardware(AthenaMotor.SPARK_MAX_BRUSHLESS, 3)
        .brake()
        .currentLimit(35)
        .integratedEncoder();
```

CTRE equivalent:

```java
MotorConfig shooter = MotorConfig.create()
        .hardware(AthenaMotor.TALON_FX, 12)
        .canbus("canivore")
        .coast()
        .currentLimit(60)
        .integratedEncoder();
```

Robot hardware aliases:

```java
public final class RobotHardware {
    public static final MotorId INTAKE_ROLLER =
            MotorId.of(AthenaMotor.SPARK_MAX_BRUSHLESS, 3);

    public static final MotorId SHOOTER_LEADER =
            MotorId.of(AthenaMotor.TALON_FX, 12).canbus("canivore");
}
```

Use aliases in mechanisms:

```java
MotorConfig roller = MotorConfig.create()
        .hardware(RobotHardware.INTAKE_ROLLER)
        .brake()
        .currentLimit(35);
```

Vendor-specific escape hatch:

```java
MotorConfig shooter = MotorConfig.create()
        .hardware(AthenaMotor.TALON_FX, 12)
        .vendor(CtreMotorOptions.class, ctre -> ctre
                .statorCurrentLimit(80)
                .supplyCurrentLimit(50));
```

The escape hatch lives in the vendor artifact. If
`athena-vendor-ctre` is not on the classpath, `CtreMotorOptions` does not exist
and the code will not compile.

Direct CTRE/REV/etc. config objects are not accepted by the generic
Athena API. Vendor artifacts may translate typed Athena option classes into
vendor config objects internally.

## Public Mechanism API Examples

Compact mechanism:

```java
public final class Intake {
    public static final MechanismConfig CONFIG = Mechanisms.simple("intake")
            .motor("roller", motor -> motor
                    .hardware(RobotHardware.INTAKE_ROLLER)
                    .brake()
                    .currentLimit(35))
            .control(control -> control
                    .percentOutput());
}
```

Flywheel:

```java
public final class Shooter {
    public static final MechanismConfig CONFIG = Mechanisms.flywheel("shooter")
            .motor("leader", motor -> motor
                    .hardware(RobotHardware.SHOOTER_LEADER)
                    .coast()
                    .currentLimit(60)
                    .integratedEncoder())
            .motor("follower", motor -> motor
                    .hardware(AthenaMotor.TALON_FX, 13)
                    .follow("leader", true))
            .control(control -> control
                    .velocity(pid -> pid
                            .p(0.14)
                            .i(0.0)
                            .d(0.001)))
            .state("idle", state -> state.target(0.0))
            .state("speaker", state -> state.target(4600.0))
            .state("amp", state -> state.target(1800.0));
}
```

Arm:

```java
public final class Arm {
    public static final MechanismConfig CONFIG = Mechanisms.arm("arm")
            .motor("pivot", motor -> motor
                    .hardware(AthenaMotor.SPARK_FLEX_BRUSHLESS, 21)
                    .brake()
                    .currentLimit(50))
            .encoder("absolute", encoder -> encoder
                    .hardware(AthenaEncoder.CANCODER, 22)
                    .canbus("canivore")
                    .offsetDegrees(91.4))
            .positionSource("absolute")
            .limits(limits -> limits
                    .minDegrees(-20)
                    .maxDegrees(105))
            .control(control -> control
                    .position(pid -> pid
                            .p(0.08)
                            .d(0.002))
                    .feedforward(ff -> ff
                            .gravity(0.32)));
}
```

## Annotation API

Annotation style stays on the back burner for V3 until the regular Java
API and internal spec model settle.

The current priority is:

```text
fluent/structured Java declarations
    -> immutable internal specs
    -> validation
    -> backend creation
```

Once that path is stable, annotations can be reconsidered as another frontend
that lowers into the same internal specs. V3 does not lock in an annotation
shape before the underlying model is proven.

## Internal Lowering Example

Public declarations lower into immutable specs before vendor code runs.
The public API exposes this as `toSpec()` for good DX. Internally,
`toSpec()` may delegate to a lowerer, but users do not need to know about
that pipeline.

```java
MechanismConfig config = Intake.CONFIG;

MechanismSpec spec = config.toSpec();
ValidationReport report = spec.validate();

if (report.hasErrors()) {
    throw new AthenaConfigurationException(report);
}

MechanismController.of(spec, List.of(shooterLeader, shooterFollower))
        .applyState("speaker");
```

Explicit validation context remains available for tests, simulation, and
advanced tooling:

```java
AthenaValidationContext context = AthenaValidationContext.create()
        .backends(BackendRegistry.discover())
        .identityMap(RobotIdentityMap.create())
        .mode(AthenaMode.SIMULATION);

ValidationReport report = spec.validate(context);
```

Implementation shape:

```java
public final class MechanismConfig {
    public MechanismSpec toSpec() {
        return MechanismLowerer.lower(this);
    }
}

public record MechanismSpec(...) {
    public ValidationReport validate() {
        return AthenaValidator.validate(this, AthenaValidationContext.global());
    }

    public ValidationReport validate(AthenaValidationContext context) {
        return AthenaValidator.validate(this, context);
    }
}
```

Example lowered motor:

```java
public record MotorSpec(
        String mechanismName,
        String name,
        MotorKind kind,
        DeviceIdentity identity,
        NeutralMode neutralMode,
        CurrentLimitSpec currentLimit,
        EncoderSpec encoder,
        VendorOptions vendorOptions
) {}
```

Example hardware kind:

```java
public enum AthenaMotor implements MotorKind {
    TALON_FX("ctre:talon-fx"),
    SPARK_MAX_BRUSHLESS("rev:spark-max-brushless"),
    SPARK_FLEX_BRUSHLESS("rev:spark-flex-brushless");

    private final String key;

    AthenaMotor(String key) {
        this.key = key;
    }

    public String key() {
        return key;
    }
}
```

Common built-in hardware enums live in generic Athena API/hardware tiers
as stable keys. A constant such as `AthenaMotor.TALON_FX` is allowed to exist
without `athena-vendor-ctre` installed; it only becomes constructible when the
matching backend is present.

## Backend Contract Example

Generic backend contract:

```java
public interface MotorBackend {
    boolean supports(MotorKind kind);

    CapabilitySet capabilities(MotorKind kind);

    MotorDevice create(MotorSpec spec);
}
```

CTRE backend:

```java
public final class CtreMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.key().equals("ctre:talon-fx");
    }

    @Override
    public CapabilitySet capabilities(MotorKind kind) {
        return CapabilitySet.of(
                MotorCapability.PERCENT_OUTPUT,
                MotorCapability.VOLTAGE_OUTPUT,
                MotorCapability.POSITION_CLOSED_LOOP,
                MotorCapability.VELOCITY_CLOSED_LOOP,
                MotorCapability.INTEGRATED_ENCODER,
                MotorCapability.ABSOLUTE_ENCODER);
    }

    @Override
    public MotorDevice create(MotorSpec spec) {
        TalonFX talon = new TalonFX(spec.identity().id(), spec.identity().canbus());
        CtreConfigurator.apply(talon, spec);
        return new CtreMotorDevice(talon, spec);
    }
}
```

REV backend:

```java
public final class RevMotorBackend implements MotorBackend {
    @Override
    public boolean supports(MotorKind kind) {
        return kind.key().startsWith("rev:");
    }

    @Override
    public MotorDevice create(MotorSpec spec) {
        SparkBase spark = RevSparkFactory.create(spec);
        RevConfigurator.apply(spark, spec);
        return new RevMotorDevice(spark, spec);
    }
}
```

Service descriptor:

```text
META-INF/services/ca.frc6390.athena.hardware.backend.MotorBackend
```

```text
ca.frc6390.athena.vendor.ctre.CtreMotorBackend
```

## Capability Validation Example

Mechanism requests velocity control:

```java
.control(control -> control.velocity(pid -> pid.p(0.14)))
```

Lowered requirement:

```java
CapabilityRequirement requirement =
        MotorCapability.VELOCITY_CLOSED_LOOP.requiredBy("shooter.leader");
```

Validation:

```java
MotorBackend backend = backends.motorBackendFor(spec.kind());

if (!backend.capabilities(spec.kind()).contains(requirement.capability())) {
    errors.add(AthenaError.missingCapability(spec, requirement));
}
```

Error:

```text
shooter.leader requires velocity closed-loop control.
Backend rev:spark-max-brushed is installed but does not report that capability.
```

## Vendor Option Example

Generic config:

```java
MotorConfig motor = MotorConfig.create()
        .hardware(AthenaMotor.TALON_FX, 1)
        .currentLimit(50);
```

Vendor-specific config when needed:

```java
MotorConfig motor = MotorConfig.create()
        .hardware(AthenaMotor.TALON_FX, 1)
        .vendor(CtreMotorOptions.class, ctre -> ctre
                .supplyCurrentLimit(50)
                .statorCurrentLimit(90)
                .torqueCurrentLimit(120));
```

Lowered form:

```java
public record VendorOptions(Map<Class<?>, Object> options) {
    public <T> Optional<T> find(Class<T> type) {
        return Optional.ofNullable(type.cast(options.get(type)));
    }
}
```

Backend usage:

```java
CtreMotorOptions options = spec.vendorOptions()
        .find(CtreMotorOptions.class)
        .orElse(CtreMotorOptions.defaults());
```

This keeps the generic path clean while allowing real vendor behavior without
forcing all vendor details into Athena Core.

## Runtime Discovery Example

Classpath contains:

```text
athena-api
athena-runtime
athena-hardware
athena-mechanisms
athena-vendor-ctre
```

Runtime registry:

```java
BackendRegistry registry = BackendRegistry.discover();

MotorBackend backend = registry.motorBackendFor(AthenaMotor.TALON_FX);
MotorDevice device = backend.create(spec);
```

Missing backend:

```text
No motor backend for rev:spark-max-brushless.

Detected Athena vendor backends:
- ctre

Fix:
- install REVLib
- enable Athena vendor auto-detection
- or explicitly add athena-vendor-rev
```

## Local Repo Build Examples

Build only core tiers:

```shell
./gradlew build -Pathena.features=api,runtime,hardware,mechanisms
```

Build CTRE only:

```shell
./gradlew build -Pathena.features=api,runtime,hardware,mechanisms,vendor-ctre
```

Build REV only:

```shell
./gradlew build -Pathena.features=api,runtime,hardware,mechanisms,vendor-rev
```

Build all:

```shell
./gradlew build -Pathena.features=all
```

`settings.gradle` can include modules conditionally for feature-selected local
builds:

```groovy
include 'athena-api'
include 'athena-runtime'
include 'athena-hardware'

if (athenaFeatures.has('mechanisms')) {
    include 'athena-mechanisms'
}

if (athenaFeatures.has('vendor-ctre')) {
    include 'athena-vendor-ctre'
}
```

## CI Matrix

```text
api + runtime
api + runtime + hardware
api + runtime + hardware + mechanisms
api + runtime + hardware + mechanisms + vendor-ctre
api + runtime + hardware + mechanisms + vendor-rev
api + runtime + hardware + mechanisms + vendor-ctre + vendor-rev
all
```

Each vendor job proves that only that vendor's dependencies are required.

## Migration Sketch

1. Create tier artifacts while keeping current packages compiling.
2. Move public API declarations into `athena-api`.
3. Move lifecycle, registries, and validation into `athena-runtime`.
4. Move generic hardware specs and capabilities into `athena-hardware`.
5. Move mechanisms out of core into `athena-mechanisms`.
6. Move current CTRE/REV/etc. implementations into `athena-vendor-*`.
7. Add the single `FRC6390-Athena.json` vendordep.
8. Add Gradle plugin vendor detection.
9. Switch vendor adapters from public mutable configs to lowered specs.
10. Remove old modular Athena vendordeps once the single vendordep path works.

## Finalized Design Decisions

- The project template/bootstrap applies the Athena Gradle plugin. The vendordep
  supplies artifacts; it does not need to apply plugins.
- `FRC6390-Athena.json` includes `athena-api`, `athena-runtime`,
  `athena-hardware`, `athena-mechanisms`, `athena-commands`,
  `athena-telemetry`, and `athena-plugin`.
- Mechanisms, commands, and telemetry are default because they are core to the
  intended student-facing Athena experience.
- Drivetrain, superstructure, auto, vision, simulation, and vendor adapters are
  opt-in or auto-detected feature artifacts.
- Localization is an opt-in feature artifact, not part of the default
  vendordep.
- Real WPILib glue is concentrated in `athena-wpilib`.
- ARCP/dashboard control remains separate from the robot runtime and connects
  through telemetry/diagnostics APIs.
- Vendor-specific options use typed Athena option extension classes with lambda
  configuration.
- Direct vendor config objects stay inside vendor artifacts and adapter internals.
- `AthenaMotor`, `AthenaEncoder`, and similar common enums include common device
  keys directly. Missing vendor backends are handled by validation/runtime
  errors, not by hiding constants.
- Third-party vendors publish an Athena feature metadata resource in their
  adapter artifact:

```json
{
  "feature": "acme",
  "displayName": "Acme Robotics",
  "detect": {
    "vendordepUuids": ["aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"],
    "dependencies": ["com.acme.frc:AcmeLib-java"]
  },
  "artifacts": [
    "com.acme.frc:athena-vendor-acme"
  ]
}
```

The Gradle plugin may also allow explicit registration for local/private
adapters:

```groovy
athena {
    vendorMetadata {
        vendor("acme") {
            dependency "com.acme.frc:AcmeLib-java"
            artifact "com.acme.frc:athena-vendor-acme:2026.3.0"
        }
    }
}
```
