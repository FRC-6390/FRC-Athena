# FRC Athena 2027

This folder is the V3 replacement workspace for Athena. The repository root is
treated as frozen legacy code while the 2027 API, module layout, examples, and
deployment infrastructure are rebuilt here.

## Goals

- One human-facing Athena vendordep: `FRC6390-Athena.json`.
- Clean tiered artifacts instead of one massive core.
- Vendor adapters selected by Gradle feature/vendor detection.
- Student-facing Java APIs that lower through `toSpec()`.
- Immutable specs, explicit validation, and small backend contracts.
- Tests, examples, and docs per major artifact.

## Modules

- `athena-api`: public declarations and common value types.
- `athena-runtime`: validation, errors, contexts, registries, and lifecycle.
- `athena-hardware`: generic hardware specs, capabilities, and backend contracts.
- `athena-mechanisms`: mechanism declarations, lowering to specs, and state runtime application.
- `athena-superstructure`: multi-mechanism coordination.
- `athena-drivetrain`: drivetrain declarations.
- `athena-vision`: generic camera declarations and target observations.
- `athena-localization`: pose-estimation declarations and field pose aliases.
- `athena-auto`: autonomous routine registry and selection model.
- `athena-commands`: command integration boundary.
- `athena-telemetry`: telemetry registration boundary.
- `athena-dashboard`: optional TCP dashboard/control transport.
- `athena-plugin`: Gradle/vendor feature detection model.
- `athena-simulation`: simulation test helpers and fake backends.
- `athena-wpilib`: optional WPILib command, lifecycle, controller, localization,
  and NetworkTables adapter boundary.
- `athena-vendor-ctre`: CTRE TalonFX/Kraken, CANcoder, Pigeon2, and typed CTRE options.
- `athena-vendor-rev`: REV Spark and through-bore encoder adapters plus typed
  REV options.
- `athena-vendor-studica`: Studica/NavX IMU adapter.
- `athena-vendor-photonvision`: PhotonVision camera adapter.
- `athena-vendor-limelight`: Limelight NetworkTables camera adapter.
- `athena-vendor-pathplanner`: PathPlanner autonomous source and command adapter.
- `athena-vendor-choreo`: Choreo autonomous source and trajectory adapter.
- `example-project`: broad student-facing example project.

## Build

From the repository root:

```shell
./gradlew -p 2027 build
```

## Current Status

The replacement workspace now has tested generic API slices for:

- common hardware keys
- motor, encoder, IMU, camera, and input configuration
- mechanism declaration and superstructure coordination/runtime application
- `toSpec()` lowering
- validation with default and explicit contexts
- backend capability checks
- differential and swerve drivetrain declarations
- command descriptors and autonomous routine selection
- telemetry snapshots and NetworkTables-shaped publishing
- dashboard packet, control-message, and optional TCP transport
- generic vision observations
- localization weighting, slip detection, field bounds, and pose aliases
- stateful simulation helpers
- WPILib adapter boundary for command, lifecycle, controller, and
  NetworkTables integration
- WPILib differential and swerve drive runtime bindings
- WPILib pose-estimator vision measurement weighting
- a documented example project with catalog verification
- generated single-vendordep metadata
- Gradle plugin feature/vendor selection
- CTRE and REV motor adapters with typed vendor options
- CTRE CANcoder and REV through-bore encoder adapters
- CTRE Pigeon2 and Studica/NavX IMU adapters
- PhotonVision and Limelight camera adapters
- PathPlanner autonomous command adapter and Choreo trajectory adapter
- release metadata and local Maven staging checks

No remaining major migrations are tracked for the fluent Java V3 replacement
pass. New work should start from a concrete adapter, dashboard workspace,
annotation DSL, or robot-use-case requirement instead of reopening the accepted
architecture shape.

## Release Metadata

Version and vendordep metadata are centralized in `gradle.properties`.

```shell
./gradlew -p 2027 generateVendordep
```

The generated file is `vendordeps/FRC6390-Athena.json`.

For a release-candidate check with local Maven staging:

```shell
./gradlew -p 2027 releaseChecklist
```

The local staging repository is written to `2027/build/staging-repo`.

## Examples

See [Example Catalog](./docs/examples.md) for a walkthrough of the current
student-facing syntax.

## Roadmap

See [Coverage Roadmap](./docs/coverage-roadmap.md) for implemented slices and
the current migration status.

See [API V3 Direction](./docs/api-v3-direction.md) for the accepted architecture
shape, artifact split, and scope boundaries.

See [Migration Matrix](./docs/migration-matrix.md) for the source-to-2027
tracking table used to keep legacy feature migration explicit.

See [Example Migration Map](./docs/example-migration-map.md) for the checked
mapping from every frozen legacy example file to its 2027 coverage.

See [Completion Evidence](./docs/completion-evidence.md) for the requirement to
evidence map used to audit the replacement workspace against the original goal.
