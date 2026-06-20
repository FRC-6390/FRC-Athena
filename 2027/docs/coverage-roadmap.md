# Coverage Roadmap

This roadmap tracks the replacement work inside `2027/`. The repository root is
treated as frozen legacy code; new work should land in this workspace.

## Implemented Slices

| Area | Evidence |
| --- | --- |
| Multi-module Gradle build | `settings.gradle`, `build.gradle` |
| Single Athena vendordep generation | `generateVendordep`, `vendordeps/FRC6390-Athena.json` |
| Public hardware keys and aliases | `athena-api` tests |
| Motor specs/configs | `athena-hardware` tests |
| Encoder specs/configs | `athena-hardware` tests |
| Typed inputs | `athena-hardware` and `athena-mechanisms` tests |
| Sensor wrappers | `athena-hardware`, `athena-vision`, and example tests |
| Backend registry/capabilities | `athena-hardware` tests |
| Mechanism declarations | `athena-mechanisms` tests |
| PID/feedforward/state declarations | `athena-mechanisms` tests |
| Mechanism state runtime controller | `MechanismController`, vendor motor tests, and example tests |
| Fluent v2 mechanism shape examples | `MechanismV2Example` and example tests |
| Differential drivetrain declarations | `athena-drivetrain` tests |
| Swerve drivetrain declarations | `athena-drivetrain` and example tests |
| Superstructure declarations, nested assembly examples, transition planning, and runtime state application | `athena-superstructure` and example tests |
| Command descriptors, named requirements, sequence/parallel groups, and robot-speed drive commands | `athena-commands` and example tests |
| Controller helpers and filters | `athena-runtime` and example tests |
| Motion limits and timed runner utilities | `athena-runtime` and example tests |
| Robot speed source blending | `athena-runtime` and example tests |
| Real WPILib command/subsystem requirement/lifecycle/NetworkTables adapters and controller helpers | `athena-wpilib` and example tests |
| Real WPILib differential and swerve drive runtime adapters | `athena-wpilib` and example tests |
| Autonomous chooser, source registry, and scoped input handoff | `athena-auto` and example tests |
| Real PathPlanner auto command calls plus Choreo trajectory loading and AutoFactory command wrappers | `athena-vendor-pathplanner`, `athena-vendor-choreo`, and example tests |
| Telemetry registry/snapshots and NetworkTables-shaped sink | `athena-telemetry` tests |
| Diagnostics channels and bounded event logs | `athena-runtime` and example tests |
| Dashboard/control bridge boundary and JSON wire payloads | `athena-dashboard` and example tests |
| Generic vision camera/target model | `athena-vision` and example tests |
| Vision frame command feedback helper | `VisionTurnAssist`, `DriveCommandExample`, and example tests |
| Real PhotonVision camera library calls and Limelight NetworkTables reads | `athena-vendor-photonvision`, `athena-vendor-limelight`, and example tests |
| Localization weighting, slip detection, field bounds, and pose aliases | `athena-localization` and example tests |
| WPILib pose-estimator vision measurement weights | `WpilibPoseEstimatorAdapter` and example tests |
| Gradle plugin and feature/vendor selection | `athena-plugin` tests |
| Resource-backed vendor metadata loading | `athena-plugin` tests, `verifyVendorMetadata` |
| Vendor ServiceLoader packaging checks | `verifyVendorServiceDescriptors` |
| Core/vendor architecture boundary checks | `verifyArchitectureBoundaries` |
| IMU specs/configs | `athena-hardware` tests |
| Typed vendor motor options | `athena-hardware`, `athena-vendor-ctre`, and `athena-vendor-rev` tests |
| Real CTRE TalonFX/Kraken and REV Spark Max/Flex motor device calls, closed-loop target calls, plus integrated and REV attached absolute encoder reads | `athena-vendor-ctre` and `athena-vendor-rev` tests |
| Real CTRE CANcoder device reads and encoder backend contract | `athena-hardware` and `athena-vendor-ctre` tests |
| Real REV through-bore duty-cycle encoder reads and encoder backend contract | `athena-vendor-rev` tests |
| Real CTRE Pigeon2 IMU library calls | `athena-vendor-ctre`, plugin metadata, and example tests |
| Real Studica/NavX IMU library calls | `athena-vendor-studica`, plugin metadata, and example tests |
| Vendor option syntax in examples | `VendorOptionsExample`, example docs, example tests |
| Simulation motor/IMU backends, mechanism/drivetrain coupling, and world state | `athena-simulation` and example tests |
| Example catalog verification | `verifyExampleDocs` |
| Legacy example migration tracking against frozen source examples | `docs/example-migration-map.md`, `verifyExampleMigrationMap` |
| Migration matrix tracking | `docs/migration-matrix.md`, `verifyMigrationMatrix` |
| Per-module README and test presence gates | `verifyModuleDocs` |
| Root README module catalog gate | `verifyReadmeModuleCatalog` |
| Module README dependency consistency gate | `verifyReadmeDependencies` |
| Public API class-level Javadoc gate | `verifyPublicApiJavadocs` |
| Release metadata and staging publication gates | `verifyReleaseMetadata`, `releaseChecklist` |

## Remaining Major Migrations

No remaining major migrations are tracked for the fluent Java V3 replacement
pass. New work should enter this document as a concrete adapter, dashboard
workspace, annotation DSL, or robot-use-case requirement.

## Verification Command

Run this before considering a slice complete:

```shell
./gradlew -p 2027 generateVendordep build
```

The command runs compilation, Javadocs, source/Javadoc jar creation, module
tests, vendordep generation, example documentation verification, and migration
matrix verification.
