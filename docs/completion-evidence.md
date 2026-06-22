# Completion Evidence

This file ties the promoted root workspace back to the transition goal: evict
the old monolithic Athena Java modules, promote the V3 artifact split to the
repository root, preserve separate workspaces that are not replaced yet, provide
a documented example project, test each module, and keep release/revision work
straightforward.

## Requirement Evidence

| Requirement | Evidence | Verification |
| --- | --- | --- |
| Promoted root replacement workspace | The root Gradle build owns the V3 modules, docs, examples, vendordep, and release workflow; replaced legacy Java modules are evicted. | `./gradlew build` |
| Clean tiered artifact layout instead of one massive core | `settings.gradle`, root `README.md`, and per-module READMEs list the public, runtime, hardware, mechanism, drivetrain, superstructure, command, auto, telemetry, dashboard, simulation, WPILib, localization, plugin, and vendor artifacts. | `verifyReadmeModuleCatalog`, `verifyModuleDocs`, `verifyArchitectureBoundaries` |
| Single human-facing Athena vendordep | `vendordeps/FRC6390-Athena.json` is generated from Gradle metadata and contains only the default student-facing artifacts. | `generateVendordep`, `verifyVendordep`, `verifyReleaseMetadata` |
| Vendor dependency isolation and conditional selection | Vendor integrations live in `athena-vendor-*`; metadata resources under `athena-plugin/src/main/resources/META-INF/athena/vendors` drive detection without forcing all vendor dependencies on students. | `verifyVendorMetadata`, `verifyVendorServiceDescriptors`, `verifyArchitectureBoundaries` |
| Student-facing Java syntax using `toSpec()` | Example sources use fluent declarations and `toSpec()` across hardware, mechanisms, drivetrain, superstructure, auto, telemetry, vision, localization, and vendor options. | `example-project:test`, `verifyExampleDocs` |
| Feature migration from evicted examples | `docs/example-migration-map.md` maps every evicted legacy example file to replacement coverage and separates implemented fluent coverage from intentionally deferred annotation/dashboard work. | `verifyExampleMigrationMap`, `verifyMigrationMatrix`, `verifyV3DirectionBacklog` |
| Full example project | `example-project` contains compile-tested student-facing examples covering the migrated syntax and adapter boundaries, with a catalog in `docs/examples.md`. | `example-project:test`, `verifyExampleDocs` |
| Proper tests for each module | Every published module has focused tests under `src/test/java`; the release build runs module tests and checks. | `verifyModuleDocs`, `./gradlew build` |
| Proper READMEs for each major artifact | Every module has a `README.md` with a `Current Slice` section and dependency documentation. | `verifyModuleDocs`, `verifyReadmeDependencies` |
| Public API comments and Javadocs | Public top-level Java API types are checked for class-level Javadocs, and release artifacts include Javadoc jars. | `verifyPublicApiJavadocs`, `./gradlew build` |
| Clean release and revision infrastructure | Version metadata lives in `gradle.properties`; release checks generate vendordep metadata, validate docs/gates, and stage all publishable artifacts locally. | `releaseChecklist`, `stageMavenPublication` |
| Preserved separate workspaces | `athena-arcp`, `athena-helios`, and `athena-vscode-extension` remain outside the default robot artifact graph until they receive dedicated migrations. | `README.md`, `docs/migration-matrix.md` |

## Accepted Scope Boundaries

| Boundary | Decision |
| --- | --- |
| Annotation/state DSL | Deferred until fluent Java specs and runtime contracts settle. |
| Dashboard layout/UI rendering | Belongs to a separate dashboard/control workspace; `athena-dashboard` owns packets, control messages, and optional TCP transport. |
| Phoenix 5 motor controllers | Requires a separate optional adapter artifact and dependency; the CTRE artifact in this pass is scoped to Phoenix 6 TalonFX/Kraken, CANcoder, and Pigeon2. |
| HeliOS camera adapter | Preserved as `athena-helios` pending a dedicated V3 vendor adapter migration. |

## Final Verification Command

Run this from the repository root before claiming the replacement is ready:

```shell
./gradlew generateVendordep build releaseChecklist
```
