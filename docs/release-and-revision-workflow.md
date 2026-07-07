# Release and Revision Workflow

Athena V3 keeps release metadata centralized so quick revisions do not require
editing every module by hand.

## Version Bump

Update `gradle.properties`:

```properties
athenaVersion=2027.0.1
```

Then regenerate the vendordep:

```shell
./gradlew generateVendordep
```

## Verification

Run the full root build before publishing:

```shell
./gradlew generateVendordep build
```

The build compiles Java with `-Xlint:all -Werror`, runs module tests, builds
Javadocs, creates source/Javadoc jars for every artifact, validates the
vendordep shape, validates release metadata, and checks the example catalog.

For a release candidate, run the release checklist:

```shell
./gradlew releaseChecklist
```

That task runs the build, validates release metadata, and publishes every Athena
artifact module to a local staging repository at:

```text
build/staging-repo
```

Use the staging repository to inspect generated POMs, jars, sources jars, and
Javadoc jars before pushing to the real Maven repository. The example project is
compiled and tested by the build, but it is not part of the staged Maven
publication set.

## Release Gates

The build enforces:

- `athenaVersion` must follow `2027.x.y` or `2027.x.y-SNAPSHOT`.
- `athenaUuid` must be a valid UUID.
- every default vendordep artifact must have a matching Gradle project.
- the root README module catalog must match the Gradle project list.
- module README dependency sections must match Gradle project dependencies.
- every public top-level Java API type must have class-level Javadocs.
- optional artifacts such as drivetrain, superstructure, vision, localization,
  dashboard, WPILib, simulation, auto, PhotonVision, Limelight, PathPlanner,
  Choreo, and other vendor adapters must not leak into the default
  student-facing vendordep.
- every publishable Athena artifact must expose its expected publication
  (`mavenJava` for Java modules, `pluginMaven` for `athena-plugin`).
- every shipped vendor metadata resource must declare valid detection fields
  and unversioned adapter artifact coordinates.
- every vendor metadata artifact coordinate must map to a real Gradle project.
- hardware vendor adapter ServiceLoader descriptors must list the expected
  backend classes and those classes must exist.
- default/core artifacts must not depend on optional artifacts or import vendor
  adapter packages.
- vendor adapter artifacts must not depend on each other; shared behavior must
  stay in generic Athena artifacts.
- vendor adapter artifacts are published separately and must not leak into the
  default student-facing vendordep.
- replaced legacy modules and the temporary `2027/` migration folder must not
  reappear in the root workspace.
- the vendordep must not publish JNI or C++ dependencies until those are
  intentionally added.

## Publication Shape

The default student-facing vendordep is:

```text
vendordeps/FRC6390-Athena.json
```

It includes only default Athena artifacts. Vendor adapters are selected later by
the Athena plugin from installed vendor dependencies or explicit feature
configuration.

## Quick Revision Checklist

1. Update `athenaVersion` in `gradle.properties`.
2. Run `./gradlew generateVendordep build`.
3. Run `./gradlew releaseChecklist` for a local staging publish.
4. Inspect `build/staging-repo`.
5. Publish the staged artifacts and `vendordeps/FRC6390-Athena.json`.
