# FRC-Athena

## Vendordep URLs

Athena vendordeps (install these from WPILib vendordep manager):
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Core.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-CTRE.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-REV.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-PhotonVision.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Limelight.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Pathplanner.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Choreo.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Studica.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/Studica-2026.0.0.json

External vendordeps required by Athena modules:
- https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json
- https://software-metadata.revrobotics.com/REVLib-2026.json
- https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
- https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
- https://choreo.autos/lib/ChoreoLib2026.json

## Blank WPILib Project Setup (Required Gradle Changes)

Minimum required for most teams:

Full guide:
- `docs/core/wpilib-project-setup.md`

- Install Athena vendordep(s).
- Keep WPILib executable/fat-jar wiring for rio deploy (especially if you edit `jar`):
```groovy
def ROBOT_MAIN_CLASS = "frc.robot.Main"
def deployArtifact = deploy.targets.roborio.artifacts.frcJava

jar {
    from { configurations.runtimeClasspath.collect { it.isDirectory() ? it : zipTree(it) } }
    from sourceSets.main.allSource
    manifest edu.wpi.first.gradlerio.GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS)
    duplicatesStrategy = DuplicatesStrategy.INCLUDE
}

deployArtifact.jarTask = jar
wpi.java.configureExecutableTasks(jar)
```

Only if using Athena State DSL/compiler-plugin features:
- add Athena maven to `pluginManagement.repositories` in `settings.gradle`
- add `athena-plugin` classpath + `apply plugin: "ca.frc6390.athena.plugin"` in `build.gradle`

## Build Commands

Build changed Athena modules and downstream dependents (default `build` behavior):
```bash
./gradlew build
```

Run the changed-module task directly:
```bash
./gradlew buildChangedModules
```

Build changed modules from a specific git base:
```bash
./gradlew buildChangedModules -PchangedSince=origin/main
```

Force build all Athena modules:
```bash
./gradlew buildAllModules
```

Force all modules through root build:
```bash
./gradlew build -PbuildAllModules=true
```

Build each module directly (per dependency module):
```bash
./gradlew :athena-core:build
./gradlew :athena-ctre:build
./gradlew :athena-rev:build
./gradlew :athena-photonvision:build
./gradlew :athena-limelight:build
./gradlew :athena-pathplanner:build
./gradlew :athena-choreo:build
./gradlew :athena-studica:build
```

Publish all modules locally to WPILib:
```bash
./gradlew publish -PpublishMode=local
```

Publish all modules online:
```bash
./gradlew publish -PpublishMode=online
```

Publish a single module locally:
```bash
./gradlew :athena-core:publishToMavenLocal
./gradlew :athena-ctre:publishToMavenLocal
./gradlew :athena-rev:publishToMavenLocal
./gradlew :athena-photonvision:publishToMavenLocal
./gradlew :athena-limelight:publishToMavenLocal
./gradlew :athena-pathplanner:publishToMavenLocal
./gradlew :athena-choreo:publishToMavenLocal
./gradlew :athena-studica:publishToMavenLocal
```

Set explicit version/FRC year during build or publish:
```bash
./gradlew build -Pversion="2026.1.99" -PfrcYear="2026"
./gradlew publish -PpublishMode=local -Pversion="2026.1.99" -PfrcYear="2026"
```

Online publish credentials:
```bash
export MAVEN_USERNAME=<username>
export MAVEN_PASSWORD=<token-or-password>
./gradlew publish -PpublishMode=online
```
