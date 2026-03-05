# Blank WPILib Project Setup (Gradle)

This is the minimum setup from a blank WPILib Java project.

## 1) Install Vendordeps First

Use WPILib vendordep manager and install at minimum:

- `FRC6390-Athena-Core.json`

Add any optional Athena modules you use (`CTRE`, `REV`, `PhotonVision`, `Limelight`, `HeliOS`, `Pathplanner`, `Choreo`, `Studica`) and their required upstream vendordeps.

## 2) Keep WPILib executable JAR wiring (rio deploy + fat jar)

If you modify `jar` for a fat jar, keep the GradleRIO manifest/task wiring.

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

The key line for the rio manifest is:

```groovy
manifest edu.wpi.first.gradlerio.GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS)
```

## 3) Optional: Athena State DSL plugin support

Only needed if you use Athena State DSL/compiler-plugin features.

`settings.gradle`:

```groovy
pluginManagement {
    repositories {
        mavenLocal()
        gradlePluginPortal()
        maven { url 'https://frcmaven.wpi.edu/artifactory/release' }
        maven { url 'https://maven.frc6390.ca/FRC-6390/FRC-Athena' }
    }
}
```

`build.gradle`:

```groovy
buildscript {
    repositories {
        mavenLocal()
        maven { url 'https://frcmaven.wpi.edu/artifactory/release' }
        maven { url 'https://maven.frc6390.ca/FRC-6390/FRC-Athena' }
    }
    dependencies {
        classpath "ca.frc6390.athena:athena-plugin:<athena-version>"
    }
}

apply plugin: "ca.frc6390.athena.plugin"
```

Use the same `<athena-version>` as your Athena vendordep version.

## 4) Verify

Run:

```bash
./gradlew build
```

If setup is correct:

- vendordep artifacts resolve
- Athena plugin resolves
- compile tasks include Athena State DSL wiring (when enabled)
