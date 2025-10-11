# VENDOR DEP
`https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/FRC6390-Athena.json`

# Publishing
```bash
export $(cat .env | xargs)
 ```

```bash
 ./gradlew publish -PpublishMode=local # version/year auto-resolve
 ```

 ```bash
export $(cat .env | xargs)
 ./gradlew publish -PpublishMode=online # version/year auto-resolve
 ```

To publish with a specific version, append `-Pversion=<major.minor.patch>` to override the automatic increment. To pin the FRC season manually, append `-PfrcYear=<season>`.

## Autonomous Registration
`RobotAuto` now manages both PathPlanner and Choreo routines with explicit ordering between named commands and auto definitions. Define enums for the keys you care about, register the commands, add autos, and let the chooser build the commands lazily:

```java
RobotAuto autos = robot.getAutos();
autos.registerNamedCommand("WaitForTag", Commands.waitUntil(hasTargetSupplier));
autos.registerNamedCommand("Home", superstructure.setState(SuperstructureState.HomePID));

autos.registerPathPlannerAuto("LeftAuto", "CompLeftSide");
autos.registerChoreoAuto("RightAuto", "CompRightSide");

robot.registerAutoChooser("LeftAuto");
```

Everything works directly with strings; if you prefer enums, implement `RobotAuto.NamedCommandKey` / `RobotAuto.AutoKey` and reuse these helpers. Once the first auto is added, named-command registration locks to prevent missing dependencies. Use `setStartingPose()` for extra metadata, and call `registerAutoRoutineChooser(...)` if you want the raw routine objects on Shuffleboard.
