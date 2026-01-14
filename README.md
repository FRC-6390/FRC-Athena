# Vendordeps
Use the module-specific vendordeps from the `vendordeps/` folder (e.g., Core, CTRE, REV, PhotonVision, Limelight, Pathplanner, Choreo, Studica). Each JSON lives at:
```
https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/<file>.json
```
Direct links:
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Core.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-CTRE.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-REV.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-PhotonVision.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Limelight.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Pathplanner.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Choreo.json
- https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/vendordeps/FRC6390-Athena-Studica.json

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

To chain or branch autos, use the sequence/branch helpers. Choreo split trajectories can be referenced with a `.index` suffix (e.g., `CompRightSide.1`) when registering the auto:

```java
autos.registerChoreoAuto("RightAuto.1", "CompRightSide.1");
autos.registerAutoSequence("TwoPiece", "RightAuto", "RightAuto.1");
autos.registerAutoBranch("TwoPieceOrLeave", hasNoteSupplier, "TwoPiece", "Leave");
```

For more complex logic (branching + interrupts), build an `AutoPlan` and register it:

```java
AutoPlan plan = AutoPlan.sequence(
        AutoPlan.step("RightAuto"),
        AutoPlan.branch(hasNoteSupplier, AutoPlan.step("RightAuto.1"), AutoPlan.step("Leave")));

AutoPlan bailout = AutoPlan.interrupt(plan, shouldAbortSupplier, AutoPlan.step("Abort"));
autos.registerAutoPlan("TwoPieceOrLeave", bailout);
```

AutoPlan steps can reset odometry explicitly:

```java
AutoPlan plan = AutoPlan.sequence(
        AutoPlan.step("RightAuto", true),
        AutoPlan.step("RightAuto", 1, new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(180))));
```

AutoPlans can also mix in custom commands:

```java
AutoPlan plan = AutoPlan.sequence(
        AutoPlan.step("RightAuto"),
        AutoPlan.command(trackTargetCommand),
        AutoPlan.step("RightAuto", 1));
```

AutoPlans support parallel, race, and deadline groups:

```java
AutoPlan plan = AutoPlan.sequence(
        AutoPlan.deadline(
                AutoPlan.step("RightAuto"),
                AutoPlan.command(spinUpShooter)),
        AutoPlan.race(
                AutoPlan.command(feedNoteCommand),
                AutoPlan.command(waitForBeamBreak)),
        AutoPlan.parallel(
                AutoPlan.step("RightAuto", 1),
                AutoPlan.command(keepAimingCommand)));
```

AutoPlans can wrap steps with timeouts or end conditions:

```java
AutoPlan plan = AutoPlan.sequence(
        AutoPlan.withTimeout(AutoPlan.step("RightAuto"), 2.5),
        AutoPlan.until(AutoPlan.command(trackTargetCommand), hasTagSupplier));
```

Everything works directly with strings; if you prefer enums, implement `RobotAuto.NamedCommandKey` / `RobotAuto.AutoKey` and reuse these helpers. Once the first auto is added, named-command registration locks to prevent missing dependencies. Use `setStartingPose()` for extra metadata, and call `registerAutoRoutineChooser(...)` if you want the raw routine objects on Shuffleboard.

## Localization Camera Auto Align
Use the `AlignAndDriveToTagCommand` to close the loop on robot-relative translations produced by any configured localization camera. Supply tuned PID controllers, the desired standoff pose (as a `Pose2d`), a tolerance pose, and clamp limits to make the command suit your drivetrain. The command always targets one or more specific camera tables, so pass the NetworkTables names you want it to consume.

```java
PIDController xController = new PIDController(1.6, 0.0, 0.0);
PIDController yController = new PIDController(1.6, 0.0, 0.0);
PIDController headingController = new PIDController(0.04, 0.0, 0.001);

Command autoAlign = new AlignAndDriveToTagCommand(
        robot,
        xController,
        yController,
        headingController,
        new Pose2d(Units.inchesToMeters(18.0), 0.0, new Rotation2d()), // hold 18" off the tag
        new Pose2d(0.03, 0.03, Rotation2d.fromDegrees(2.5)), // per-axis tolerance
        2.0,                          // clamp translation speed
        3.0,                          // clamp rotation speed
        3.0,                          // optional timeout
        "limelight-front");

robot.getAutos().registerNamedCommand("AutoAlignToTag", autoAlign);
```

You can restrict the command to particular cameras or roles when needed:

```java
EnumSet<LocalizationCameraConfig.CameraRole> preferredRoles =
        EnumSet.of(LocalizationCameraConfig.CameraRole.ALIGNMENT);
Set<String> cameraTables = Set.of("limelight-left", "photon-front");

Command filteredAlign = new AlignAndDriveToTagCommand(
        robot,
        xController,
        yController,
        headingController,
        new Pose2d(Units.inchesToMeters(18.0), 0.0, new Rotation2d()),
        new Pose2d(0.03, 0.03, Rotation2d.fromDegrees(2.5)),
        2.0,
        3.0,
        3.0,
        cameraTables,
        preferredRoles);
```

Tune the controller gains/tolerances to match your drivetrain and sensor noise profile. The command stops driving when all three controllers report on-target or when the timeout elapses.

## Stateful Mechanisms
`MechanismConfig.stateful*` builders now accept a `StateGraph` so you can declare required transition states and guard conditions in one place. The graph expands any `requestState(...)` call into the full path and injects boolean predicates that must pass before each hop:

```java
BooleanSupplier armClear = arm::atSafeAngle;
BooleanSupplier elevatorClear = elevator::atSafeHeight;

StateGraph<SuperState> graph = StateGraph
        .create(SuperState.class)
        .path(SuperState.STOW, SuperState.CLEARANCE, SuperState.SCORE_HIGH)
        .guardPath(StateGraph.Guards.allOf(armClear, elevatorClear),
                   SuperState.STOW, SuperState.CLEARANCE);

MechanismConfig.statefulGeneric(SuperState.STOW)
        .setStateGraph(graph)
        .setStateAction(superstructure::zeroSensors, SuperState.STOW)
        .build();
```

At runtime request goals declaratively: `superstructure.getStateMachine().requestState(SuperState.SCORE_HIGH);` or continue using `queueState(...)`—the graph expands either call, inserts any intermediate states, and waits on the attached guards before each hop. See `src/main/java/ca/frc6390/athena/mechanisms/examples/ExampleSuperstructure.java` for a complete mock-up.
