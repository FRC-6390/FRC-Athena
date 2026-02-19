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
`RobotAuto` now uses a single fail-fast path API and a program context model:

- Register trajectories with `path(reference, source)`.
- Alias only when needed with `path(reference, source, id)`.
- Build command flow in `build(ctx)` using sequence/parallel/select/wait fallback helpers.

```java
public final class TestAuto implements RobotAuto.AutoProgram {
    @Override
    public String id() {
        return "TestAuto";
    }

    @Override
    public void register(RobotAuto.AutoRegisterCtx ctx) {
        ctx.registerNamedCommand("Home", superstructure.homeCommand());
        ctx.path("TestAuto", RobotAuto.TrajectorySource.CHOREO);
    }

    @Override
    public RobotAuto.AutoRoutine build(RobotAuto.AutoBuildCtx ctx) {
        return custom(() -> ctx.auto(id()));
    }
}

autos.registerProgram(this, new TestAuto());
registerAutoChooser("TestAuto");
```

`path(...)` validates the reference at registration time. Bad source/file combinations throw immediately instead of silently scheduling `Commands.none()` later.

### Basic path + markers
Use the trajectory file name as id unless you intentionally alias it:

```java
ctx.path("HCDKS", RobotAuto.TrajectorySource.CHOREO);                 // id = HCDKS
ctx.path("traj.0", RobotAuto.TrajectorySource.CHOREO, "Opening");   // id = Opening
```

### Complex branching + fallback
Split conditions, parallel behavior, and timeout fallbacks are built in:

```java
RobotAuto.AutoInput<Boolean> intakeBeam = RobotAuto.AutoInput.of("intakeBeam", Boolean.class);
RobotAuto.AutoInput<Integer> routeIndex = RobotAuto.AutoInput.of("routeIndex", Integer.class);

@Override
public void register(RobotAuto.AutoRegisterCtx ctx) {
    ctx.input(intakeBeam, sensors::hasNote);
    ctx.input(routeIndex, strategy::selectedRoute);

    ctx.path("CenterStart", RobotAuto.TrajectorySource.PATH_PLANNER);
    ctx.path("PickupA", RobotAuto.TrajectorySource.PATH_PLANNER);
    ctx.path("PickupB", RobotAuto.TrajectorySource.PATH_PLANNER);
    ctx.path("Bailout", RobotAuto.TrajectorySource.PATH_PLANNER);
}

@Override
public RobotAuto.AutoRoutine build(RobotAuto.AutoBuildCtx ctx) {
    return custom(() -> ctx.sequence(
        ctx.auto("CenterStart"),
        ctx.selectIndex(() -> ctx.inputValue(routeIndex), "Bailout", "PickupA", "PickupB"),
        ctx.parallel(
            superstructure.scorePrepCommand(),
            ctx.waitFor(() -> ctx.inputValue(intakeBeam), 0.8, superstructure.recoverCommand()))));
}
```

`ctx.auto("Path", 1)` resolves split paths (`Path.1`) directly. It uses an explicit registered split id first, then derives from base trajectory reference if needed.

### Auto Planner PID DSL + Autotuner
Localization planner PID can now be configured with a structured DSL:

```java
.localization(l -> l
    .autoPlannerPid(p -> {
        p.translation().kp(7.0).ki(0.0).kd(0.0).iZone(0.0);
        p.rotation().kp(2.0).ki(0.0).kd(0.0).iZone(0.0);

        // Enable built-in autotuner publish (defaults to relay-theta tuner)
        p.autotuner();

        // Optional advanced tuning config:
        // p.autotunerConfig(a -> a
        //     .enabled(true)
        //     .dashboardPath("Athena/Localization/AutoPlannerPidAutotuner")
        //     .program(ctx -> ctx.relayTheta()));
    }))
```

When enabled, Athena publishes a `Run` command at:
`Athena/Localization/AutoPlannerPidAutotuner/Run`

Default relay-theta tuner output keys:
- `.../Suggested/Rotation/kP`
- `.../Suggested/Rotation/kI`
- `.../Suggested/Rotation/kD`
- `.../Ku`
- `.../TuSec`

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
EnumSet<VisionCameraConfig.CameraRole> preferredRoles =
        EnumSet.of(VisionCameraConfig.CameraRole.AUTO);
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
        .hooks(h -> h.stateAction(superstructure::zeroSensors, SuperState.STOW))
        .build();
```

At runtime request goals declaratively: `superstructure.getStateMachine().requestState(SuperState.SCORE_HIGH);` or continue using `queueState(...)`—the graph expands either call, inserts any intermediate states, and waits on the attached guards before each hop. See `src/main/java/ca/frc6390/athena/mechanisms/examples/ExampleSuperstructure.java` for a complete mock-up.
