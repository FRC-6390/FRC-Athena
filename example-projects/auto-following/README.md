# Autonomous and driver-assist examples

This project is a catalog, not one giant competition auto. Each selectable auto is implemented in
its own Java file and registered by `AutoExamples`.

## Complexity ladder

| File | What it demonstrates |
| --- | --- |
| `PathPlannerSimpleAuto` | One PathPlanner `.auto` file through `PathPlannerPathProvider` |
| `PathPlannerMarkersAuto` | PathPlanner event markers through `NamedCommands` |
| `PathPlannerMultiPathAuto` | Several path files and mechanism commands sequenced in Java |
| `ConditionalPathPlannerAuto` | Sensor-dependent scoring and lane choices evaluated at the branch |
| `DynamicPathPlannerAuto` | A short `PathPlannerPath` generated from live localization |
| `ChoreoSimpleAuto` | One Choreo trajectory through `ChoreoPathAdapter` |
| `ChoreoMarkersAuto` | Choreo events through `AutoFactory.bind` |
| `ChoreoMultiPathSplitAuto` | Multiple trajectories, named events, and split indices |
| `ConditionalChoreoAuto` | A Choreo routine that selects its next trajectory at runtime |
| `CustomProviderMarkersAuto` | Athena `PathGraph` metadata driven by `ExampleMarkerPathProvider` |
| `TeleopAssistRoutines` | Pathfinding, live-target paths, heading assist, and a scoring mini-auto |

The mechanism and drivetrain classes are deliberately small integration boundaries. Replace their
method bodies with the real robot outputs; keep the auto classes focused on orchestration.

## GUI assets expected by the examples

Create these in PathPlanner under `src/main/deploy/pathplanner`:

- Autos: `PP-Leave`, `PP-Markers-And-States`, `PP-To-Center-Piece`,
  `PP-Center-To-Score`, `PP-Exit`, `PP-Center-Lane`, and `PP-Safe-Lane`.
- Named commands used by markers: `pp-prepare-score`, `pp-score`, `pp-intake`, and `pp-stow`.
- Configure the robot in the PathPlanner GUI so `pathplanner/settings.json` is deployed.

Create these in Choreo under `src/main/deploy/choreo`:

- Trajectories: `Choreo-Leave`, `Choreo-Markers-And-States`, `Choreo-Two-Piece`,
  `Choreo-Exit`, `Choreo-Collect`, `Choreo-Return-To-Score`, and `Choreo-Safe-Exit`.
- Put at least two splits in `Choreo-Two-Piece`.
- Events: the four `choreo-*` names registered by `ChoreoMarkersAuto`, plus
  `deploy-intake` on split 0.

Generated GUI files are season/tool-version-specific, so this catalog names the expected assets
instead of checking in fake or stale trajectory JSON.

## Marker ownership

There are three marker systems in the sample, and they are intentionally not presented as one:

- PathPlanner follows its own path timing and dispatches `NamedCommands`.
- Choreo follows its own trajectory timing and dispatches factory bindings or routine triggers.
- A custom Athena `PathProvider` can report its events to `PathGraph.trigger(name)`. The example
  provider keeps calling `trigger` while each marker command is active, so multi-cycle markers finish.

`Autos.marker(...)` records Athena marker metadata; it does not automatically connect a vendor
event stream to `PathGraph`. A future bridge could normalize these, but doing it implicitly today
would risk firing a command twice.

## Teleop usage

`TeleopAssistRoutines` returns ordinary WPILib commands. Bind them to `onTrue`, `whileTrue`, or
another control policy. The heading example owns the drivetrain while active but continuously mixes
the driver's translation with automatic rotation. The pathfinding and generated-path examples own
all drivetrain axes and should cancel immediately when the driver releases the assist control.

The distinction matters: a full path follower and a default drive command both require the same
subsystem, so they cannot run concurrently merely by putting them in parallel. Shared control needs
axis-level intent arbitration, which is the focus of `docs/simple-auto-api-sketch.md`.

## Running the sample

From the repository root, publish Athena locally and compile the standalone example:

```powershell
./gradlew.bat compileAutoFollowingExample
```

The default selection is `PP 1 - Leave`. It will only execute successfully when its PathPlanner
assets and GUI robot settings have been deployed.
