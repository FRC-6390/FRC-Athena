# Native autonomous examples

This standalone robot shows Athena's autonomous API across Choreo, PathPlanner, and a custom generated
path provider. Selectable autos remain ordinary Athena `Action` trees and each auto lives in one file.

| File | What it demonstrates |
| --- | --- |
| `ChoreoMarkersAuto` | Bind Choreo event names directly to mechanism Actions |
| `ChoreoMultiPathSplitAuto` | Sequence split points, a second path, and mechanism Actions |
| `ConditionalChoreoAuto` | Choose the next path from live robot state at the branch point |
| `PathPlannerAuto` | Run a PathPlanner `.auto` through `PathPlannerPathProvider` and `AutoBuilder` |
| `GeneratedPathAuto` | Run and preview a vendor-neutral path owned by a team `PathProvider` |
| `PathPlannerSetup` | Convert PathPlanner chassis-speed callbacks into normal Athena drive Actions |
| `CommandInteropExamples` | Wrap and schedule a WPILib command through Athena's `CommandGraph` |

`ExampleDrive` owns a `RobotVelocityPool`, its autonomous channel, and one shared `pooledDrive` action.
The native follower publishes Choreo output through that channel, PathPlanner updates the same channel,
and path completion clears it. `PathPlannerSetup` configures PathPlanner's global `AutoBuilder` and requests
the shared Athena drive Action from its output callback. `GeneratedPathProvider` demonstrates the provider extension contract, including
runtime output and dashboard preview geometry. The Robot declares one `AutoChooser`; Athena discovers the
chooser and all three providers, publishes the selection and preview, and binds nested paths automatically.

Create these trajectories under `src/main/deploy/choreo` before executing the example:

- `Choreo-Markers-And-States`, with any desired `choreo-prepare-score`, `choreo-score`,
  `choreo-intake`, and `choreo-stow` events.
- `Choreo-Two-Piece`, with at least two splits, and `Choreo-Exit`.
- `Choreo-Collect`, `Choreo-Return-To-Score`, and `Choreo-Safe-Exit`.

Use the PathPlanner GUI to create `PathPlanner-Two-Piece.auto` and its referenced paths under
`src/main/deploy/pathplanner`. The GUI-generated `settings.json` supplies the physical `RobotConfig` used by
`PathPlannerSetup`. The generated straight-line example needs no deploy file.

Publish Athena locally and compile the standalone project from the repository root:

```sh
./gradlew compileAutoFollowingExample
```

The action passed to `defaultAuto(...)` is selected initially. Its first path name becomes the
chooser label automatically; explicit string labels remain optional. Dashboard changes update the
next selection and preview only; Athena freezes that selection when autonomous begins.
