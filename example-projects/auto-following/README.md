# Native autonomous examples

This standalone robot shows Athena's autonomous API without a second command scheduler. Every auto
is an ordinary `Action`, every path is a `PathAction`, and each selectable auto lives in one file.

| File | What it demonstrates |
| --- | --- |
| `ChoreoMarkersAuto` | Bind Choreo event names directly to mechanism Actions |
| `ChoreoMultiPathSplitAuto` | Sequence split points, a second path, and mechanism Actions |
| `ConditionalChoreoAuto` | Choose the next path from live robot state at the branch point |

`ExampleDrive` owns the follower through `kinematics.follow(...)`; `AutoContext` only connects that
follower to Choreo. It does not construct Choreo `AutoFactory`, adapt WPILib commands, or run
another scheduler. `AutoExamples` only registers the one-file routines. Translation and heading
gains are ordinary Athena gain objects, so their existing mechanism telemetry/runtime overrides
are discovered automatically.

Create these trajectories under `src/main/deploy/choreo` before executing the example:

- `Choreo-Markers-And-States`, with any desired `choreo-prepare-score`, `choreo-score`,
  `choreo-intake`, and `choreo-stow` events.
- `Choreo-Two-Piece`, with at least two splits, and `Choreo-Exit`.
- `Choreo-Collect`, `Choreo-Return-To-Score`, and `Choreo-Safe-Exit`.

Publish Athena locally and compile the standalone project from the repository root:

```sh
./gradlew compileAutoFollowingExample
```

The first registered routine is selected by default. A dashboard chooser can call
`AutoRuntime.select(name)` during autonomous init when multiple routines are deployed.
