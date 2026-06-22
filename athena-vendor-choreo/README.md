# athena-vendor-choreo

Choreo adapter for Athena 2027.

This module is selected by the Athena Gradle plugin when Choreo dependencies or
vendordep UUIDs are detected. It owns ChoreoLib imports and bridges Choreo
trajectories into Athena's generic command model.

## Current Slice

- `ChoreoAutoSource` loads trajectory names into `CommandSpec` descriptors.
- `ChoreoAutoSource.loadTrajectory()` delegates to ChoreoLib
  `Choreo.loadTrajectory(String)` for real trajectories.
- `ChoreoAutoSource.trajectoryNames()` exposes ChoreoLib-discovered trajectory
  names.
- `ChoreoAutoFactoryAdapter` wraps Choreo `AutoFactory` trajectory, split,
  reset, routine, and warmup command creation.
- `ChoreoAutos.register()` installs the source into `AutoRegistry`.
- Tests verify command naming, source registration, trajectory loading, and
  AutoFactory command creation through test clients.

## Example

```java
ChoreoAutos.register(AutoRegistry.get());
CommandSpec command = AutoRegistry.get().require("choreo").load("LeaveCommunity");

ChoreoAutoFactoryAdapter adapter = new ChoreoAutoFactoryAdapter(autoFactory);
Command trajectory = adapter.trajectoryCommand("LeaveCommunity");
Command routine = adapter.routineCommand("TwoPiece");
```

## Dependencies

- Production: `athena-auto`.
- Production external: WPILib Java artifacts, Gson, and ChoreoLib Java.
- Test-only: none.
