# athena-vendor-pathplanner

PathPlanner adapter for Athena 2027.

This module is selected by the Athena Gradle plugin when PathPlanner
dependencies or vendordep UUIDs are detected. It owns PathPlannerLib imports
and bridges PathPlanner autos into Athena's generic command model or real
WPILib commands.

## Current Slice

- `PathPlannerAutoSource` loads path names into `CommandSpec` descriptors.
- `PathPlannerAutoSource.loadCommand()` delegates to PathPlannerLib
  `AutoBuilder.buildAuto(String)` for real WPILib commands.
- `PathPlannerAutoSource.autoNames()` exposes PathPlannerLib-discovered auto
  names.
- `PathPlannerAutos.register()` installs the source into `AutoRegistry`.
- Tests verify command naming, source registration, PathPlanner command loading,
  and auto-name discovery through a test client.

## Example

```java
PathPlannerAutos.register(AutoRegistry.get());
CommandSpec command = AutoRegistry.get().require("pathplanner").load("LeaveCommunity");
```

## Dependencies

- Production: `athena-auto`.
- Production external: WPILib Java artifacts, Jackson annotations, and
  PathPlannerLib Java.
- Test-only: none.
