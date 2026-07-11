# Tank drive

This project is a complete WPILib robot using Athena's current graph and controller APIs.

- `Robot` owns the drivetrain and controls, so Athena discovers them automatically.
- `Controls` binds a supplier-backed drive action to `Events.teleopPeriodic()`.
- `DriveTrain.arcadeDrive(...)` reads the processed axes each runtime cycle without mutable command state.
- Leaders declare brake mode and current limits; followers inherit their leaders' output requests.
- Side-specific `SimModel` declarations are discovered with the drivetrain and run automatically in simulation.

There are no manual `configure()` or `register()` calls. Add mechanisms, actions, controller hook groups,
hardware, control bindings, and simulation models as fields beneath the robot's mechanism graph.
