# Mechanisms Source Catalog

This directory is physically organized by concern while preserving the package
`ca.frc6390.athena.mechanisms` for API stability.

## Folder Map

- `base/`: core mechanism runtime, config DSL, common context/types.
- `sections/`: extracted fluent sections (`sysId`, `sim`, `networkTables`).
- `types/`: concrete mechanism types (`Arm`, `Elevator`, `Flywheel`, etc.).
- `superstructure/`: composition and superstructure orchestration.
- `state/`: generic state machine/stateful runtime classes.
- `autotune/`: PID autotuning contexts/programs.
- `visualization/`: default mechanism visualization helpers.
- `config/`: config-file schemas/loader/export/applier.
- `sim/`: simulation model + visualization plumbing.
- `statespec/`: state DSL annotation/runtime interfaces.
- `registration/`: registration/factory interfaces.

## Primary Entry Points

- `base/Mechanism.java`
- `base/MechanismConfig.java`
- `types/*.java`
- `superstructure/SuperstructureMechanism.java`
- `statespec/StateDsl.java`
