# Mechanisms File Catalog

## Core Runtime (`athena-core/.../mechanisms/base`)

- `Mechanism`: primary mechanism runtime/control loop implementation.
- `MechanismConfig`: builder/DSL for composing mechanism behavior.
- `MechanismContext`, `MechanismControlContext`: runtime contexts.
- `MechanismGroup`: bulk grouping helper.
- `MechanismConfigIO`, `MechanismConfigRecord`: config IO bridge.
- `MechanismLimitSwitchConfig`, `MechanismTravelRange`: safety/config helpers.
- `OutputType`, `OutputConversions`: output-space behavior.

## Focused Sections (`.../mechanisms/sections`)

- `MechanismSysIdSection`: SysId config/trigger binding API.
- `MechanismSimulationSection`: simulation helpers.
- `MechanismNetworkTablesSection`: NT publishing toggles/path control.

## Typed Mechanisms (`.../mechanisms/types`)

- `ArmMechanism`
- `ElevatorMechanism`
- `FlywheelMechanism`
- `SimpleMotorMechanism`
- `TurretMechanism`

## Superstructure (`.../mechanisms/superstructure`)

- `SuperstructureMechanism`: parent composition runtime.
- `SuperstructureConfig`, `SuperstructureContext`: setup + runtime context.
- `SuperstructureMechanismsView`: sub-mechanism view helpers.

## State Runtime (`.../mechanisms/state`)

- `StateMachine`, `StateGraph`, `StatefulMechanism`, `StatefulLike`

## State DSL (`.../mechanisms/statespec`)

- `StateDsl`, `AthenaState`, `AthenaStateLogic`
- `StateBuilder`, `StateCtx`, `TransitionDirective`
- `StateSeed`, `StateSeedProvider`, `StateSeedRuntime`

## Config File Model (`.../mechanisms/config`)

- `MechanismConfigLoader`, `MechanismConfigApplier`, `MechanismConfigExport`
- `Mechanism*Config` schema records
- `AthenaTomlWriter`

## Simulation (`.../mechanisms/sim`)

- `MechanismSimulationModel`, `MechanismSimulationConfig`
- `MechanismSensorSimulation`, `MechanismSensorSimulationConfig`
- `MechanismVisualization`, `MechanismVisualizationConfig`, `MechanismSimUtil`

## Autotuning (`.../mechanisms/autotune`)

- `MechanismPidAutotuners`, `MechanismPidAutotunerContext`, `MechanismPidAutotunerProgram`

## Registration (`.../mechanisms/registration`)

- `RegisterableMechanism`, `RegisterableMechanismFactory`

## Visualization Helpers (`.../mechanisms/visualization`)

- `MechanismDefaultVisualization`
- `MechanismVisualizationDefaults`
- `ElevatorMechanismVisualization`
