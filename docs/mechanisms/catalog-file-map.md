# Mechanisms File Catalog

## V2 Public API (`athena-core/.../api/mechanism`)

- `MechanismConfig`, `StatefulMechanismConfig`: structured authoring roots.
- `FlowMechanismConfig`, `StatefulFlowMechanismConfig`, `Mechanisms`: flow authoring roots.
- `MechanismDefinitions`: annotation loading, structured lowering, validation, and direct build.
- `annotation/*`: mechanism, identity, motor, encoder, input, loop, and automation annotations.
- `motor/*`, `encoder/*`, `input/*`, `identity/*`: reusable V2 section objects.
- `behavior/*`: PID, feedforward, bang-bang, custom loops, and state automation.
- `definition/*`: immutable lowered model shared by all authoring styles.
- `introspection/*`, `validation/*`, `runtime/*`: annotation loading, validation, and runtime lowering.

## Core Runtime (`athena-core/.../mechanisms/base`)

- `Mechanism`: primary mechanism runtime/control loop implementation.
- `MechanismRuntimeConfig`, `MechanismLifecycleHooks`: runtime-owned mechanism setup.
- `MechanismContext`, `MechanismControlContext`: runtime contexts exposed to hooks and loops.
- `MechanismConfigIO`, `MechanismConfigRecord`: record-based config snapshot/apply path.
- `OutputType`, `OutputConversions`, `MechanismTravelRange`: output and bounds helpers.

## Typed Runtime Wrappers (`.../mechanisms/types`)

- `ArmMechanism`
- `ElevatorMechanism`
- `FlywheelMechanism`
- `SimpleMotorMechanism`
- `TurretMechanism`

## Superstructure Runtime (`.../mechanisms/superstructure`)

- `SuperstructureMechanism`: parent composition runtime.
- `SuperstructureRuntimeConfig`, `SuperstructureLifecycleHooks`: runtime-owned superstructure setup.
- `SuperstructureContext`, `SuperstructureMechanismsView`: context and child lookup helpers.

## State Runtime (`.../mechanisms/state`)

- `StateMachine`, `StateGraph`, `StatefulMechanism`, `StatefulMechanismRuntimeConfig`, `StatefulLike`

## State DSL (`.../mechanisms/statespec`)

- `StateDsl`, `AthenaState`, `AthenaStateLogic`
- `StateBuilder`, `StateCtx`, `TransitionDirective`
- `StateSeed`, `StateSeedProvider`, `StateSeedRuntime`

## Config Export and Schema (`.../mechanisms/config`)

- `MechanismConfigExport`
- `MechanismConfigFile` and related `Mechanism*Config` schema records
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
