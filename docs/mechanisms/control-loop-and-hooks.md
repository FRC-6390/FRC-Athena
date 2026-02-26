# Mechanism Control Loop and Hook Behavior

This document defines Athena mechanism behavior for:

- hook disable/enable gates
- control loop disable/enable gates
- NetworkTables output/status paths for loop visibility
- unified feedforward (FF) configuration API

It is separate from realtime transport protocol work and applies to mechanism runtime behavior.

## 1. Disable APIs

Mechanism exposes three hard-disable callers:

- `disableAllHooks()`
- `disableAllControlLoops()`
- `disableAllHooksAndControlLoops()`

Equivalent section callers exist:

- `control().disableAllHooks()`
- `control().disableAllControlLoops()`
- `control().disableAllHooksAndControlLoops()`
- `loops().disableAll()`

## 2. Required Disable Semantics

Calling `disableAllControlLoops()` MUST:

1. set `controlLoopsEnabled=false`
2. set `pidEnabled=false`
3. set `feedforwardEnabled=false`
4. disable all named loops
5. disable custom PID/control cycle execution
6. reset loop runners and cached loop outputs to `0`

Calling `disableAllHooks()` MUST:

- set `hooksEnabled=false`
- stop periodic hook execution
- stop state-machine hook execution pathways while disabled

All disable APIs are idempotent.

## 3. PID and FF Gating Rules

When PID is disabled, any PID helper output MUST be `0`.

When feedforward is disabled, any feedforward helper output MUST be `0`.

This includes named periodic/custom loops using helper context calls so disabled controllers cannot continue contributing stale outputs.

## 4. Loop Enable Rules

Global and per-loop gates both apply.

A loop is effectively enabled only when:

- global `controlLoopsEnabled=true`
- loop name is not in the disabled-loop set

Enabling an individual loop re-enables the global loop subsystem if it was disabled.

## 5. NetworkTables Paths

Legacy aggregate NT output keys are removed:

- `/Control/Output/pid`
- `/Control/Output/feedforward`

Per-loop outputs are published instead under:

- `/Control/Output/Loops/<loopName>/enabled` (`boolean`)
- `/Control/Output/Loops/<loopName>/value` (`double`)
- `/Control/Output/Loops/<loopName>/periodMs` (`double`)

Mechanism status includes:

- `/Control/Status/hooksEnabled` (`boolean`)
- `/Control/Status/controlLoopsEnabled` (`boolean`)

## 6. Unified Feedforward API

Old per-mechanism FF profile APIs are removed (arm/elevator-specific profile entry points).

Use unified control config:

- `ff(name, builder -> builder.simple(...))`
- `ff(name, builder -> builder.arm(...))`
- `ff(name, builder -> builder.elevator(...))`

Control loops can declare their runtime input source:

- `pid(name, p -> p.source(InputSource.position))` (or `velocity`, `setpoint`, `input("key")`)
- `bangBang(name, b -> b.source(...))`
- `ff(name, f -> f.source(...))`
- Profiled PID is configured via `pid(name, p -> p.profiled(maxVelocity, maxAcceleration))`.
- Built-in mechanism PID measurement/controller paths are removed; PID is run through named control loops only.
- Built-in mechanism feedforward output paths are removed; FF output is produced through named control loops only.

Default sources are:

- PID: `position`
- Bang-bang: `position`
- Feedforward: `velocity`

Feedforward profile type enum:

- `SIMPLE`
- `ARM`
- `ELEVATOR`

## 7. Migration Notes

Replace old FF declarations with unified `ff(...)` builder calls.

Example:

```java
cfg.control(c -> c.ff("ff", ff -> ff.arm(0.2, 0.4, 0.6, 0.8).tolerance(0.05)));
```

For mechanism defaults, use named entries like `ff`, `feedforward`, `default`, or `main` so mechanism constructors can resolve the intended profile type.
