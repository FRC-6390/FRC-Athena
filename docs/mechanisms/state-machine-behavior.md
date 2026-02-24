# State Machine and Guard Behavior Contract

This document defines expected behavior for `StateMachine` with `StateGraph`-expanded transitions.

## Core Rules

- `force(target)` expands through graph-defined intermediate states.
- `guard(from, to, condition)` must pass before that transition is consumed.
- `force(target, append=true)` appends instead of replacing queued states.
- `update()` consumes at most one queued transition per call.

## Queue Semantics

- If no graph is installed, force queues direct target states.
- If graph path is unavailable, machine falls back to direct target.
- `getNextStateQueue()` lists pending queue states in order.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/state/StateMachineExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/state/StateMachineExamplesTest.java`
