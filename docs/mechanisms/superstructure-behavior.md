# Superstructure Behavior Contract

This document defines expected behavior for superstructure state propagation and guard-constrained transitions.

## Constraint and Propagation Rules

- Top-level superstructure state requests propagate into nested mechanisms/superstructures.
- Guarded paths block transitions until prerequisites are satisfied.
- Once guards are satisfied, transitions proceed through required intermediate states.

## Nested Superstructure Rules

- Nested superstructure goals and leaf mechanism goals must remain consistent with the parent requested goal.
- Leaf setpoints remain non-zero when state definitions specify non-zero setpoints.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/superstructure/ExampleSuperstructure.java`
  - `athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/superstructure/ExampleCompositeSuperstructure.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/superstructure/SuperstructureExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/superstructure/SuperstructureNestedPropagationTest.java`
