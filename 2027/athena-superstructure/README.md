# athena-superstructure

Coordination layer for multiple mechanisms.

This module models named mechanism parts and superstructure states without
forcing fake mechanism states into stateless child mechanisms.

## Current Slice

- Named superstructure parts backed by mechanism declarations or nested
  superstructures.
- Named superstructure states that map each part to a mechanism state.
- Runtime transition planning that resolves nested superstructure states into
  ordered leaf mechanism targets.
- Runtime superstructure controller that applies planned targets through
  registered `MechanismController` instances.
- Command factories for one-shot state application and stop commands.
- Guard hooks for rejecting unsafe transitions before robot code executes the
  returned targets.
- Validation for duplicate parts, duplicate states, missing parts, and missing
  child state targets.

## Dependencies

- Production: `athena-commands`, `athena-mechanisms`.
- Test-only: `athena-simulation`.
