# athena-hardware

Generic hardware declarations, immutable hardware specs, backend contracts, and
capability metadata.

Vendor-specific device classes do not belong in this module.

## Current Slice

- Motor declarations and immutable motor specs.
- Encoder declarations and immutable encoder specs.
- IMU declarations, immutable IMU specs, and robot-relative mount pose values.
- Typed mechanism input declarations for digital, analog, runtime, and constant
  inputs.
- Sensor wrappers for limit switches, buttons, and beam breaks.
- Backend contracts for motors, encoders, and IMUs plus motor capability
  metadata.
- Runtime motor devices expose optional integrated and controller-attached
  absolute encoder position and velocity reads.

## Test Coverage

The module has direct tests for:

- motor config lowering and alias handling
- encoder config lowering and alias handling
- IMU config lowering, alias handling, mount pose defaults, and finite checks
- typed input lowering
- sensor wrapper lowering and trigger semantics
- backend registry lookup/global replacement
- capability set immutability
- motor spec default normalization

## Dependencies

- Production: `athena-api`, `athena-runtime`.
- Test-only: none.
