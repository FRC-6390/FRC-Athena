# athena-auto

Autonomous routine registry and selection model for Athena 2027.

This module does not depend on PathPlanner, Choreo, or WPILib chooser classes.
Vendor/path-tool adapters should register auto sources by key and return generic
`CommandSpec` descriptors.

## Current Slice

- Auto source registry with clear missing dependency guidance.
- Auto routine metadata and immutable routine specs.
- Auto chooser validation requiring registered routines and valid defaults.
- Prepared execution view that returns the selected command descriptor.
- Scoped auto inputs for passing typed values between prepared routines.

## Dependencies

- Production: `athena-commands`, `athena-runtime`.
- Test-only: none.
