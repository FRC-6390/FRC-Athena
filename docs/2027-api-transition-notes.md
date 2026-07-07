# 2027 API Transition Notes

## Direction

The new Athena direction is refs plus introspection:

- robot code declares real objects, refs, states, hooks, rules, sims, and children as fields
- Athena discovers those fields
- Athena validates and runs the discovered model
- user code should not need to manually return `spec()` or call `toSpec()`

The older config/spec path still exists while the new systems are being built:

- `MechanismConfig`
- `Drivetrains.swerve(...)`
- `SwerveDrivetrainConfig`
- `SwerveModuleConfig`
- public `spec()` / `toSpec()` authoring
- `ImuId`

Do not remove those until the new mechanism, drivetrain, localization, controls, and auto paths are in place. Once the new paths cover the required behavior, purge the old user-facing config/spec APIs instead of maintaining compatibility layers.

Internal normalized definitions may still exist for validation/runtime adapters, but they should be produced by introspection, not manually authored by users.

## Swerve Module Open Questions

Swerve modules should be treated as specialized mechanisms. They need stricter role semantics than generic mechanisms because kinematics and validation depend on module identity and ordering.

Rejected or weak options:

- role inferred only from field/class name: too implicit for a focused drivetrain system
- constructor role such as `new Module(FRONT_LEFT)`: adds initialization ceremony and can disagree with names
- four fixed module types only: too restrictive because 3+ module swerve layouts are valid

Promising direction:

- module classes extend a known module model, such as `SwerveModules.SDS.MK5N.R3`
- module order is supplied by `@SwerveModuleOrder(0)`, with declaration order as the fallback
- module location is either declared directly with `ModuleLocationRef` or derived from `TrackWidth` plus `WheelBase` for a four-module rectangular drivebase
- arbitrary 3+ module layouts are valid, but every module needs an explicit location unless Athena can derive one from the drivebase geometry
- module fields still declare their own `MotorRef`, `EncoderRef`, PID/feedforward refs, offsets, and overrides

Open design issue:

- preserve the old "one module pattern, here are the IDs/offsets for the rest" speed without reintroducing global defaults or hiding module hardware far away from each module declaration

Possible solution to test later:

- a local base class inside the drivetrain for repeated hardware pattern
- each concrete nested module supplies IDs, offset, and optional role
- weird modules override only the pieces that differ

Current test direction:

- drivebase introspection can construct nested module classes directly, so the drivetrain does not need `frontLeft = new FrontLeft()` fields unless robot code explicitly wants handles
- module order is still explicit with `@SwerveModuleOrder`, because source/reflection order is not a strong enough contract for drivetrain kinematics
- `ControlLoopRef` is the control binding: output motor, followers, feedback refs, extra refs, and PID/FF/CRT/custom methods
- states/requests choose the demand: percent, voltage, position, velocity
- percent and voltage are open-loop bypasses even when emitted through a `ControlLoopRef`; position and velocity are closed-loop demands that carry the loop binding
- swerve modules should declare separate drive and steer control loops, not pretend the whole module is one scalar mechanism

Still open:

- decide whether known swerve module models should validate required loop names such as `driveControl` and `steerControl`, or use a small role annotation if names feel too implicit
- decide how loop metadata maps onto vendor motor-controller features at runtime
- decide how much of `AxisRef` remains after `ControlLoopRef` proves itself; avoid carrying two overlapping concepts longer than needed

## Locked Low-Level Ref Direction

- `MotorRef` is hardware identity/config plus direct access to its integrated `EncoderRef`
- `EncoderRef` owns sensor source, signal type, gear ratio, wheel conversion, offset, and final unit meaning
- use `encoder.gearRatio(...)`, `encoder.wheelDiameterMeters(...)` / `wheelDiameterInches(...)`, `encoder.offset(...)`, and `encoder.units(...)`
- do not put gear ratio, wheel diameter, or unit conversion on `AxisRef` or future control APIs
- do not create multiple encoder refs for raw/geared/unit-converted variants unless there are actually multiple physical sensors
