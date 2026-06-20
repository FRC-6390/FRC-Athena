# athena-mechanisms

Student-facing mechanism declarations, lowering into immutable mechanism specs,
and runtime state application through generic motor devices.

The public DX should prefer:

```java
MechanismSpec spec = config.toSpec();
ValidationReport report = spec.validate();
```

## Current Slice

Mechanisms can declare motors, encoders, typed inputs, named encoder sources,
PID/feedforward control gains, named states, and a control mode. The declaration
lowers through `toSpec()` into immutable specs that validate against installed
backend capabilities and local mechanism consistency rules.

`MechanismController` applies state targets to runtime `MotorDevice` instances:
percent output states call `setPercentOutput`, position states call
`setPositionTargetRotations`, and velocity states call
`setVelocityTargetRotationsPerSecond`.

## Dependencies

- Production: `athena-api`, `athena-runtime`, `athena-hardware`.
- Test-only: none.
