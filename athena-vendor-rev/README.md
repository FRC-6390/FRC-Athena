# athena-vendor-rev

REV adapter for Athena 2027.

This module owns REV-specific option types and backend implementations. It is
selected by the Athena Gradle plugin when REV dependencies are detected.

## Current Slice

- `RevMotorOptions` provides typed REV-specific motor configuration.
- `RevMotorBackend` supports Spark MAX and Spark Flex motor keys.
- `RevMotorDevice` creates REVLib `SparkMax` or `SparkFlex` controllers and
  writes percent output, voltage, stop, idle mode, current limit, and ramp
  configuration.
- `RevMotorDevice` reads Spark integrated encoder position and velocity through
  REVLib.
- `RevMotorDevice` reads controller-attached Spark absolute encoder position
  and velocity through REVLib.
- `RevEncoderBackend` supports standalone REV through-bore encoders connected
  to a DIO duty-cycle input.
- `RevThroughBoreEncoderDevice` reads absolute position through WPILib
  `DutyCycleEncoder`.
- ServiceLoader metadata exposes motor and encoder backends to `BackendRegistry`.
- Tests verify support detection, capabilities, typed options, and REV command
  writes plus integrated, attached absolute, and standalone through-bore encoder
  reads through test controllers.

## Example

```java
MotorSpec sparkSpec = MotorConfig.create()
        .hardware(AthenaMotor.SPARK_FLEX_BRUSHLESS, 21)
        .vendor(RevMotorOptions.class, rev -> rev
                .smartCurrentLimit(50)
                .openLoopRampSeconds(0.2))
        .toSpec("arm", "pivot");

MotorDevice device = new RevMotorBackend().create(sparkSpec);
double rotations = device.integratedPositionRotations();
double absoluteRotations = device.absolutePositionRotations();

EncoderSpec throughBoreSpec = EncoderConfig.create()
        .hardware(AthenaEncoder.REV_THROUGH_BORE, 7)
        .absolutePosition()
        .toSpec("arm", "absolute");

EncoderDevice throughBore = new RevEncoderBackend().create(throughBoreSpec);
double throughBoreRotations = throughBore.absolutePositionRotations();
```

## Dependencies

- Production: `athena-hardware`.
- Production external: WPILib Java artifacts and REVLib Java.
- Test-only: none.
