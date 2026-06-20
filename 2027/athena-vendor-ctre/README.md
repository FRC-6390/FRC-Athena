# athena-vendor-ctre

CTRE adapter for Athena 2027.

This module owns Phoenix-specific option types and backend implementations. It
is selected by the Athena Gradle plugin when CTRE dependencies or vendordep UUIDs
are detected.

## Current Slice

- `CtreMotorOptions` provides typed CTRE-specific motor configuration.
- `CtreMotorBackend` supports CTRE TalonFX, Kraken X60, and Kraken X44 motor
  keys.
- `CtreMotorDevice` creates a Phoenix 6 `TalonFX` family controller and writes
  percent output, voltage, stop, neutral mode commands, and integrated encoder
  reads.
- `CtreEncoderBackend` supports CTRE CANcoder encoder keys.
- `CtreEncoderDevice` creates a Phoenix 6 `CANcoder` and reads relative
  position, absolute position, and velocity signals.
- `CtreImuBackend` supports CTRE Pigeon2 IMU keys.
- `CtrePigeon2Device` creates a Phoenix 6 `Pigeon2` and reads yaw, pitch, and
  roll signals.
- ServiceLoader metadata exposes the motor, encoder, and IMU backends to
  `BackendRegistry`.
- Tests verify support detection, capabilities, typed options, Phoenix command
  writes, TalonFX/Kraken-family integrated encoder reads, CANcoder reads, and
  Pigeon2 reads through test controllers.

## Example

```java
MotorConfig.create()
        .hardware(AthenaMotor.KRAKEN_X60, 12)
        .vendor(CtreMotorOptions.class, ctre -> ctre
                .statorCurrentLimit(80)
                .supplyCurrentLimit(50));

EncoderConfig.create()
        .hardware(AthenaEncoder.CANCODER, 22)
        .absolutePosition();

ImuConfig.create()
        .hardware(AthenaImu.PIGEON_2, 30)
        .canbus("canivore");
```

## Dependencies

- Production: `athena-hardware`.
- Production external: WPILib Java artifacts and Phoenix 6 `wpiapi-java`.
- Test-only: none.
