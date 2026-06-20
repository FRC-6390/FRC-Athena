# athena-vendor-studica

Optional Studica/NavX adapter.

This module is not part of the default Athena vendordep. The Gradle plugin can
add it when a robot project installs Studica, or teams can enable it explicitly.

## Current Slice

- Studica/NavX IMU backend for `AthenaImu.NAVX`.
- ServiceLoader registration for the IMU backend.
- `StudicaImuDevice` creates a real Studica `AHRS` and exposes yaw, accumulated
  angle, zero, reset, and close controls through Athena-side APIs.

## Dependencies

- Production: `athena-hardware`.
- Production external: WPILib utility/math/NetworkTables artifacts and Studica
  Java.
- Test-only: none.
