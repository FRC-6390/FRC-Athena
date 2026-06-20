# athena-vendor-limelight

Limelight adapter for Athena 2027.

This module is selected by the Athena Gradle plugin when Limelight dependencies
or vendordep UUIDs are detected. Limelight primarily publishes NetworkTables
values, so this adapter reads those values and translates them into Athena's
generic `VisionFrame`.

## Current Slice

- `LimelightCameraAdapter` supports Limelight camera specs.
- `LimelightCameraAdapter.latestFrame()` reads real Limelight NetworkTables
  entries from a WPILib `NetworkTableInstance`.
- `LimelightTarget` mirrors the target values Athena needs from Limelight.
- Tests verify support detection, target translation, NetworkTables reads, and
  no-target behavior.

## Example

```java
VisionFrame frame = LimelightCameraAdapter.frameFromTarget(
        LimelightTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.92));

VisionFrame live = new LimelightCameraAdapter("limelight-front").latestFrame();
```

## Dependencies

- Production: `athena-vision`.
- Production external: WPILib ntcore and Jackson core.
- Test-only: none.
