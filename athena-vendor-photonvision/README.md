# athena-vendor-photonvision

PhotonVision adapter for Athena 2027.

This module is selected by the Athena Gradle plugin when PhotonVision
dependencies or vendordep UUIDs are detected. It owns PhotonLib imports and
translates real PhotonVision pipeline results into Athena's generic
`VisionFrame` model.

## Current Slice

- `PhotonVisionCameraAdapter` supports PhotonVision camera specs and can read
  unread frames from a real PhotonLib `PhotonCamera`.
- Real `PhotonPipelineResult` and `PhotonTrackedTarget` values are converted to
  Athena `VisionObservation` values.
- `PhotonVisionTarget` mirrors the target values Athena needs from
  PhotonVision for examples and test fixtures.
- Tests verify support detection, target translation, no-target behavior, and
  real PhotonLib target/result conversion.

## Example

```java
VisionFrame frame = PhotonVisionCameraAdapter.frameFromTargets(List.of(
        PhotonVisionTarget.aprilTag(7, -3.2, -1.5, 2.4, 0.92)));
```

## Dependencies

- Production: `athena-vision`.
- Production external: WPILib utility/units/math/NetworkTables/WPILibJ,
  Jackson annotations, QuickBuffers runtime, and PhotonLib Java artifacts.
- Test-only: none.
