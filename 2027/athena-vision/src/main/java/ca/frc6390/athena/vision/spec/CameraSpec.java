package ca.frc6390.athena.vision.spec;

import java.util.Objects;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.runtime.validation.ValidationReport;

/**
 * Immutable normalized camera declaration.
 *
 * @param ownerPath owner path
 * @param name local camera name
 * @param kind camera kind
 * @param cameraName camera or NetworkTables name
 * @param mountPose robot-relative mount pose
 */
public record CameraSpec(
        String ownerPath,
        String name,
        CameraKind kind,
        String cameraName,
        CameraMountPose mountPose) {
    public CameraSpec {
        ownerPath = ownerPath == null || ownerPath.isBlank() ? "vision" : ownerPath;
        name = name == null || name.isBlank() ? "camera" : name;
        Objects.requireNonNull(kind, "kind");
        cameraName = cameraName == null || cameraName.isBlank() ? "camera" : cameraName;
        mountPose = mountPose == null ? CameraMountPose.identity() : mountPose;
    }

    /**
     * Returns a dotted path useful in validation errors.
     *
     * @return camera path
     */
    public String path() {
        return ownerPath + "." + name;
    }

    /**
     * Validates camera declaration values.
     *
     * @return validation report
     */
    public ValidationReport validate() {
        ValidationReport.Builder report = ValidationReport.builder();
        if (!mountPose.isFinite()) {
            report.error("vision.invalid-mount-pose", path(), "Camera mount pose must be finite.");
        }
        return report.build();
    }
}
