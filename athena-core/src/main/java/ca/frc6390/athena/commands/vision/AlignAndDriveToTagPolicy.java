package ca.frc6390.athena.commands.vision;

import java.util.Collection;
import java.util.EnumSet;
import java.util.LinkedHashSet;
import java.util.Set;

import ca.frc6390.athena.sensors.camera.VisionCamera.TargetObservation;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig.CameraRole;

final class AlignAndDriveToTagPolicy {
    private AlignAndDriveToTagPolicy() {}

    static Set<String> sanitizeCameraTables(Collection<String> cameraTables) {
        LinkedHashSet<String> tables = new LinkedHashSet<>();
        if (cameraTables != null) {
            for (String table : cameraTables) {
                if (table == null) {
                    continue;
                }
                String trimmed = table.trim();
                if (!trimmed.isEmpty()) {
                    tables.add(trimmed);
                }
            }
        }
        if (tables.isEmpty()) {
            throw new IllegalArgumentException("At least one camera table name must be provided.");
        }
        return Set.copyOf(tables);
    }

    static EnumSet<CameraRole> sanitizeRoles(EnumSet<CameraRole> roles) {
        if (roles == null || roles.isEmpty()) {
            return EnumSet.noneOf(CameraRole.class);
        }
        return EnumSet.copyOf(roles);
    }

    static TargetObservation pickBetterObservation(TargetObservation current, TargetObservation candidate) {
        if (candidate == null) {
            return current;
        }
        if (current == null) {
            return candidate;
        }

        double currentConfidence = current.hasConfidence() ? current.confidence() : 0.0;
        double candidateConfidence = candidate.hasConfidence() ? candidate.confidence() : 0.0;
        if (candidateConfidence > currentConfidence + 1e-9) {
            return candidate;
        }
        if (currentConfidence > candidateConfidence + 1e-9) {
            return current;
        }

        double currentDistance = current.hasDistance() ? Math.abs(current.distanceMeters()) : Double.POSITIVE_INFINITY;
        double candidateDistance = candidate.hasDistance() ? Math.abs(candidate.distanceMeters()) : Double.POSITIVE_INFINITY;
        if (candidateDistance < currentDistance) {
            return candidate;
        }

        double currentMagnitude = current.hasTranslation() ? current.translation().getNorm() : Double.POSITIVE_INFINITY;
        double candidateMagnitude = candidate.hasTranslation() ? candidate.translation().getNorm() : Double.POSITIVE_INFINITY;
        if (candidateMagnitude < currentMagnitude) {
            return candidate;
        }

        return current;
    }

    static double sanitizeToleranceMeters(double value) {
        double sanitized = Math.abs(value);
        return Double.isFinite(sanitized) ? sanitized : 0.0;
    }

    static double sanitizeToleranceDegrees(double value) {
        double sanitized = Math.abs(value);
        return Double.isFinite(sanitized) ? sanitized : 0.0;
    }
}
