package ca.frc6390.athena.core.localization;

import java.util.List;

import ca.frc6390.athena.sensors.camera.VisionCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.Timer;

final class RobotLocalizationVisionUtil {
    private RobotLocalizationVisionUtil() {}

    static Matrix<N3, N1> sanitizeVisionStdDevs(
            Matrix<N3, N1> stdDevs,
            double translationLimit,
            double rotationLimit,
            double stdEpsilon) {
        if (stdDevs == null) {
            return null;
        }
        double sanitizedX = sanitizeStdDevEntry(stdDevs.get(0, 0), translationLimit, stdEpsilon);
        double sanitizedY = sanitizeStdDevEntry(stdDevs.get(1, 0), translationLimit, stdEpsilon);
        double sanitizedTheta = sanitizeStdDevEntry(stdDevs.get(2, 0), rotationLimit, stdEpsilon);
        if (Double.isNaN(sanitizedX) || Double.isNaN(sanitizedY) || Double.isNaN(sanitizedTheta)) {
            return null;
        }
        return VecBuilder.fill(sanitizedX, sanitizedY, sanitizedTheta);
    }

    static double sanitizeStdDevEntry(double value, double maxValue, double stdEpsilon) {
        double sanitized = Math.abs(value);
        if (!Double.isFinite(sanitized) || sanitized < stdEpsilon) {
            return Double.NaN;
        }
        if (maxValue > 0.0 && sanitized >= maxValue) {
            return Double.NaN;
        }
        return sanitized;
    }

    static boolean shouldApplyVisionMeasurement(
            Pose2d visionPose,
            double timestampSeconds,
            Matrix<N3, N1> stdDevs,
            boolean hasAcceptedVisionMeasurement,
            double lastVisionMeasurementTimestamp,
            double minVisionUpdateSeparationSeconds,
            double visionMaxLatencySeconds,
            Pose2d referencePose,
            double visionOutlierTranslationMeters,
            double visionOutlierRotationRadians,
            double visionStdDevOutlierMultiplier,
            double stdEpsilon) {
        if (!isFinitePose(visionPose)) {
            return false;
        }
        if (!Double.isFinite(timestampSeconds)) {
            return false;
        }
        if (hasAcceptedVisionMeasurement) {
            if (timestampSeconds <= lastVisionMeasurementTimestamp) {
                return false;
            }
            if (minVisionUpdateSeparationSeconds > 0.0
                    && (timestampSeconds - lastVisionMeasurementTimestamp) < minVisionUpdateSeparationSeconds) {
                return false;
            }
        }
        double ageSeconds = Timer.getFPGATimestamp() - timestampSeconds;
        if (visionMaxLatencySeconds > 0.0) {
            if (ageSeconds > visionMaxLatencySeconds || ageSeconds < -0.1) {
                return false;
            }
        }
        if (!hasAcceptedVisionMeasurement) {
            return true;
        }
        Pose2d reference = referencePose != null ? referencePose : new Pose2d();
        Pose2d delta = visionPose.relativeTo(reference);
        double translationError = delta.getTranslation().getNorm();
        double rotationError = Math.abs(delta.getRotation().getRadians());

        double translationThreshold = visionOutlierTranslationMeters;
        double rotationThreshold = visionOutlierRotationRadians;
        if (stdDevs != null) {
            double translationStd = Math.max(stdDevs.get(0, 0), stdDevs.get(1, 0));
            double rotationStd = stdDevs.get(2, 0);
            if (Double.isFinite(translationStd) && translationStd > stdEpsilon) {
                translationThreshold = Math.max(translationThreshold, translationStd * visionStdDevOutlierMultiplier);
            }
            if (Double.isFinite(rotationStd) && rotationStd > stdEpsilon) {
                rotationThreshold = Math.max(rotationThreshold, rotationStd * visionStdDevOutlierMultiplier);
            }
        }

        return translationError <= translationThreshold && rotationError <= rotationThreshold;
    }

    static Matrix<N3, N1> adjustVisionStdDevsForDisagreement(
            Matrix<N3, N1> stdDevs,
            Pose2d visionPose,
            Pose2d referencePose,
            double visionOutlierTranslationMeters,
            double visionOutlierRotationRadians,
            double translationLimit,
            double rotationLimit,
            double stdEpsilon) {
        if (stdDevs == null) {
            return null;
        }
        Pose2d reference = referencePose != null ? referencePose : new Pose2d();
        Pose2d delta = visionPose.relativeTo(reference);
        double translationError = delta.getTranslation().getNorm();
        double rotationError = Math.abs(delta.getRotation().getRadians());

        double translationDenominator = Math.max(visionOutlierTranslationMeters, stdEpsilon);
        double rotationDenominator = Math.max(visionOutlierRotationRadians, stdEpsilon);

        double translationScale = 1.0 + Math.pow(translationError / translationDenominator, 2.0);
        double rotationScale = 1.0 + Math.pow(rotationError / rotationDenominator, 2.0);

        translationScale = Math.min(translationScale, 6.0);
        rotationScale = Math.min(rotationScale, 6.0);

        double scaledX = clampStdDev(stdDevs.get(0, 0) * translationScale, translationLimit);
        double scaledY = clampStdDev(stdDevs.get(1, 0) * translationScale, translationLimit);
        double scaledTheta = clampStdDev(stdDevs.get(2, 0) * rotationScale, rotationLimit);
        return VecBuilder.fill(scaledX, scaledY, scaledTheta);
    }

    static double clampStdDev(double value, double limit) {
        if (limit > 0.0) {
            return Math.min(value, limit);
        }
        return value;
    }

    static Matrix<N4, N1> expandStdDevsTo3d(Matrix<N3, N1> stdDevs, double zStd) {
        if (stdDevs == null) {
            return null;
        }
        return VecBuilder.fill(stdDevs.get(0, 0), stdDevs.get(1, 0), zStd, stdDevs.get(2, 0));
    }

    static Matrix<N3, N1> scaleStdDevs(Matrix<N3, N1> stdDevs, double scale) {
        if (stdDevs == null) {
            return null;
        }
        return VecBuilder.fill(
                stdDevs.get(0, 0) * scale,
                stdDevs.get(1, 0) * scale,
                stdDevs.get(2, 0) * scale);
    }

    static boolean hasPoseAgreement(
            List<VisionCamera.VisionMeasurement> measurements,
            double agreementMeters) {
        if (measurements == null || measurements.size() < 2 || agreementMeters <= 0.0) {
            return false;
        }
        for (int i = 0; i < measurements.size(); i++) {
            Pose2d poseA = measurements.get(i).pose2d();
            if (poseA == null) {
                continue;
            }
            for (int j = i + 1; j < measurements.size(); j++) {
                Pose2d poseB = measurements.get(j).pose2d();
                if (poseB == null) {
                    continue;
                }
                double distance =
                        poseA.getTranslation().getDistance(poseB.getTranslation());
                if (distance <= agreementMeters) {
                    return true;
                }
            }
        }
        return false;
    }

    private static boolean isFinitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }
}
