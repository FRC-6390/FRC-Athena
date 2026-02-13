package ca.frc6390.athena.core.examples;

import java.util.Objects;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.localization.PoseBoundingBox2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Hook-based RobotCore examples for driver assist behaviors.
 */
public final class RobotCoreHookExamples {
    private RobotCoreHookExamples() {}

    /**
     * Adds a robot-periodic driver assist that, while inside {@code zone}:
     *
     * <p>1) Centers the robot on the zone's X midpoint (field frame)<br>
     * 2) Locks heading to either 90 or 270 degrees (chosen on zone entry)<br>
     * 3) Does not command travel along the lane direction (field Y stays zero)
     */
    public static <T extends RobotDrivetrain<T>> RobotCore.RobotCoreConfig<T> withBoundingBoxCenterLineAssist(
            RobotCore.RobotCoreConfig<T> config,
            PoseBoundingBox2d zone) {
        return withBoundingBoxCenterLineAssist(config, zone, BoundingBoxCenterLineAssistTuning.defaults());
    }

    public static <T extends RobotDrivetrain<T>> RobotCore.RobotCoreConfig<T> withBoundingBoxCenterLineAssist(
            RobotCore.RobotCoreConfig<T> config,
            PoseBoundingBox2d zone,
            BoundingBoxCenterLineAssistTuning tuning) {
        Objects.requireNonNull(config, "config");
        Objects.requireNonNull(zone, "zone");
        Objects.requireNonNull(tuning, "tuning");

        LaneAssistState state = new LaneAssistState();

        return config.hooks(h -> {
            h.onInit(ctx -> {
                RobotSpeeds speeds = ctx.drivetrain().getRobotSpeeds();
                speeds.registerSpeedSource(tuning.speedSource(), tuning.speedSourcePriority());
                speeds.setSpeedSourceState(tuning.speedSource(), true);
            });

            // This runs from RobotCore.robotPeriodic() (phase ROBOT_PERIODIC).
            h.onPeriodic(ctx -> {
                RobotSpeeds speeds = ctx.drivetrain().getRobotSpeeds();
                if (!DriverStation.isTeleopEnabled()) {
                    stopAssist(speeds, tuning, state);
                    return;
                }

                if (ctx.localization() == null) {
                    stopAssist(speeds, tuning, state);
                    return;
                }

                Pose2d pose = ctx.localization().getFieldPose();
                boolean inZone = zone.contains(pose);
                if (!inZone) {
                    stopAssist(speeds, tuning, state);
                    return;
                }

                if (!state.inZone) {
                    state.targetHeading = Rotation2d.fromDegrees(selectEntryHeadingDegrees(pose, speeds, tuning));
                    state.inZone = true;
                }

                double centerX = (zone.minX() + zone.maxX()) * 0.5;
                double xErrorMeters = centerX - pose.getX();
                double fieldVx = MathUtil.clamp(
                        xErrorMeters * tuning.centeringKpMpsPerMeter(),
                        -tuning.maxCenteringSpeedMps(),
                        tuning.maxCenteringSpeedMps());

                double headingErrorRad = MathUtil.angleModulus(
                        state.targetHeading.getRadians() - pose.getRotation().getRadians());
                double omegaRadPerSec = Math.abs(headingErrorRad) <= Math.toRadians(tuning.headingToleranceDeg())
                        ? 0.0
                        : MathUtil.clamp(
                                headingErrorRad * tuning.headingKpRadPerSecPerRad(),
                                -tuning.maxHeadingSpeedRadPerSec(),
                                tuning.maxHeadingSpeedRadPerSec());

                // Keep the robot on the lane centerline only; do not command lane-progress motion.
                ChassisSpeeds assistRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                        fieldVx,
                        0.0,
                        omegaRadPerSec,
                        pose.getRotation());

                speeds.setSpeeds(tuning.speedSource(), assistRobotRelative);
            });

            h.onExit(ctx -> stopAssist(ctx.drivetrain().getRobotSpeeds(), tuning, state));
        });
    }

    public record BoundingBoxCenterLineAssistTuning(
            String speedSource,
            int speedSourcePriority,
            double centeringKpMpsPerMeter,
            double maxCenteringSpeedMps,
            double headingKpRadPerSecPerRad,
            double maxHeadingSpeedRadPerSec,
            double headingToleranceDeg,
            double yTravelDirectionLatchThresholdMps) {

        public BoundingBoxCenterLineAssistTuning {
            Objects.requireNonNull(speedSource, "speedSource");
            if (speedSource.isBlank()) {
                throw new IllegalArgumentException("speedSource must not be blank");
            }
            if (speedSourcePriority <= 0) {
                throw new IllegalArgumentException("speedSourcePriority must be > 0");
            }
            if (!Double.isFinite(centeringKpMpsPerMeter) || centeringKpMpsPerMeter < 0.0) {
                throw new IllegalArgumentException("centeringKpMpsPerMeter must be finite and >= 0");
            }
            if (!Double.isFinite(maxCenteringSpeedMps) || maxCenteringSpeedMps < 0.0) {
                throw new IllegalArgumentException("maxCenteringSpeedMps must be finite and >= 0");
            }
            if (!Double.isFinite(headingKpRadPerSecPerRad) || headingKpRadPerSecPerRad < 0.0) {
                throw new IllegalArgumentException("headingKpRadPerSecPerRad must be finite and >= 0");
            }
            if (!Double.isFinite(maxHeadingSpeedRadPerSec) || maxHeadingSpeedRadPerSec < 0.0) {
                throw new IllegalArgumentException("maxHeadingSpeedRadPerSec must be finite and >= 0");
            }
            if (!Double.isFinite(headingToleranceDeg) || headingToleranceDeg < 0.0) {
                throw new IllegalArgumentException("headingToleranceDeg must be finite and >= 0");
            }
            if (!Double.isFinite(yTravelDirectionLatchThresholdMps) || yTravelDirectionLatchThresholdMps < 0.0) {
                throw new IllegalArgumentException("yTravelDirectionLatchThresholdMps must be finite and >= 0");
            }
        }

        public static BoundingBoxCenterLineAssistTuning defaults() {
            return new BoundingBoxCenterLineAssistTuning(
                    "laneAssist",
                    2,
                    1.8,
                    1.4,
                    4.5,
                    Math.toRadians(240.0),
                    2.0,
                    0.15);
        }
    }

    private static final class LaneAssistState {
        private boolean inZone;
        private Rotation2d targetHeading = Rotation2d.fromDegrees(90.0);
    }

    private static void stopAssist(
            RobotSpeeds speeds,
            BoundingBoxCenterLineAssistTuning tuning,
            LaneAssistState state) {
        if (speeds != null) {
            speeds.stopSpeeds(tuning.speedSource());
        }
        state.inZone = false;
    }

    private static double selectEntryHeadingDegrees(
            Pose2d pose,
            RobotSpeeds speeds,
            BoundingBoxCenterLineAssistTuning tuning) {
        ChassisSpeeds driverRobotRelative = speeds.getSpeeds("drive");
        ChassisSpeeds driverFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(driverRobotRelative, pose.getRotation());

        double vyField = driverFieldRelative.vyMetersPerSecond;
        if (Math.abs(vyField) >= tuning.yTravelDirectionLatchThresholdMps()) {
            return vyField >= 0.0 ? 90.0 : 270.0;
        }

        double headingDeg = pose.getRotation().getDegrees();
        double to90 = Math.abs(Math.toDegrees(MathUtil.angleModulus(Math.toRadians(90.0 - headingDeg))));
        double to270 = Math.abs(Math.toDegrees(MathUtil.angleModulus(Math.toRadians(270.0 - headingDeg))));
        return to90 <= to270 ? 90.0 : 270.0;
    }
}
