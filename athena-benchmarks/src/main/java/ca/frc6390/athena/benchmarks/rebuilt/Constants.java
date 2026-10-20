package ca.frc6390.athena.benchmarks.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.runtime.geometry.Geometry2d;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final HardwareBus RIO = HardwareBus.rio();
    public static final HardwareBus CAN = HardwareBus.can("can");
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.21;
    public static final Distance FIELD_LENGTH = Meters.of(FIELD_LENGTH_METERS);
    public static final Distance FIELD_WIDTH = Meters.of(FIELD_WIDTH_METERS);
    
    private Constants() {
    }

    public static final class Field {
        public static final double BLUE_HUB_X_METERS = 4.63;
        public static final double RED_HUB_X_METERS = 11.87;
        public static final double HUB_Y_METERS = 4.0;

        public static final Rectangle2d BLUE_LEFT_TRENCH =
                Rectangle2d.of(Meters.of(3.75), Meters.of(6.8), Meters.of(5.45), Meters.of(8.024));
        public static final Rectangle2d BLUE_RIGHT_TRENCH =
                Rectangle2d.of(Meters.of(3.75), Meters.of(0.0), Meters.of(5.45), Meters.of(1.2264050245285034));
        public static final Rectangle2d RED_LEFT_TRENCH =
                Rectangle2d.of(Meters.of(11.05), Meters.of(6.8), Meters.of(12.74), Meters.of(8.0));
        public static final Rectangle2d RED_RIGHT_TRENCH =
                Rectangle2d.of(Meters.of(11.05), Meters.of(0.0), Meters.of(12.74), Meters.of(1.2));
        public static final Geometry2d TRENCHES = point -> Math.min(
                Math.min(BLUE_LEFT_TRENCH.signedDistancePrimitive(point),
                        BLUE_RIGHT_TRENCH.signedDistancePrimitive(point)),
                Math.min(RED_LEFT_TRENCH.signedDistancePrimitive(point),
                        RED_RIGHT_TRENCH.signedDistancePrimitive(point)));

        private Field() {
        }

        public static double allianceHubX() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                    ? RED_HUB_X_METERS
                    : BLUE_HUB_X_METERS;
        }

        public static double rearFacingHubHeadingRadians(Pose2d robotPose) {
            double angleToHub = Math.atan2(
                    HUB_Y_METERS - robotPose.getY(),
                    allianceHubX() - robotPose.getX());
            return angleToHub - Math.PI;
        }

        public static double distanceToAllianceHubMeters(Pose2d robotPose) {
            return Math.hypot(
                    allianceHubX() - robotPose.getX(),
                    HUB_Y_METERS - robotPose.getY());
        }
    }

    public static final class ShotCalibration {
        public static final Distance NEAR_DISTANCE = Inches.of(64.0);
        // public static final Distance NEAR_MIDDLE_DISTANCE = Inches.of(86.0);
        public static final Distance MIDDLE_DISTANCE = Meters.of(2.995);
        // public static final Distance MIDDLE_FAR_DISTANCE = Inches.of(130.0);
        public static final Distance FAR_DISTANCE = Inches.of(151.0);

        public static final double NEAR_SHOOTER_RPS = 76.0;
        // public static final double NEAR_MIDDLE_SHOOTER_RPS = 80.0;
        public static final double MIDDLE_SHOOTER_RPS = 81.2;
        // public static final double MIDDLE_FAR_SHOOTER_RPS = 81.2;
        public static final double FAR_SHOOTER_RPS = 81.2;

        public static final double NEAR_HOOD_DEGREES = 13.791;
        // public static final double NEAR_MIDDLE_HOOD_DEGREES = 17.4;
        public static final double MIDDLE_HOOD_DEGREES = 21.967; 
        // public static final double MIDDLE__FAR_HOOD_DEGREES = 29.356;
        public static final double FAR_HOOD_DEGREES = 36.473;

        private ShotCalibration() {
        }
    }
}
