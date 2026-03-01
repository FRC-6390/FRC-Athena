package ca.frc6390.athena.drivetrains.swerve.modules;

import edu.wpi.first.math.util.Units;

/**
 * REV MAXSwerve and EasySwerve drivetrain module presets.
 */
public final class SwerveVendorREV {

    /**
     * 3in MAXSwerve module speed options from
     * REV-21-3005 base and upgrade kit speed ratio charts.
     */
    public static enum MAXSwerve3in implements SwerveVendorModule {
        LOW(1d / 5.50d, 9424d / 203d, 1, 3),
        MEDIUM(1d / 5.08d, 9424d / 203d, 1, 3),
        HIGH(1d / 4.71d, 9424d / 203d, 1, 3),
        EXTRA_HIGH_1(1d / 4.50d, 9424d / 203d, 1, 3),
        EXTRA_HIGH_2(1d / 4.29d, 9424d / 203d, 1, 3),
        EXTRA_HIGH_3(1d / 4.00d, 9424d / 203d, 1, 3),
        EXTRA_HIGH_4(1d / 3.75d, 9424d / 203d, 1, 3),
        EXTRA_HIGH_5(1d / 3.56d, 9424d / 203d, 1, 3);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        MAXSwerve3in(double driveGearRatio, double rotationGearRatio, double encoderGearRatio, double wheelDiameterInches) {
            this.driveGearRatio = driveGearRatio;
            this.rotationGearRatio = rotationGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.wheelDiameter = Units.inchesToMeters(wheelDiameterInches);
        }

        @Override
        public double getDriveGearRatio() {
            return driveGearRatio;
        }

        @Override
        public double getSteerGearRatio() {
            return rotationGearRatio;
        }

        @Override
        public double getEncoderGearRatio() {
            return encoderGearRatio;
        }

        @Override
        public double getWheelDiameter() {
            return wheelDiameter;
        }
    }

    /**
     * 4in EasySwerve module presets from REV-21-3006 specifications.
     */
    public static enum EasySwerve4in implements SwerveVendorModule {
        STANDARD(1d / 6.3d, 20d, 1, 4);

        private final double driveGearRatio, rotationGearRatio, encoderGearRatio, wheelDiameter;

        EasySwerve4in(double driveGearRatio, double rotationGearRatio, double encoderGearRatio, double wheelDiameterInches) {
            this.driveGearRatio = driveGearRatio;
            this.rotationGearRatio = rotationGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.wheelDiameter = Units.inchesToMeters(wheelDiameterInches);
        }

        @Override
        public double getDriveGearRatio() {
            return driveGearRatio;
        }

        @Override
        public double getSteerGearRatio() {
            return rotationGearRatio;
        }

        @Override
        public double getEncoderGearRatio() {
            return encoderGearRatio;
        }

        @Override
        public double getWheelDiameter() {
            return wheelDiameter;
        }
    }
}
