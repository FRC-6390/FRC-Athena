package ca.frc6390.athena.wpilib.drivetrain;

import ca.frc6390.athena.drivetrain.spec.DifferentialDrivetrainSpec;
import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.util.Objects;
import java.util.function.DoubleConsumer;

/**
 * Applies Athena robot speed output to a WPILib differential drive.
 */
public final class WpilibDifferentialDriveAdapter {
    private final RobotSpeeds speeds;
    private final DriveOutput drive;
    private final double trackWidthMeters;
    private final double maxWheelMetersPerSecond;

    private WpilibDifferentialDriveAdapter(
            RobotSpeeds speeds,
            DriveOutput drive,
            double trackWidthMeters,
            double maxWheelMetersPerSecond) {
        this.speeds = Objects.requireNonNull(speeds, "speeds");
        this.drive = Objects.requireNonNull(drive, "drive");
        this.trackWidthMeters = finitePositive(trackWidthMeters, 1.0);
        this.maxWheelMetersPerSecond = finitePositive(maxWheelMetersPerSecond, 1.0);
    }

    /**
     * Creates an adapter backed by WPILib motor controllers.
     *
     * @param speeds Athena speed blender
     * @param spec differential drivetrain spec
     * @param left left motor controller
     * @param right right motor controller
     * @param maxWheelMetersPerSecond wheel speed that maps to full output
     * @return differential drive adapter
     */
    public static WpilibDifferentialDriveAdapter create(
            RobotSpeeds speeds,
            DifferentialDrivetrainSpec spec,
            MotorController left,
            MotorController right,
            double maxWheelMetersPerSecond) {
        return create(
                speeds,
                trackWidthMeters(spec),
                new WpilibDriveOutput(new DifferentialDrive(
                        Objects.requireNonNull(left, "left"),
                        Objects.requireNonNull(right, "right"))),
                maxWheelMetersPerSecond);
    }

    /**
     * Creates an adapter backed by output consumers and a drivetrain spec.
     *
     * @param speeds Athena speed blender
     * @param spec differential drivetrain spec
     * @param leftOutput left output consumer
     * @param rightOutput right output consumer
     * @param maxWheelMetersPerSecond wheel speed that maps to full output
     * @return differential drive adapter
     */
    public static WpilibDifferentialDriveAdapter create(
            RobotSpeeds speeds,
            DifferentialDrivetrainSpec spec,
            DoubleConsumer leftOutput,
            DoubleConsumer rightOutput,
            double maxWheelMetersPerSecond) {
        return create(speeds, trackWidthMeters(spec), leftOutput, rightOutput, maxWheelMetersPerSecond);
    }

    /**
     * Creates an adapter backed by output consumers.
     *
     * @param speeds Athena speed blender
     * @param trackWidthMeters drivetrain track width
     * @param leftOutput left output consumer
     * @param rightOutput right output consumer
     * @param maxWheelMetersPerSecond wheel speed that maps to full output
     * @return differential drive adapter
     */
    public static WpilibDifferentialDriveAdapter create(
            RobotSpeeds speeds,
            double trackWidthMeters,
            DoubleConsumer leftOutput,
            DoubleConsumer rightOutput,
            double maxWheelMetersPerSecond) {
        return create(
                speeds,
                trackWidthMeters,
                new DirectDriveOutput(
                        Objects.requireNonNull(leftOutput, "leftOutput"),
                        Objects.requireNonNull(rightOutput, "rightOutput")),
                maxWheelMetersPerSecond);
    }

    private static WpilibDifferentialDriveAdapter create(
            RobotSpeeds speeds,
            double trackWidthMeters,
            DriveOutput drive,
            double maxWheelMetersPerSecond) {
        return new WpilibDifferentialDriveAdapter(
                speeds,
                drive,
                trackWidthMeters,
                maxWheelMetersPerSecond);
    }

    /**
     * Applies the current robot-relative output.
     */
    public void periodic() {
        apply(speeds.calculate());
    }

    /**
     * Applies the current output after converting field-relative sources.
     *
     * @param headingRadians robot heading in radians
     */
    public void periodic(double headingRadians) {
        apply(speeds.calculate(headingRadians));
    }

    /**
     * Stops the WPILib drive.
     */
    public void stop() {
        drive.stopMotor();
    }

    private void apply(RobotVelocity velocity) {
        double halfTrack = trackWidthMeters / 2.0;
        double leftMetersPerSecond = velocity.xMetersPerSecond()
                - velocity.angularRadiansPerSecond() * halfTrack;
        double rightMetersPerSecond = velocity.xMetersPerSecond()
                + velocity.angularRadiansPerSecond() * halfTrack;
        drive.tankDrive(
                clamp(leftMetersPerSecond / maxWheelMetersPerSecond),
                clamp(rightMetersPerSecond / maxWheelMetersPerSecond),
                false);
    }

    private static double trackWidthMeters(DifferentialDrivetrainSpec spec) {
        DifferentialDrivetrainSpec safeSpec = Objects.requireNonNull(spec, "spec");
        return safeSpec.trackWidth() == null ? 1.0 : safeSpec.trackWidth().meters();
    }

    private static double finitePositive(double value, double fallback) {
        return Double.isFinite(value) && value > 0.0 ? value : fallback;
    }

    private static double clamp(double value) {
        double finite = Double.isFinite(value) ? value : 0.0;
        return Math.max(-1.0, Math.min(1.0, finite));
    }

    interface DriveOutput {
        void tankDrive(double leftOutput, double rightOutput, boolean squareInputs);

        void stopMotor();
    }

    private record WpilibDriveOutput(DifferentialDrive drive) implements DriveOutput {
        private WpilibDriveOutput {
            Objects.requireNonNull(drive, "drive");
        }

        @Override
        public void tankDrive(double leftOutput, double rightOutput, boolean squareInputs) {
            drive.tankDrive(leftOutput, rightOutput, squareInputs);
        }

        @Override
        public void stopMotor() {
            drive.stopMotor();
        }
    }

    private record DirectDriveOutput(DoubleConsumer leftOutput, DoubleConsumer rightOutput) implements DriveOutput {
        private DirectDriveOutput {
            Objects.requireNonNull(leftOutput, "leftOutput");
            Objects.requireNonNull(rightOutput, "rightOutput");
        }

        @Override
        public void tankDrive(double leftOutputValue, double rightOutputValue, boolean squareInputs) {
            leftOutput.accept(leftOutputValue);
            rightOutput.accept(rightOutputValue);
        }

        @Override
        public void stopMotor() {
            leftOutput.accept(0.0);
            rightOutput.accept(0.0);
        }
    }
}
