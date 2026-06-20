package ca.frc6390.athena.examples;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.api.hardware.CameraId;
import ca.frc6390.athena.api.hardware.EncoderId;
import ca.frc6390.athena.api.hardware.ImuId;
import ca.frc6390.athena.api.hardware.MotorId;

/**
 * Central robot hardware map for the example project.
 *
 * <p>Examples use simulation hardware keys so the project validates without
 * external vendor libraries. Real robot projects should keep the same aliasing
 * pattern and swap the kind/id values for installed hardware.</p>
 */
public final class RobotHardware {
    /** Intake roller motor used by simulation examples. */
    public static final MotorId INTAKE_ROLLER = MotorId.of(AthenaMotor.SIM, 1);

    /** Shooter leader motor used by simulation examples. */
    public static final MotorId SHOOTER_LEADER = MotorId.of(AthenaMotor.SIM, 2);

    /** Shooter velocity encoder used by simulation examples. */
    public static final EncoderId SHOOTER_ENCODER = EncoderId.of(AthenaEncoder.SIM, 20);

    /** Left drivetrain leader motor. */
    public static final MotorId DRIVE_LEFT_LEADER = MotorId.of(AthenaMotor.SIM, 3);

    /** Right drivetrain leader motor. */
    public static final MotorId DRIVE_RIGHT_LEADER = MotorId.of(AthenaMotor.SIM, 4);

    /** Robot orientation sensor used by simulation examples. */
    public static final ImuId ROBOT_IMU = ImuId.of(AthenaImu.SIM, 0);

    /** Front vision camera used by simulation examples. */
    public static final CameraId FRONT_CAMERA = CameraId.of(AthenaCamera.SIM, "frontCam");

    /** Front-left swerve drive motor. */
    public static final MotorId SWERVE_FRONT_LEFT_DRIVE = MotorId.of(AthenaMotor.SIM, 31);

    /** Front-left swerve steer motor. */
    public static final MotorId SWERVE_FRONT_LEFT_STEER = MotorId.of(AthenaMotor.SIM, 32);

    /** Front-left swerve absolute encoder. */
    public static final EncoderId SWERVE_FRONT_LEFT_ENCODER = EncoderId.of(AthenaEncoder.SIM, 41);

    /** Front-right swerve drive motor. */
    public static final MotorId SWERVE_FRONT_RIGHT_DRIVE = MotorId.of(AthenaMotor.SIM, 33);

    /** Front-right swerve steer motor. */
    public static final MotorId SWERVE_FRONT_RIGHT_STEER = MotorId.of(AthenaMotor.SIM, 34);

    /** Front-right swerve absolute encoder. */
    public static final EncoderId SWERVE_FRONT_RIGHT_ENCODER = EncoderId.of(AthenaEncoder.SIM, 42);

    /** Back-left swerve drive motor. */
    public static final MotorId SWERVE_BACK_LEFT_DRIVE = MotorId.of(AthenaMotor.SIM, 35);

    /** Back-left swerve steer motor. */
    public static final MotorId SWERVE_BACK_LEFT_STEER = MotorId.of(AthenaMotor.SIM, 36);

    /** Back-left swerve absolute encoder. */
    public static final EncoderId SWERVE_BACK_LEFT_ENCODER = EncoderId.of(AthenaEncoder.SIM, 43);

    /** Back-right swerve drive motor. */
    public static final MotorId SWERVE_BACK_RIGHT_DRIVE = MotorId.of(AthenaMotor.SIM, 37);

    /** Back-right swerve steer motor. */
    public static final MotorId SWERVE_BACK_RIGHT_STEER = MotorId.of(AthenaMotor.SIM, 38);

    /** Back-right swerve absolute encoder. */
    public static final EncoderId SWERVE_BACK_RIGHT_ENCODER = EncoderId.of(AthenaEncoder.SIM, 44);

    private RobotHardware() {
    }
}
