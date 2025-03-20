package ca.frc6390.athena.drivetrains.differential;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.control.TankDriveCommand;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.devices.EncoderGroup;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialDrivetrain extends SubsystemBase implements RobotDrivetrain<DifferentialDrivetrain> {

    private final RobotSpeeds robotSpeeds;
    private final IMU imu;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrive drive;
    private final MotorControllerGroup leftMotors, rightMotors;
    private final EncoderGroup leftEncoders, rightEncoders;

    public DifferentialDrivetrain(IMU imu, double maxVelocity, double trackwidth, MotorControllerGroup leftMotors, MotorControllerGroup rightMotors){
       this(imu, maxVelocity, trackwidth, leftMotors, rightMotors, leftMotors.getEncoderGroup(), rightMotors.getEncoderGroup());
    }

    public DifferentialDrivetrain(IMU imu, double maxVelocity, double trackwidth, MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, EncoderGroup leftEncoders, EncoderGroup rightEncoders){
        this.imu = imu;
        robotSpeeds = new RobotSpeeds(maxVelocity, maxVelocity);

        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;

        this.rightEncoders = rightEncoders;
        this.leftEncoders = leftEncoders;

        this.drive = new DifferentialDrive(leftMotors::setSpeed, rightMotors::setSpeed);
        this.kinematics = new DifferentialDriveKinematics(trackwidth);
    }

    @Override
    public IMU getIMU() {
        return imu;
    }

    @Override
    public void setNeutralMode(MotorNeutralMode mode) {
       leftMotors.setNeutralMode(mode);
       rightMotors.setNeutralMode(mode);
    }

    @Override
    public RobotSpeeds getRobotSpeeds() {
        return robotSpeeds;
    }

    @Override
    public void update() {
        imu.update();
        leftMotors.update();
        rightMotors.update();
        leftEncoders.update();
        rightEncoders.update();
        
        ChassisSpeeds speeds = getRobotSpeeds().calculate();
        drive.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier thetaInput){
        return new TankDriveCommand(this, xInput, thetaInput);
    }

    public void setDriveCommand(DoubleSupplier xInput, DoubleSupplier thetaInput){
        this.setDefaultCommand(getDriveCommand(xInput, thetaInput));
    }

    @Override
    public Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
        return getDriveCommand(xInput, thetaInput);
    }

    @Override
    public void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
        this.setDefaultCommand(getDriveCommand(xInput, yInput, thetaInput));
    }
    
    @Override
    public void periodic() {
        update();
    }

    @Override
    public DifferentialDrivetrain get() {
        return this;
    }

    public DifferentialDriveWheelPositions getPositions(){
        return new DifferentialDriveWheelPositions(leftEncoders.getPosition(), rightEncoders.getPosition());
    }

     @Override
    public RobotLocalization<DifferentialDriveWheelPositions> localization(RobotLocalizationConfig config) {
        
        DifferentialDrivePoseEstimator f = new DifferentialDrivePoseEstimator(kinematics, imu.getYaw(), getPositions().leftMeters, getPositions().rightMeters, new Pose2d(), config.getStd(), config.getVisionStd());
        DifferentialDrivePoseEstimator r = new DifferentialDrivePoseEstimator(kinematics, imu.getYaw(), getPositions().leftMeters, getPositions().rightMeters, new Pose2d(), config.getStd(), config.getVisionStd());

        return new RobotLocalization<DifferentialDriveWheelPositions>(f, r, config, getRobotSpeeds(), imu, this::getPositions);
    }
}
