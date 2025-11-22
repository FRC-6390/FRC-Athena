package ca.frc6390.athena.drivetrains.differential;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.control.TankDriveCommand;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.devices.EncoderGroup;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.drivetrains.differential.sim.DifferentialDrivetrainSimulation;
import ca.frc6390.athena.drivetrains.differential.sim.DifferentialSimulationConfig;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DifferentialDrivetrain extends SubsystemBase implements RobotDrivetrain<DifferentialDrivetrain> {

    private final RobotSpeeds robotSpeeds;
    private final IMU imu;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrive drive;
    private final MotorControllerGroup leftMotors, rightMotors;
    private final EncoderGroup leftEncoders, rightEncoders;
    private DifferentialDrivetrainSimulation simulation;
    private Field2d simulationField;
    private double lastSimulationTimestamp = -1;
    private double lastLeftCommand = 0;
    private double lastRightCommand = 0;

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

        this.drive = new DifferentialDrive(this::setLeftOutput, this::setRightOutput);
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
        if (imu != null) {
            imu.update();
        }
        leftMotors.update();
        rightMotors.update();
        if (simulation == null) {
            if (leftEncoders != null) {
                leftEncoders.update();
            }
            if (rightEncoders != null) {
                rightEncoders.update();
            }
        }
        
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
    public void simulationPeriodic() {
        if (simulation != null) {
            double now = Timer.getFPGATimestamp();
            if (lastSimulationTimestamp < 0) {
                lastSimulationTimestamp = now;
            }
            double dt = now - lastSimulationTimestamp;
            lastSimulationTimestamp = now;
            simulation.update(dt > 0 ? dt : 0.02, lastLeftCommand, lastRightCommand);
            if (simulationField != null) {
                simulationField.setRobotPose(simulation.getPose());
            }
        }
    }

    @Override
    public DifferentialDrivetrain get() {
        return this;
    }

    public DifferentialDriveWheelPositions getPositions(){
     double leftPosition = leftEncoders != null ? leftEncoders.getPosition() : 0;
     double rightPosition = rightEncoders != null ? rightEncoders.getPosition() : 0;
     return new DifferentialDriveWheelPositions(leftPosition, rightPosition);
    }

     @Override
    public RobotLocalization<DifferentialDriveWheelPositions> localization(RobotLocalizationConfig config) {
        RobotLocalizationConfig effectiveConfig = config != null ? config : RobotLocalizationConfig.defualt();
        DifferentialDriveWheelPositions positions = getPositions();
        if (effectiveConfig.poseSpace() == RobotLocalizationConfig.PoseSpace.THREE_D) {
            Rotation3d gyroRotation = getImuRotation3d();
            DifferentialDrivePoseEstimator3d fieldEstimator =
                    new DifferentialDrivePoseEstimator3d(
                            kinematics,
                            gyroRotation,
                            positions.leftMeters,
                            positions.rightMeters,
                            new Pose3d(),
                            effectiveConfig.getStd3d(),
                            effectiveConfig.getVisionStd3d());
            DifferentialDrivePoseEstimator3d relativeEstimator =
                    new DifferentialDrivePoseEstimator3d(
                            kinematics,
                            gyroRotation,
                            positions.leftMeters,
                            positions.rightMeters,
                            new Pose3d(),
                            effectiveConfig.getStd3d(),
                            effectiveConfig.getVisionStd3d());
            return new RobotLocalization<>(fieldEstimator, relativeEstimator, effectiveConfig, getRobotSpeeds(), imu, this::getPositions);
        }

        DifferentialDrivePoseEstimator fieldEstimator =
                new DifferentialDrivePoseEstimator(
                        kinematics,
                        imu.getYaw(),
                        positions.leftMeters,
                        positions.rightMeters,
                        new Pose2d(),
                        effectiveConfig.getStd(),
                        effectiveConfig.getVisionStd());
        DifferentialDrivePoseEstimator relativeEstimator =
                new DifferentialDrivePoseEstimator(
                        kinematics,
                        imu.getYaw(),
                        positions.leftMeters,
                        positions.rightMeters,
                        new Pose2d(),
                        effectiveConfig.getStd(),
                        effectiveConfig.getVisionStd());

        return new RobotLocalization<>(fieldEstimator, relativeEstimator, effectiveConfig, getRobotSpeeds(), imu, this::getPositions);
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        RobotDrivetrain.super.shuffleboard(tab, level);
        if (simulationField != null) {
            tab.add("Sim Pose", simulationField).withWidget(BuiltInWidgets.kField);
        }
        return tab;
    }

    public DifferentialDrivetrain configureSimulation(DifferentialSimulationConfig config) {
        if (!RobotBase.isSimulation()) {
            return this;
        }
        simulation = new DifferentialDrivetrainSimulation(this, config, leftEncoders, rightEncoders);
        if (simulationField == null) {
            simulationField = new Field2d();
        }
        simulationField.setRobotPose(simulation.getPose());
        lastSimulationTimestamp = -1;
        return this;
    }

    public boolean hasSimulation() {
        return simulation != null;
    }

    public Pose2d getSimulatedPose() {
        return simulation != null ? simulation.getPose() : new Pose2d();
    }

    public void resetSimulationPose(Pose2d pose) {
        if (simulation != null) {
            simulation.resetPose(pose);
        }
    }

    private Rotation3d getImuRotation3d() {
        return new Rotation3d(
                imu.getRoll().getRadians(),
                imu.getPitch().getRadians(),
                imu.getYaw().getRadians());
    }

    private void setLeftOutput(double output) {
        lastLeftCommand = output;
        leftMotors.setSpeed(output);
    }

    private void setRightOutput(double output) {
        lastRightCommand = output;
        rightMotors.setSpeed(output);
    }

}
