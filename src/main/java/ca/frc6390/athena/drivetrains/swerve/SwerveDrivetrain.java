package ca.frc6390.athena.drivetrains.swerve;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.commands.control.SwerveDriveCommand;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveDrivetrainSimulation;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveSimulationConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase implements RobotDrivetrain<SwerveDrivetrain> {

  public SwerveModule[] swerveModules;
  public SwerveDriveKinematics kinematics;
  public PIDController driftpid;
  public boolean enableDriftCorrection, fieldRelative;
  public double desiredHeading, driftActivationSpeed;
  public IMU imu;
  public RobotSpeeds robotSpeeds;
  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
  private SwerveDrivetrainSimulation simulation;
  private Field2d simulationField;
  private double lastSimulationTimestamp = -1;
  private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();
  
  public SwerveDrivetrain(IMU imu, SwerveModuleConfig... modules) {

    this.imu = imu;

    enableDriftCorrection = false; 
    driftpid = new PIDController(0, 0,0);
    driftpid.enableContinuousInput(-Math.PI, Math.PI);

    swerveModules = new SwerveModule[modules.length];
    Translation2d[] moduleLocations = new Translation2d[swerveModules.length];
    double maxVelocity = 0;
    double maxModuleRadius = 0;
    for (int i = 0; i < modules.length; i++) {
      swerveModules[i] = new SwerveModule(modules[i]);
      maxVelocity = modules[i].maxSpeedMetersPerSecond();
      moduleLocations[i] = swerveModules[i].getModuleLocation();
      maxModuleRadius = Math.max(maxModuleRadius, moduleLocations[i].getNorm());
    }

    kinematics = new SwerveDriveKinematics(moduleLocations);
    double maxAngularVelocity = maxModuleRadius > 1e-6 ? maxVelocity / maxModuleRadius : maxVelocity;
    robotSpeeds = new RobotSpeeds(maxVelocity, maxAngularVelocity);
    getIMU().addVirtualAxis("drift", () -> getIMU().getYaw());
    getIMU().setVirtualAxis("drift", getIMU().getVirtualAxis("driver"));
    desiredHeading = getIMU().getVirtualAxis("drift").getRadians();
    
  }

  public SwerveDrivetrain withDriftCorretion(PIDController controller){
    driftpid = controller;
    driftpid.enableContinuousInput(-Math.PI, Math.PI);
    enableDriftCorrection = true;
    return this;
  }
  
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  } 

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  private void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }


  private double driftCorrection(ChassisSpeeds speeds) {
    if (Math.abs(speeds.omegaRadiansPerSecond) > driftActivationSpeed && Math.abs(imu.getVelocityZ().getRadians()) > 0.5) {
      desiredHeading = getIMU().getVirtualAxis("drift").getRadians();
      return 0;
    } else {
      return driftpid.calculate(getIMU().getVirtualAxis("drift").getRadians(), desiredHeading);
    }
  }

  public SwerveDrivetrain setDriftCorrectionPID(PIDController controller) {
    driftpid = controller;
    return this;
  }
  
  public void setDriftCorrectionMode(boolean enabled) {
    enableDriftCorrection = enabled;
  }

  public boolean getDriftCorrectionMode() {
    return enableDriftCorrection;
  }

  public void setFieldRelative(boolean fieldRelative) {
      this.fieldRelative = fieldRelative;
  }

  @Override
  public IMU getIMU() {
      return imu;
  }

  @Override
  public void setNeutralMode(MotorNeutralMode mode) {
    Arrays.stream(swerveModules).forEach(m -> m.setNeutralMode(mode));
    // for (int i = 0; i < swerveModules.length; i++) {
    //   swerveModules[i].setNeutralMode(mode);
    // }
  }

  @Override
  public RobotSpeeds getRobotSpeeds() {
    return robotSpeeds;
  }

  @Override
  public void update() {
    imu.update();
    
    ChassisSpeeds speed = robotSpeeds.calculate();

    if (enableDriftCorrection) {
      speed.omegaRadiansPerSecond += driftCorrection(speed);
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

    publisher.set(states);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, getRobotSpeeds().getMaxVelocity());

    setModuleStates(states);   

    lastCommandedSpeeds = new ChassisSpeeds(
        speed.vxMetersPerSecond,
        speed.vyMetersPerSecond,
        speed.omegaRadiansPerSecond
    );
  }

  public SwerveDrivetrain configureSimulation(SwerveSimulationConfig config) {
    if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return this;
    }
    simulation = new SwerveDrivetrainSimulation(this, config);
    if (simulationField == null) {
      simulationField = new Field2d();
    }
    simulationField.setRobotPose(new Pose2d());
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

  @Override
  public Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    return new SwerveDriveCommand(this, xInput, yInput, thetaInput, fieldRelative);
  }

  @Override
  public void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    this.setDefaultCommand(getDriveCommand(xInput, yInput, thetaInput));
  }

  @Override
  public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {

    ShuffleboardLayout swervelayout = tab.getLayout("Swerve Modules", BuiltInLayouts.kList);

    {
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].shuffleboard(swervelayout.getLayout("Module " + i, BuiltInLayouts.kList)).withPosition(1, i+1);
      }
    }

    getIMU().shuffleboard(tab.getLayout("IMU", BuiltInLayouts.kList));

    if(level.equals(SendableLevel.DEBUG)){
      ShuffleboardLayout speedsLayout = tab.getLayout("Robot Speeds", BuiltInLayouts.kList);
      { 
        ShuffleboardLayout chassisLayout = speedsLayout.getLayout("Chassis", BuiltInLayouts.kList);
        chassisLayout.addDouble("X", () -> robotSpeeds.getSpeeds("drive").vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar);
        chassisLayout.addDouble("Y", () -> robotSpeeds.getSpeeds("drive").vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar);
        chassisLayout.addDouble("Z", () -> robotSpeeds.getSpeeds("drive").omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar);
      }
      ShuffleboardLayout driftCorrectionLayout = tab.getLayout("Drift Correction", BuiltInLayouts.kList);
      {
        driftCorrectionLayout.add("Drift Correction", (builder) -> builder.addBooleanProperty("Drift Correction", this::getDriftCorrectionMode, this::setDriftCorrectionMode)).withWidget(BuiltInWidgets.kBooleanBox);
        driftCorrectionLayout.addDouble("Desired Heading", () -> desiredHeading).withWidget(BuiltInWidgets.kGyro);// might not display properly bc get heading is +-180
        driftCorrectionLayout.add(driftpid).withWidget(BuiltInWidgets.kPIDController);
      }
    }
   
    ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands",BuiltInLayouts.kList);
    {
      commandsLayout.add("Reset Heading", new InstantCommand(() -> getIMU().setYaw(0))).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Brake Mode", new InstantCommand(() -> setNeutralMode(MotorNeutralMode.Brake))).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Coast Mode", new InstantCommand(() -> setNeutralMode(MotorNeutralMode.Coast))).withWidget(BuiltInWidgets.kCommand);
    }

    if (simulationField != null) {
      tab.add("Sim Pose", simulationField).withWidget(BuiltInWidgets.kField);
    }

    return tab;
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
        simulation.update(dt > 0 ? dt : 0.02, lastCommandedSpeeds);
        if (simulationField != null) {
          simulationField.setRobotPose(simulation.getPose());
        }
      }
  }

  @Override
  public SwerveDrivetrain get() {
    return this;
  }

  @Override
  public RobotLocalization<SwerveModulePosition[]> localization(RobotLocalizationConfig config) {
    RobotLocalizationConfig effectiveConfig = config != null ? config : RobotLocalizationConfig.defualt();
    if (effectiveConfig.poseSpace() == RobotLocalizationConfig.PoseSpace.THREE_D) {
      Rotation3d gyroRotation = getImuRotation3d();
      SwerveDrivePoseEstimator3d fieldEstimator =
              new SwerveDrivePoseEstimator3d(
                      kinematics,
                      gyroRotation,
                      getPositions(),
                      new Pose3d(),
                      effectiveConfig.getStd3d(),
                      effectiveConfig.getVisionStd3d());
      SwerveDrivePoseEstimator3d relativeEstimator =
              new SwerveDrivePoseEstimator3d(
                      kinematics,
                      gyroRotation,
                      getPositions(),
                      new Pose3d(),
                      effectiveConfig.getStd3d(),
                      effectiveConfig.getVisionStd3d());
      return new RobotLocalization<>(fieldEstimator, relativeEstimator, effectiveConfig, robotSpeeds, imu, this::getPositions);
    }

    SwerveDrivePoseEstimator fieldEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics,
                    imu.getYaw(),
                    getPositions(),
                    new Pose2d(),
                    effectiveConfig.getStd(),
                    effectiveConfig.getVisionStd());
    SwerveDrivePoseEstimator relativeEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics,
                    imu.getYaw(),
                    getPositions(),
                    new Pose2d(),
                    effectiveConfig.getStd(),
                    effectiveConfig.getVisionStd());

    return new RobotLocalization<>(fieldEstimator, relativeEstimator, effectiveConfig, robotSpeeds, imu, this::getPositions);
  }

  private Rotation3d getImuRotation3d() {
    return new Rotation3d(
            imu.getRoll().getRadians(),
            imu.getPitch().getRadians(),
            imu.getYaw().getRadians());
  }
}
