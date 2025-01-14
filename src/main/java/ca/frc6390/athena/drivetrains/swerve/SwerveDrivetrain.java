package ca.frc6390.athena.drivetrains.swerve;

import java.util.Map;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.SwerveDriveCommand;
import ca.frc6390.athena.core.RobotIMU;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase implements RobotDrivetrain {

  public SwerveModule[] swerveModules;
  public SwerveDriveKinematics kinematics;
  public PIDController driftpid;
  public ChassisSpeeds driveSpeeds, feedbackSpeeds;
  public boolean enableDriftCorrection;
  public double desiredHeading, maxVelocity;
  public RobotIMU imu;

  public SwerveDrivetrain(SwerveModuleConfig[] configs, RobotIMU imu) {
    this(configs, imu, false, new PIDController(0, 0, 0));
  }

  public SwerveDrivetrain(SwerveModuleConfig[] configs, RobotIMU imu, boolean driftCorrection,
      PIDController driftCorrectionPID) {
    driftCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);
    driveSpeeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();
    swerveModules = new SwerveModule[configs.length];
    for (int i = 0; i < configs.length; i++) {
      swerveModules[i] = new SwerveModule(configs[i]);

      maxVelocity = configs[i].driveMotor().maxSpeedMetersPerSecond();
    }
    Translation2d[] moduleLocations = new Translation2d[swerveModules.length];
    for (int i = 0; i < configs.length; i++) {
      moduleLocations[i] = swerveModules[i].getModuleLocation();
    }
    kinematics = new SwerveDriveKinematics(moduleLocations);
    enableDriftCorrection = driftCorrection;
    driftpid = driftCorrectionPID;
    this.imu = imu;
  }

  
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  } 

  public SwerveModulePosition[] getSwerveModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  public ChassisSpeeds getDriveSpeeds() {
    return driveSpeeds;
  }

  private void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }


  private double driftCorrection(ChassisSpeeds speeds) {
    if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0) {
      desiredHeading = getIMU().getYaw().getRadians();
      return 0;
    } else {
      return driftpid.calculate(getIMU().getYaw().getRadians(), desiredHeading);
    }
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public void setDriftCorrectionMode(boolean enabled) {
    enableDriftCorrection = enabled;
  }

  public boolean getDriftCorrectionMode() {
    return enableDriftCorrection;
  }

  @Override
  public RobotIMU getIMU() {
      return imu;
  }

  @Override
  public void setNeutralMode(DriveTrainNeutralMode mode) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setNeutralMode(mode);
    }
  }

  @Override
  public DriveTrainNeutralMode getNeutralMode() {
    return swerveModules[0].getNeutralMode();
  }

  @Override
  public void drive(ChassisSpeeds driveSpeeds) {
    this.driveSpeeds = driveSpeeds;
  }

  @Override
  public void addFeedbackSpeed(ChassisSpeeds feedbackSpeeds) {
    this.feedbackSpeeds = feedbackSpeeds;
  }

  @Override
  public void update() {
    imu.update();
    
    ChassisSpeeds speed = driveSpeeds.plus(feedbackSpeeds);

    if (enableDriftCorrection) {
      speed.omegaRadiansPerSecond += driftCorrection(speed);
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, getMaxVelocity());
    setModuleStates(states);    
  }

  @Override
  public Command createDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    return new SwerveDriveCommand(this, xInput, yInput, thetaInput);
  }

  public ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
  }

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
    
    ShuffleboardLayout swervelayout = tab.getLayout("Swerve Modules", BuiltInLayouts.kGrid).withSize(4, 8).withProperties(Map.of("Number of columns", 2, "Number of rows", 2));

    {
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].shuffleboard(swervelayout.getLayout("Module " + i, BuiltInLayouts.kGrid));
      }
    }

    ShuffleboardLayout imuLayout = tab.getLayout("IMU", BuiltInLayouts.kGrid).withSize(4, 8).withProperties(Map.of("Number of columns", 2, "Number of rows", 3));

    getIMU().shuffleboard(imuLayout);

    ShuffleboardLayout speedsLayout = tab.getLayout("Robot Speeds", BuiltInLayouts.kGrid).withSize(2, 3).withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
    { 
      Map<String, Object> props = Map.of("Min", -getMaxVelocity(), "Max", getMaxVelocity(),"Label position", "TOP");
      ShuffleboardLayout chassisLayout = speedsLayout.getLayout("Chassis", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 3,"Label position", "TOP"));
      chassisLayout.addDouble("X", () -> driveSpeeds.vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      chassisLayout.addDouble("Y", () -> driveSpeeds.vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      chassisLayout.addDouble("Z", () -> driveSpeeds.omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      ShuffleboardLayout feedbackLayout = speedsLayout.getLayout("Feedback", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 3,"Label position", "TOP"));
      feedbackLayout.addDouble("X", () -> driveSpeeds.vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      feedbackLayout.addDouble("Y", () -> feedbackSpeeds.vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      feedbackLayout.addDouble("Z", () -> feedbackSpeeds.omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
    }

    ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands",BuiltInLayouts.kGrid).withSize(1, 3).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    {
      commandsLayout.add("Reset Heading", new InstantCommand(() -> getIMU().setYaw(0))).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Brake Mode", new InstantCommand(() -> setNeutralMode(DriveTrainNeutralMode.Brake))).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Coast Mode", new InstantCommand(() -> setNeutralMode(DriveTrainNeutralMode.Coast))).withWidget(BuiltInWidgets.kCommand);
    }

    ShuffleboardLayout driftCorrectionLayout = tab.getLayout("Drift Correction", BuiltInLayouts.kList).withSize(2, 2);
    {
      driftCorrectionLayout.add("Drift Correction", (builder) -> builder.addBooleanProperty("Drift Correction", this::getDriftCorrectionMode, this::setDriftCorrectionMode)).withWidget(BuiltInWidgets.kBooleanBox);
      driftCorrectionLayout.addDouble("Desired Heading", () -> desiredHeading).withWidget(BuiltInWidgets.kGyro);// might not display properly bc get heading is +-180
      driftCorrectionLayout.add(driftpid).withWidget(BuiltInWidgets.kPIDController);
    }
    return tab;
  }

  
}