package ca.frc6390.athena.drivetrains.swerve;

import java.util.Map;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.SwerveDriveCommand;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDriveTrainIDs.DriveIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDriveTrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDriveTrainIDs.EncoderIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDriveTrainIDs.SteerIDs;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
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
  public boolean enableDriftCorrection;
  public double desiredHeading, maxVelocity;
  public IMU imu;
  public RobotSpeeds robotSpeeds;

  public record SwerveDrivetrainConfig(SwerveModuleConfig[] modules) {

    public static SwerveDrivetrainConfig standard(SwerveModuleConfig config){
      return SwerveDrivetrainConfig.custom(config,config,config,config);
   }

    public static SwerveDrivetrainConfig custom(SwerveModuleConfig... modules){
       return new SwerveDrivetrainConfig(modules);
    }

    public SwerveDrivetrainConfig setModulueLocations(Translation2d[] locations){
      if (locations.length != modules.length) {
        throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
      }

      for (int i = 0; i < locations.length; i++) {
        modules[i].setLocation(locations[i]);
      } 

      return this;
    }

    public SwerveDrivetrainConfig setModulueLocations(double trackWidth, double wheelbase){
      return setModulueLocations(SwerveModuleConfig.generateModuleLocations(trackWidth, wheelbase));
    }

    public SwerveDrivetrainConfig setDriveIDs(DriveIDs ids){
      return setDriveIDs(ids.getIDs());
    }

    public SwerveDrivetrainConfig setDriveIDs(int[] ids){

      if (ids.length != modules.length) {
        throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
      }

      for (int i = 0; i < ids.length; i++) {
        modules[i].setDriveID(ids[i]);
      } 

      return this;
    }

    public SwerveDrivetrainConfig setSteerIDs(SteerIDs ids){
      return setSteerIDs(ids.getIDs());
    }

    public SwerveDrivetrainConfig setSteerIDs(int[] ids){

      if (ids.length != modules.length) {
        throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
      }

      for (int i = 0; i < ids.length; i++) {
        modules[i].setSteerID(ids[i]);
      } 

      return this;
    }

    public SwerveDrivetrainConfig setEncoderIDs(EncoderIDs ids){
      return setEncoderIDs(ids.getIDs());
    }

    public SwerveDrivetrainConfig setEncoderIDs(int[] ids){

      if (ids.length != modules.length) {
        throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
      }

      for (int i = 0; i < ids.length; i++) {
        modules[i].setEncoderID(ids[i]);
      } 

      return this;
    }

    public SwerveDrivetrainConfig setIDs(DrivetrainIDs ids){
      setDriveIDs(ids.getDrive());
      setSteerIDs(ids.getSteer());
      setEncoderIDs(ids.getEncoders());
      return this;
    }

    public SwerveDrivetrainConfig setOffsets(double... offsets){
      if (offsets.length != modules.length) {
        throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
      }

      for (int i = 0; i < offsets.length; i++) {
        modules[i].setOffset(offsets[i]);
      } 
      return this;
    }
  }

  public SwerveDrivetrain(IMU imu, SwerveModuleConfig... modules) {
    this.imu = imu;

    enableDriftCorrection = false; 
    driftpid = new PIDController(0, 0,0);
    driftpid.enableContinuousInput(-Math.PI, Math.PI);

    swerveModules = new SwerveModule[modules.length];
    for (int i = 0; i < modules.length; i++) {
      swerveModules[i] = new SwerveModule(modules[i]);

      maxVelocity = modules[i].maxSpeedMetersPerSecond();
    }
    Translation2d[] moduleLocations = new Translation2d[swerveModules.length];
    for (int i = 0; i < modules.length; i++) {
      moduleLocations[i] = swerveModules[i].getModuleLocation();
    }

    kinematics = new SwerveDriveKinematics(moduleLocations);
    robotSpeeds = new RobotSpeeds(maxVelocity, maxVelocity);
  }

  public SwerveDrivetrain withDriftCorretion(PIDController controller){
    driftpid = controller;
    enableDriftCorrection = true;
    return this;
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
  public IMU getIMU() {
      return imu;
  }

  @Override
  public void setNeutralMode(MotorNeutralMode mode) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setNeutralMode(mode);
    }
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

    SwerveDriveKinematics.desaturateWheelSpeeds(states, getMaxVelocity());

    setModuleStates(states);    
  }

  @Override
  public Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    return new SwerveDriveCommand(this, xInput, yInput, thetaInput);
  }

  @Override
  public void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    this.setDefaultCommand(getDriveCommand(xInput, yInput, thetaInput));
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
      chassisLayout.addDouble("X", () -> robotSpeeds.getDriverSpeeds().vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      chassisLayout.addDouble("Y", () -> robotSpeeds.getDriverSpeeds().vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      chassisLayout.addDouble("Z", () -> robotSpeeds.getDriverSpeeds().omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      ShuffleboardLayout feedbackLayout = speedsLayout.getLayout("Feedback", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 3,"Label position", "TOP"));
      feedbackLayout.addDouble("X", () -> robotSpeeds.getFeedbackSpeeds().vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      feedbackLayout.addDouble("Y", () -> robotSpeeds.getFeedbackSpeeds().vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
      feedbackLayout.addDouble("Z", () -> robotSpeeds.getFeedbackSpeeds().omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
    }

    ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands",BuiltInLayouts.kGrid).withSize(1, 3).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    {
      commandsLayout.add("Reset Heading", new InstantCommand(() -> getIMU().setYaw(0))).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Brake Mode", new InstantCommand(() -> setNeutralMode(MotorNeutralMode.Brake))).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Coast Mode", new InstantCommand(() -> setNeutralMode(MotorNeutralMode.Coast))).withWidget(BuiltInWidgets.kCommand);
    }

    ShuffleboardLayout driftCorrectionLayout = tab.getLayout("Drift Correction", BuiltInLayouts.kList).withSize(2, 2);
    {
      driftCorrectionLayout.add("Drift Correction", (builder) -> builder.addBooleanProperty("Drift Correction", this::getDriftCorrectionMode, this::setDriftCorrectionMode)).withWidget(BuiltInWidgets.kBooleanBox);
      driftCorrectionLayout.addDouble("Desired Heading", () -> desiredHeading).withWidget(BuiltInWidgets.kGyro);// might not display properly bc get heading is +-180
      driftCorrectionLayout.add(driftpid).withWidget(BuiltInWidgets.kPIDController);
    }
    return tab;
  }

  @Override
  public void periodic() {
      update();
  }
}