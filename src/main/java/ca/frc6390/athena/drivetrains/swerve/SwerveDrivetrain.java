package ca.frc6390.athena.drivetrains.swerve;

import java.util.Map;

import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.core.RobotIMU;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {

  public static SwerveModule[] swerveModules;
  public static SwerveDriveKinematics kinematics;
  public static PIDController driftpid;
  public ChassisSpeeds speeds, feedbackSpeeds;
  public static boolean enableDriftCorrection, ena;
  public double desiredHeading, maxVelocity, maxAngularVelocity,  maxVelocityPrecentOutput, maxAngularVelocityPrecentOutput;
  public RobotIMU<?> imu;

  public SwerveDrivetrain(SwerveModuleConfig[] configs, RobotIMU<?> imu) {
    this(configs, imu, false, new PIDController(0, 0, 0));
  }

  public SwerveDrivetrain(SwerveModuleConfig[] configs, RobotIMU<?> imu, boolean driftCorrection,
      PIDController driftCorrectionPID) {
    speeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();
    swerveModules = new SwerveModule[configs.length];
    for (int i = 0; i < configs.length; i++) {
      swerveModules[i] = new SwerveModule(configs[i]);

      maxVelocity = configs[i].driveMotor().maxSpeedMetersPerSecond();
      maxAngularVelocity = configs[i].driveMotor().maxSpeedMetersPerSecond();
    }
    Translation2d[] moduleLocations = new Translation2d[swerveModules.length];
    for (int i = 0; i < configs.length; i++) {
      moduleLocations[i] = swerveModules[i].getModuleLocation();
    }
    kinematics = new SwerveDriveKinematics(moduleLocations);
    enableDriftCorrection = driftCorrection;
    driftpid = driftCorrectionPID;
    this.imu = imu;
    maxVelocityPrecentOutput = 1;
    maxAngularVelocityPrecentOutput = 1;

  }

  public void resetHeading() {
    imu.setYaw(0);
  }

  public double getRoll() {
    return Math.IEEEremainder(imu.getRoll(), 360);
  }

  public double getPitch() {
    return Math.IEEEremainder(imu.getPitch(), 360);
  }

  public double getHeading() {
    return Math.IEEEremainder(imu.getYaw(), 360);
  }

  public ChassisSpeeds getSpeeds() {
    return speeds;
  }

  public void zeroHeading() {
    imu.setYaw(0);
  }

  public void setHeading(double heading) {
    imu.setYaw(heading);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  private void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  public void setNeutralMode(NeutralModeValue mode) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setNeutralMode(mode);
    }
  }

  public void drive(ChassisSpeeds driveSpeeds) {
    speeds = driveSpeeds;
  }

  public void addFeedbackSpeed(ChassisSpeeds feedbackSpeeds) {
    this.feedbackSpeeds = feedbackSpeeds;
  }

  private ChassisSpeeds driftCorrection(ChassisSpeeds speeds) {
    if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0) {
      desiredHeading = getHeading();
    } else {
      speeds.omegaRadiansPerSecond += driftpid.calculate(getHeading(), desiredHeading);
    }
    return speeds;
  }

  public void setMaxVelocityPrecentOutput(double maxVelocityPrecentOutput) {
    this.maxVelocityPrecentOutput = MathUtil.clamp(maxVelocityPrecentOutput, 0, 1);
  }

  public double getMaxVelocityPrecentOutput() {
    return maxVelocityPrecentOutput;
  }

  public void setMaxAngularVelocityPrecentOutput(double maxAngularVelocityPrecentOutput) {
    this.maxAngularVelocityPrecentOutput = MathUtil.clamp(maxAngularVelocityPrecentOutput, 0, 1);
  }

  public double getMaxAngluarVelocityPrecentOutput() {
    return maxAngularVelocityPrecentOutput;
  }

  public double getMaxVelocity() {
    return maxVelocity * maxVelocityPrecentOutput;
  }

  public double getMaxAngularVelocity() {
    return maxAngularVelocity * maxAngularVelocityPrecentOutput;
  }

  public void setDriftCorrectionMode(boolean enabled) {
    enableDriftCorrection = enabled;
  }

  public boolean getDriftCorrectionMode() {
    return enableDriftCorrection;
  }


  public void update() {
    ChassisSpeeds speed = speeds.plus(feedbackSpeeds);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

    if (enableDriftCorrection) {
      speeds = driftCorrection(speed);
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(states, getMaxVelocity());
    setModuleStates(states);

    imu.update();
  }

 

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
    
    ShuffleboardLayout swervelayout = tab.getLayout("Swerve Modules", BuiltInLayouts.kGrid).withSize(4, 8).withProperties(Map.of("Number of columns", 2, "Number of rows", 2));

    {
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].shuffleboard(swervelayout.getLayout("Module " + i, BuiltInLayouts.kGrid));
      }
    }

    tab.addDouble("Heading", this::getHeading).withWidget(BuiltInWidgets.kGyro);
    ShuffleboardLayout speedsLayout = tab.getLayout("Robot Speeds", BuiltInLayouts.kGrid).withSize(2, 3).withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
    {
      ShuffleboardLayout chassisLayout = speedsLayout.getLayout("Chassis", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 3,"Label position", "TOP"));
      chassisLayout.addDouble("X", () -> speeds.vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar);
      chassisLayout.addDouble("Y", () -> speeds.vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar);
      chassisLayout.addDouble("Z", () -> speeds.omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar);
      ShuffleboardLayout feedbackLayout = speedsLayout.getLayout("Feedback", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 1, "Number of rows", 3,"Label position", "TOP"));
      feedbackLayout.addDouble("X", () -> speeds.vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", 1, "Max", 3,"Label position", "TOP"));
      feedbackLayout.addDouble("Y", () -> feedbackSpeeds.vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar);
      feedbackLayout.addDouble("Z", () -> feedbackSpeeds.omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar);
    }

    ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands",BuiltInLayouts.kGrid).withSize(1, 3).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    {
      commandsLayout.add("Reset Heading", new InstantCommand(this::resetHeading)).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Brake Mode", new InstantCommand(() -> setNeutralMode(NeutralModeValue.Brake))).withWidget(BuiltInWidgets.kCommand);
      commandsLayout.add("Coast Mode", new InstantCommand(() -> setNeutralMode(NeutralModeValue.Coast))).withWidget(BuiltInWidgets.kCommand);
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