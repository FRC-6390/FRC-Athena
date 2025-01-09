package ca.frc6390.athena.drivetrains.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
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
  private static Pigeon2 gyro;
  private static SwerveDriveKinematics kinematics;
  private static PIDController driftpid;
  private ChassisSpeeds speeds, feedbackSpeeds;
  private static boolean enableDriftCorrection;
  private double desiredHeading;

  public SwerveDrivetrain(SwerveModuleConfig[] configs, int gyro) {
    this(configs, gyro, false, new PIDController(0, 0, 0));
  }

  public SwerveDrivetrain(SwerveModuleConfig[] configs, int gyroId, boolean driftCorrection,
      PIDController driftCorrectionPID) {
    speeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();
    swerveModules = new SwerveModule[configs.length];
    for (int i = 0; i < configs.length; i++) {
      swerveModules[i] = new SwerveModule(configs[i]);
    }
    gyro = new Pigeon2(gyroId, "can");
    Translation2d[] moduleLocations = new Translation2d[swerveModules.length];
    for (int i = 0; i < configs.length; i++) {
      moduleLocations[i] = swerveModules[i].getModuleLocation();
    }
    kinematics = new SwerveDriveKinematics(moduleLocations);
    enableDriftCorrection = driftCorrection;
    driftpid = driftCorrectionPID;
  }

  public double getAngularVelocityZ() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public void resetHeading() {
    gyro.setYaw(0);
  }

  public double getRoll() {
    return Math.IEEEremainder(gyro.getRoll().refresh().getValueAsDouble(), 360);
  }

  public double getPitch() {
    return Math.IEEEremainder(gyro.getPitch().refresh().getValueAsDouble(), 360);
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw().refresh().getValueAsDouble(), 360);
  }

  public ChassisSpeeds getSpeeds() {
    return speeds;
  }

  public void zeroHeading() {
    gyro.setYaw(0);
  }

  public void setHeading(double heading) {
    gyro.setYaw(heading);
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

  private void driftCorrection(ChassisSpeeds speeds) {
    if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0) {
      desiredHeading = getHeading();
    } else {
      speeds.omegaRadiansPerSecond += driftpid.calculate(desiredHeading);
    }
    ;
  }

  public void update() {
    ChassisSpeeds speed = speeds.plus(feedbackSpeeds);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

    if (enableDriftCorrection) {
      driftCorrection(speed);
    }

    setModuleStates(states);

  }

  public void setDriftCorrectionMode(boolean enabled) {
    enableDriftCorrection = enabled;
  }

  public boolean getDriftCorrectionMode() {
    return enableDriftCorrection;
  }

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {

    ShuffleboardLayout swervelayout = tab.getLayout("Swerve Modules", BuiltInLayouts.kGrid).withSize(2, 6);
    {
      int x = 0, y = 0;
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].shuffleboard(swervelayout.getLayout("Module " + i)).withPosition(x, y);

        if (x < 1) {
          x++;
        } else {
          x = 0;
          y += 3;
        }
      }
    }

    tab.addDouble("Swerve Modules", this::getHeading).withWidget(BuiltInWidgets.kGyro); // might not display properly bc
                                                                                              // get heading is +-180
    ShuffleboardLayout speedsLayout = tab.getLayout("Robot Speeds", BuiltInLayouts.kGrid).withSize(2, 3);
    {
      ShuffleboardLayout chassisLayout = speedsLayout.getLayout("Chassis", BuiltInLayouts.kList);
      chassisLayout.addDouble("X", () -> speeds.vxMetersPerSecond);
      chassisLayout.addDouble("Y", () -> speeds.vyMetersPerSecond);
      chassisLayout.addDouble("Z", () -> speeds.omegaRadiansPerSecond);
      ShuffleboardLayout feedbackLayout = speedsLayout.getLayout("Feedback", BuiltInLayouts.kList);
      feedbackLayout.addDouble("X", () -> feedbackSpeeds.vxMetersPerSecond);
      feedbackLayout.addDouble("Y", () -> feedbackSpeeds.vyMetersPerSecond);
      feedbackLayout.addDouble("Z", () -> feedbackSpeeds.omegaRadiansPerSecond);
    }

    ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands", BuiltInLayouts.kList);
    {
      commandsLayout.add("Reset Heading", new InstantCommand(this::resetHeading));
      commandsLayout.add("Brake Mode", new InstantCommand(() -> setNeutralMode(NeutralModeValue.Brake)));
      commandsLayout.add("Coast Mode", new InstantCommand(() -> setNeutralMode(NeutralModeValue.Coast)));
    }

    ShuffleboardLayout driftCorrectionLayout = tab.getLayout("Drift Correction", BuiltInLayouts.kList);
    {
      driftCorrectionLayout.add((builder) -> builder.addBooleanProperty("Drift Correction", this::getDriftCorrectionMode, this::setDriftCorrectionMode)).withWidget(BuiltInWidgets.kBooleanBox);
      driftCorrectionLayout.addDouble("Desired Heading", () -> desiredHeading).withWidget(BuiltInWidgets.kGyro);// might not display properly bc get heading is +-180
      driftCorrectionLayout.add(driftpid).withWidget(BuiltInWidgets.kPIDController);
    }
    return tab;
  }
}