package ca.frc6390.athena.drivetrains.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    this(configs, 1, true, new PIDController(0, 0, 0));
  }

  public SwerveDrivetrain(SwerveModuleConfig[] configs, int gyroPigeon2, boolean driftCorrection,
    PIDController driftCorrectionPID) {
    for (int i = 0; i < configs.length; i++) {
      swerveModules[i] = new SwerveModule(configs[i]) ;
    }
    gyro = new Pigeon2(gyroPigeon2);
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

  public void lockWheels() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
    }
  }

  public void unlockWheels() {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
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

  public void enableDriftCorrection(boolean enabled) {
    enableDriftCorrection = enabled;
  }
}