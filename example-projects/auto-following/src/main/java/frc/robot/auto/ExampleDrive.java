package frc.robot.auto;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Minimal drivetrain boundary used by both vendor examples. */
public final class ExampleDrive extends SubsystemBase {
    private final PIDController choreoX = new PIDController(4.0, 0.0, 0.0);
    private final PIDController choreoY = new PIDController(4.0, 0.0, 0.0);
    private final PIDController choreoHeading = new PIDController(3.0, 0.0, 0.0);
    private Pose2d pose = new Pose2d();
    private ChassisSpeeds measuredSpeeds = new ChassisSpeeds();

    public ExampleDrive() {
        choreoHeading.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Pose2d pose() {
        return pose;
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
    }

    public ChassisSpeeds robotRelativeSpeeds() {
        return measuredSpeeds;
    }

    /** Replace this body with module-state or drivetrain request output. */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        measuredSpeeds = speeds;
    }

    /** Choreo gives field-relative feedforward velocities plus the desired pose. */
    public void followChoreoSample(SwerveSample sample) {
        ChassisSpeeds fieldRelative = new ChassisSpeeds(
                sample.vx + choreoX.calculate(pose.getX(), sample.x),
                sample.vy + choreoY.calculate(pose.getY(), sample.y),
                sample.omega + choreoHeading.calculate(pose.getRotation().getRadians(), sample.heading));
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, pose.getRotation()));
    }

    /**
     * Loads PathPlanner's RobotConfig from deploy/pathplanner/settings.json.
     * Call once during robot configuration, after the GUI settings file is deployed.
     */
    public void configurePathPlanner() {
        try {
            AutoBuilder.configure(
                    this::pose,
                    this::resetPose,
                    this::robotRelativeSpeeds,
                    this::driveRobotRelative,
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(4.0, 0.0, 0.0)),
                    RobotConfig.fromGUISettings(),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception exception) {
            throw new IllegalStateException("PathPlanner GUI settings could not be loaded", exception);
        }
    }
}
