package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/** Connects PathPlanner's global AutoBuilder to ordinary Athena drive Actions. */
public final class PathPlannerSetup implements Mechanism {
    private final ExampleDrive drive;

    public final HookBinding configure = Events.robotInit().onStart(this::configureAutoBuilder);

    public PathPlannerSetup(ExampleDrive drive) {
        this.drive = drive;
    }

    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                    drive::pose,
                    pose -> drive.resetPose(pose).request(),
                    ChassisSpeeds::new,
                    (speeds, feedforwards) -> {
                        drive.autoVelocity.set(RobotVelocity.robot(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond));
                        drive.pooledDrive.request();
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(4.0, 0.0, 0.0),
                            new PIDConstants(3.0, 0.0, 0.0)),
                    RobotConfig.fromGUISettings(),
                    () -> DriverStation.getAlliance()
                            .map(alliance -> alliance == DriverStation.Alliance.Red)
                            .orElse(false));
        } catch (IOException | ParseException exception) {
            throw new IllegalStateException(
                    "Create src/main/deploy/pathplanner/settings.json with the PathPlanner GUI.",
                    exception);
        }
    }
}
