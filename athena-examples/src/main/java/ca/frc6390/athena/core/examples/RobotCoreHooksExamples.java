package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.OutputType;
import java.util.List;

/**
 * Example hook/input setup for RobotCore lifecycle and loop profiles.
 */
public final class RobotCoreHooksExamples {
    private RobotCoreHooksExamples() {}

    public static RobotCoreHooks<SwerveDrivetrain> createHooks(List<String> timeline) {
        return RobotCoreHooks.<SwerveDrivetrain>empty()
                .hooks(hooks -> {
                    hooks.onInit(ctx -> timeline.add("robot.init"));
                    hooks.onPeriodic(ctx -> timeline.add("robot.periodic"));
                    hooks.onPeriodic(ctx -> timeline.add("robot.periodic.50ms"), 50.0);
                    hooks.onTeleInit(ctx -> timeline.add("tele.init.alias"));
                    hooks.onTeleopInit(ctx -> timeline.add("tele.init"));
                    hooks.onAutoInit(ctx -> timeline.add("auto.init.alias"));
                    hooks.onAutonomousInit(ctx -> timeline.add("auto.init"));
                    hooks.controlLoop("headingHold", 20.0, ctx -> 0.0);
                    hooks.pidProfile("headingPid", OutputType.PERCENT, 1.0, 0.0, 0.0, 0.0, 1.0);
                    hooks.feedforwardProfile("driveVoltage", 0.1, 0.2, 0.0);
                })
                .inputs(inputs -> {
                    inputs.boolVal("slowMode", () -> false);
                    inputs.doubleVal("speedScale", () -> 0.65);
                    inputs.intVal("selectedLane", () -> 2);
                    inputs.stringVal("driver", () -> "pilot");
                });
    }
}
