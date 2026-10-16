package ca.frc6390.athena.wpilib.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.LifecycleAction;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.NamedResource;
import ca.frc6390.athena.robot.RobotRuntime;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Round trips between WPILib commands and Athena Actions. Uses WPILib's real {@link CommandScheduler}, driven
 * explicitly with {@code run()}, and a runtime with one empty mechanism that owns command-only Actions.
 */
class WpilibCommandsTest {
    private CommandScheduler scheduler;

    @BeforeEach
    void setUp() {
        // WPILib refuses to schedule commands that do not run when disabled, so the simulated driver station
        // must be in teleop, as it is on a robot whose Athena runtime is in teleop.
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.notifyNewData();
        scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
    }

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    @Test
    void wrappedCommandRunsInWpilibSchedulerAndCompletesTheAction() {
        List<String> events = new ArrayList<>();
        RobotRuntime runtime = RobotRuntime.create().register(new Host());
        Command command = Commands.runOnce(() -> events.add("ran"));
        Action action = WpilibCommands.action(command);

        runtime.request(action);
        runtime.teleopPeriodic(0.0, 0.02);
        scheduler.run();
        runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(List.of("ran"), events);
        assertTrue(runtime.isActionComplete(action));
        assertFalse(scheduler.isScheduled(command));
        runtime.close();
    }

    @Test
    void cancellingWrappedCommandEndsItInWpilibScheduler() {
        RobotRuntime runtime = RobotRuntime.create().register(new Host());
        Command forever = Commands.run(() -> { });
        Action action = WpilibCommands.action(forever);

        runtime.request(action);
        runtime.teleopPeriodic(0.0, 0.02);
        scheduler.run();
        assertTrue(scheduler.isScheduled(forever));

        runtime.cancel(action);

        assertFalse(scheduler.isScheduled(forever));
        assertFalse(runtime.isActionRunning(action));
        runtime.close();
    }

    @Test
    void wrappedCommandsSharingASubsystemExcludeEachOther() {
        RobotRuntime runtime = RobotRuntime.create().register(new Host());
        Shooter shooter = new Shooter();
        Command first = Commands.run(() -> { }, shooter).withName("first");
        Command second = Commands.run(() -> { }, shooter).withName("second");
        Action firstAction = WpilibCommands.action(first);
        Action secondAction = WpilibCommands.action(second);

        runtime.request(firstAction);
        runtime.teleopPeriodic(0.0, 0.02);
        scheduler.run();
        assertTrue(scheduler.isScheduled(first));

        runtime.request(secondAction);
        runtime.teleopPeriodic(0.02, 0.02);
        scheduler.run();

        assertFalse(scheduler.isScheduled(first), "the older command must end when a shared subsystem is claimed");
        assertTrue(scheduler.isScheduled(second));
        assertFalse(runtime.isActionRunning(firstAction));
        assertTrue(runtime.isActionRunning(secondAction));
        runtime.close();
    }

    @Test
    void subsystemRequirementsMapToStableDistinctNamedResources() {
        Shooter first = new Shooter();
        Shooter second = new Shooter();

        LifecycleAction action = WpilibCommands.action(Commands.run(() -> { }, first));

        assertTrue(action.requirements().contains(WpilibCommands.resource(first)));
        assertSame(WpilibCommands.resource(first), WpilibCommands.resource(first));
        assertFalse(WpilibCommands.resource(first).equals(WpilibCommands.resource(second)));
        assertTrue(WpilibCommands.resource(first) instanceof NamedResource);
    }

    @Test
    void athenaActionRequestedAsCommandCompletesWhenTheActionFinishes() {
        RobotRuntime runtime = RobotRuntime.create().register(new Host());
        boolean[] done = {false};
        Action action = Actions.lifecycle("until-flag")
                .until(() -> done[0])
                .build();
        Command command = WpilibCommands.command(runtime, action);

        scheduler.schedule(command);
        runtime.teleopPeriodic(0.0, 0.02);
        scheduler.run();
        assertTrue(scheduler.isScheduled(command));
        assertTrue(runtime.isActionRunning(action));

        done[0] = true;
        runtime.teleopPeriodic(0.02, 0.02);
        scheduler.run();

        assertFalse(scheduler.isScheduled(command));
        assertTrue(runtime.isActionComplete(action));
        runtime.close();
    }

    @Test
    void interruptingAthenaCommandCancelsTheAction() {
        RobotRuntime runtime = RobotRuntime.create().register(new Host());
        List<String> events = new ArrayList<>();
        Action action = Actions.lifecycle("hold")
                .onEnd(interrupted -> events.add("end:" + interrupted))
                .build();
        Command command = WpilibCommands.command(runtime, action);

        scheduler.schedule(command);
        runtime.teleopPeriodic(0.0, 0.02);
        scheduler.cancel(command);
        runtime.teleopPeriodic(0.02, 0.02);

        assertEquals(List.of("end:true"), events);
        assertFalse(runtime.isActionRunning(action));
        runtime.close();
    }

    /** Owns command-only Actions; the runtime needs one registered mechanism root to host them. */
    private static final class Host implements Mechanism {
    }

    /** Stand-in subsystem; two instances share a name, so their resources must be disambiguated. */
    private static final class Shooter extends SubsystemBase {
    }
}
