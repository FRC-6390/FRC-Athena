package ca.frc6390.athena.robot.explore;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.RuntimeInputs;
import ca.frc6390.athena.robot.explore.ExplorationReport.Category;
import ca.frc6390.athena.robot.replay.ReplayReader;
import ca.frc6390.athena.robot.replay.ReplaySimulation;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class SimExplorerTest {
    @TempDir
    Path directory;

    /**
     * Teleop only. Button 2 requests a shot on the feeder roller and keeps it while held. Button 1 holds the roller
     * with a hook-held lease that is newer than the shot whenever it is pressed after it, so the shot gets no output
     * for as long as both are held. The bug is the shared roller, not a timing accident.
     */
    @Test
    void starvationBugIsFoundShrunkAndSaved() throws IOException {
        Path save = directory.resolve("saved");
        ExplorationReport report = SimExplorer.of(session -> RobotRuntime.simulated(session)
                        .register(new StarvedRoller()))
                .seed(3)
                .duration(60)
                .controllers(0)
                .modes(ModeSchedule.of(new ModeSchedule.Segment(LifecycleMode.TELEOP, 60)))
                .inputs(InputModel.standard().buttons(2).buttonPressesPerSecond(1.0).meanHoldSeconds(3.0)
                        .axes(0).povChangesPerSecond(0.0))
                .save(save)
                .run();

        List<ExplorationReport.Finding> starvation = report.findings(Category.STARVATION);
        assertEquals(1, starvation.size(), report::describe);
        ExplorationReport.Finding finding = starvation.get(0);
        assertTrue(finding.timeSeconds() >= 2.0, "starvation needs the threshold: " + finding.timeSeconds());
        assertTrue(finding.shrunk());
        assertFalse(finding.history().isEmpty());

        List<InputEvent> minimal = finding.minimalInputs();
        assertTrue(minimal.size() <= 4, () -> "minimal input: " + minimal);
        assertTrue(minimal.stream().anyMatch(event -> event.kind() == InputEvent.Kind.BUTTON
                && event.index() == 2 && event.value() == 1.0), () -> "button 2 press kept: " + minimal);
        assertTrue(minimal.stream().anyMatch(event -> event.kind() == InputEvent.Kind.BUTTON
                && event.index() == 1 && event.value() == 1.0), () -> "button 1 press kept: " + minimal);

        Path replay = finding.replay();
        assertNotNull(replay, report::describe);
        assertTrue(Files.isRegularFile(replay));
        ReplaySimulation.ReplayReport comparison = ReplaySimulation.replay(replay)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new StarvedRoller()))
                .report();
        assertTrue(comparison.identical(), () -> "replay differences: " + comparison.differences());
        assertTrue(markerKinds(replay).contains(101), "starvation marker planted in the recording");
    }

    /** Random modes and random input against a robot with disjoint resources and no constraints or timeouts. */
    @Test
    void cleanRobotProducesNoFindings() {
        ExplorationReport report = SimExplorer.of(session -> RobotRuntime.simulated(session)
                        .register(new CleanRobot()))
                .seed(11)
                .duration(120)
                .controllers(0)
                .modes(ModeSchedule.random())
                .inputs(InputModel.standard())
                .run();
        assertTrue(report.clean(), report::describe);
        assertTrue(report.cycles() > 5000, "cycles run: " + report.cycles());
    }

    @Test
    void sameSeedGivesSameReport() {
        SimExplorer explorer = SimExplorer.of(session -> RobotRuntime.simulated(session).register(new StarvedRoller()))
                .seed(3)
                .duration(40)
                .controllers(0)
                .modes(ModeSchedule.of(new ModeSchedule.Segment(LifecycleMode.TELEOP, 40)))
                .inputs(InputModel.standard().buttons(2).buttonPressesPerSecond(1.0).meanHoldSeconds(3.0));
        ExplorationReport first = explorer.run();
        ExplorationReport second = explorer.run();
        assertEquals(first, second);
        assertFalse(first.clean(), "the bug is present for this seed");
    }

    @Test
    void randomModeScheduleIsLegal() {
        List<ModeSchedule.Segment> segments = ModeSchedule.random().materialize(42L, 600.0);
        LifecycleMode previous = LifecycleMode.DISABLED;
        for (ModeSchedule.Segment segment : segments) {
            if (previous != LifecycleMode.DISABLED) {
                assertEquals(LifecycleMode.DISABLED, segment.mode(), "enabled modes go through disabled");
            }
            previous = segment.mode();
        }
        assertEquals(LifecycleMode.DISABLED, segments.get(0).mode());
    }

    @Test
    void shrinkerKeepsExactlyTheNeededElements() {
        List<Integer> items = new ArrayList<>();
        for (int value = 1; value <= 100; value++) {
            items.add(value);
        }
        List<Integer> minimal = Shrinker.minimize(items, candidate -> candidate.contains(17) && candidate.contains(42));
        assertEquals(List.of(17, 42), minimal);
    }

    private static Set<Integer> markerKinds(Path file) throws IOException {
        Set<Integer> kinds = new java.util.HashSet<>();
        try (ReplayReader reader = ReplayReader.open(file)) {
            ReplayReader.Cursor cursor = reader.cursor();
            while (cursor.next()) {
                if (!cursor.isFrame()) {
                    kinds.add(cursor.markerKind());
                }
            }
        }
        return kinds;
    }

    private static boolean button(int number) {
        return RuntimeInputs.injected().map(input -> input.joystick(0).button(number)).orElse(false);
    }

    private static double axis(int number) {
        return RuntimeInputs.injected().map(input -> input.joystick(0).axis(number)).orElse(0.0);
    }

    /** Teleop mechanism with the starvation bug described on the test. */
    private static final class StarvedRoller implements Mechanism {
        private final MotorDevice roller = MotorDevice.of(MotorKinds.KRAKEN_X60, 201);
        private final HookBinding shoot = Events.when(() -> button(2)).active().whileActive(roller.percent(0.8));
        private final HookBinding hold = Events.when(() -> button(1)).active().whileActive(roller.percent(-0.2));
    }

    /** Three disjoint resources, each driven by one input, with no constraints, timeouts, or paths. */
    private static final class CleanRobot implements Mechanism {
        private final MotorDevice intake = MotorDevice.of(MotorKinds.KRAKEN_X60, 202);
        private final MotorDevice shooter = MotorDevice.of(MotorKinds.KRAKEN_X60, 203);
        private final MotorDevice drive = MotorDevice.of(MotorKinds.KRAKEN_X60, 204);
        private final HookBinding intakeHook = Events.when(() -> button(1)).active().whileActive(intake.percent(0.5));
        private final HookBinding shooterHook = Events.when(() -> button(2)).active().whileActive(shooter.percent(0.9));
        private final HookBinding driveHook = Events.teleopPeriodic().whileActive(drive.percent(() -> axis(0)));
    }
}
