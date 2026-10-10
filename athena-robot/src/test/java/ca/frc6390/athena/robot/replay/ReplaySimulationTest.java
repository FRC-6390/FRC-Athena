package ca.frc6390.athena.robot.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ReplaySimulationTest {
    @TempDir
    Path directory;

    /** Records a simulated run with a jam in the middle, then replays it against the same robot code. */
    @Test
    void replayOfTheSameCodeHasZeroDifferences() throws IOException {
        Path file = recordJamRun(0.8);

        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new JamClearingIntake(0.8)))
                .report();

        assertTrue(report.frames() > 0);
        assertEquals(60, report.cyclesReplayed());
        assertEquals(60, report.commandsCompared());
        assertEquals(0, report.differenceCount(), () -> "differences: " + report.differences());
        assertTrue(report.identical());
    }

    /** The recording contains the jam-clearing reverse command, so the replay exercises the stall hook. */
    @Test
    void replayReproducesTheRecordedJamResponse() throws IOException {
        Path file = recordJamRun(0.8);
        RecordedCommands recorded = RecordedCommands.read(file, new JamClearingIntake(0.8).roller.defaultName());
        assertTrue(recorded.values().contains(-0.6), "jam clear command recorded: " + recorded.values());
        assertTrue(recorded.values().contains(0.8));
        assertFalse(recorded.values().stream().anyMatch(value -> Double.isNaN(value)));
    }

    /** Replaying the recording through modified code shows the command differences. */
    @Test
    void modifiedCodeShowsCommandDifferences() throws IOException {
        Path file = recordJamRun(0.8);

        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new JamClearingIntake(0.5)))
                .report();

        assertFalse(report.identical());
        assertTrue(report.differenceCount() > 0);
        ReplaySimulation.CommandDifference first = report.differences().get(0);
        assertEquals(new JamClearingIntake(0.8).roller.defaultName(), first.motor());
        assertEquals(0.8, first.recordedValue(), 1.0e-9);
        assertEquals(0.5, first.replayedValue(), 1.0e-9);
    }

    @Test
    void replayModeMapsSimulationFramesToRobotModes() {
        assertEquals(LifecycleMode.DISABLED,
                ReplaySimulation.replayMode(LifecycleMode.SIMULATION, false, false));
        assertEquals(LifecycleMode.AUTONOMOUS,
                ReplaySimulation.replayMode(LifecycleMode.SIMULATION, true, true));
        assertEquals(LifecycleMode.TELEOP,
                ReplaySimulation.replayMode(LifecycleMode.SIMULATION, true, false));
        assertEquals(LifecycleMode.DISABLED, ReplaySimulation.replayMode(LifecycleMode.DISABLED, false, false));
    }

    /**
     * Runs 60 simulated cycles at 50 Hz with the motor locked between cycles 20 and 40. Each cycle runs teleop
     * mechanisms and then simulated physics, the same sequence AthenaRobot uses in simulation.
     */
    private Path recordJamRun(double intakePercent) throws IOException {
        SimulationSession session = SimulationSession.create();
        JamClearingIntake intake = new JamClearingIntake(intakePercent);
        RobotRuntime runtime = RobotRuntime.simulated(session).register(intake);
        runtime.replay(ReplayPolicy.of(), directory);
        for (int cycle = 0; cycle < 60; cycle++) {
            double now = cycle * 0.02;
            if (cycle == 20) session.motor(intake.roller).locked(true);
            if (cycle == 40) session.motor(intake.roller).locked(false);
            runtime.periodic(
                    new MechanismContext(now, 0.0, 0.02, true, false, true),
                    new EventContext(now, 0.02, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
            runtime.simulationPeriodic(now, 0.02);
        }
        return ReplayRecorderTest.flushed(runtime.replayRecorder().orElseThrow());
    }

    private record RecordedCommands(List<Double> values) {
        static RecordedCommands read(Path file, String motor) throws IOException {
            List<Double> values = new java.util.ArrayList<>();
            try (ReplayReader reader = ReplayReader.open(file)) {
                ReplayReader.Cursor cursor = reader.cursor();
                while (cursor.next()) {
                    if (cursor.isFrame() && cursor.bool("lifecycle/mechanismsRan")) {
                        values.add(cursor.f64("command/" + motor + "/value"));
                    }
                }
            }
            return new RecordedCommands(values);
        }
    }

    /** Same shape as the stall-clearing intake in MotorStallIntegrationTest, with a configurable command. */
    private static final class JamClearingIntake implements Mechanism {
        private final MotorDevice roller = MotorDevice.of(MotorKinds.KRAKEN_X44, 71)
                .supplyCurrentLimit(35)
                .statorCurrentLimit(50);
        private final Action intake;
        private final Action clearJam = Actions.sequence()
                .forTime(0.12, roller.percent(-0.6))
                .forTime(0.04, roller.neutral());
        private final HookBinding run;
        private final HookBinding recover = Events.when(roller.stall().forSeconds(0.06))
                .rising()
                .onStart(clearJam);

        JamClearingIntake(double percent) {
            intake = roller.percent(percent);
            run = Events.teleopPeriodic().whileActive(intake);
        }
    }
}
