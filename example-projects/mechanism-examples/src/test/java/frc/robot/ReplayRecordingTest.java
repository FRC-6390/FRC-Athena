package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.ReplaySimulation;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import frc.robot.mechanisms.SingleJointArm;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Records a simulated teleop run of the single-joint arm, then replays the recording against the same robot code.
 * A change to the arm's commands makes the replay report differences (see ReplaySimulation.ReplayReport).
 */
class ReplayRecordingTest {
    @TempDir
    Path directory;

    @Test
    void recordedRunReplaysWithZeroDifferences() throws IOException, InterruptedException {
        Path file = recordScoringRun();

        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new ScoringRobot()))
                .report();

        assertTrue(report.cyclesReplayed() > 0);
        assertTrue(report.commandsCompared() > 0);
        assertTrue(report.identical(), () -> "differences: " + report.differences());
    }

    /** Runs 150 cycles (3 s) of teleop. The score action runs from the start, so the arm moves and holds. */
    private Path recordScoringRun() throws IOException, InterruptedException {
        SimulationSession session = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(session).register(new ScoringRobot());
        runtime.replay(ReplayPolicy.of(), directory);
        for (int cycle = 0; cycle < 150; cycle++) {
            double now = cycle * 0.02;
            runtime.periodic(
                    new MechanismContext(now, 0.0, 0.02, true, false, true),
                    new EventContext(now, 0.02, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
            runtime.simulationPeriodic(now, 0.02);
        }
        runtime.replayRecorder().orElseThrow().flush();
        runtime.replayRecorder().orElseThrow().awaitFlushed(Duration.ofSeconds(10));
        return runtime.replayRecorder().orElseThrow().lastFile().orElseThrow();
    }

    /** Teleop hook that keeps the arm scoring while the robot is in teleop. */
    private static final class ScoringRobot implements Mechanism {
        public final SingleJointArm arm = new SingleJointArm();
        public final HookBinding run = Events.teleopPeriodic().whileActive(arm.score);
    }
}
