package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import ca.frc6390.athena.robot.RuntimeInputs;
import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.ReplaySimulation;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.wpilibj.XboxController;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Records a simulated driver pressing A, then replays the recording through the same and through modified robot code.
 */
class ReplayInputsTest {
    private static final int CYCLES = 60;
    private static final double DT = 0.02;
    private static final int PRESS_FROM = 10;
    private static final int PRESS_UNTIL = 30;

    @TempDir
    Path directory;

    @Test
    void sameBindingsReplayWithZeroDifferences() throws Exception {
        Path file = record(XboxController.Button.kA.value);

        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new Shooter(XboxController.Button.kA.value)))
                .report();

        assertEquals(CYCLES, report.cyclesReplayed());
        assertTrue(report.commandsCompared() > 0);
        assertTrue(report.identical(), () -> "differences: " + report.differences());
    }

    @Test
    void bindingTheButtonToAnotherActionShowsDifferences() throws Exception {
        Path file = record(XboxController.Button.kA.value);

        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new Shooter(XboxController.Button.kB.value)))
                .report();

        assertFalse(report.identical());
        assertTrue(report.differenceCount() > 0);
        assertEquals(0.5, report.differences().get(0).recordedValue(), 1.0e-9);
        assertEquals(0.0, report.differences().get(0).replayedValue(), 1.0e-9);
    }

    @Test
    void replayReleasesInjectedInputWhenItEnds() throws Exception {
        Path file = record(XboxController.Button.kA.value);
        ReplaySimulation.replay(file)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new Shooter(XboxController.Button.kA.value)));

        assertFalse(RuntimeInputs.injected().isPresent());
    }

    /** Runs the simulated driver: A is held for cycles 10 to 29. Returns the flushed recording. */
    private Path record(int button) throws IOException, InterruptedException {
        SimulationSession session = SimulationSession.create();
        Shooter shooter = new Shooter(button);
        RobotRuntime runtime = RobotRuntime.simulated(session).register(shooter);
        boolean[] pressed = {false};
        runtime.inputs(() -> driver(pressed[0]));
        runtime.replay(ReplayPolicy.of(), directory);
        for (int cycle = 0; cycle < CYCLES; cycle++) {
            double now = cycle * DT;
            pressed[0] = cycle >= PRESS_FROM && cycle < PRESS_UNTIL;
            RuntimeInputs.inject(driver(pressed[0]));
            try {
                runtime.periodic(
                        new MechanismContext(now, 0.0, DT, true, false, true),
                        new EventContext(now, DT, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
            } finally {
                RuntimeInputs.release();
            }
            runtime.simulationPeriodic(now, DT);
        }
        var recorder = runtime.replayRecorder().orElseThrow();
        recorder.flush().orElseThrow();
        assertTrue(recorder.awaitFlushed(Duration.ofSeconds(10)));
        return recorder.lastFile().orElseThrow();
    }

    private static RuntimeInputSnapshot driver(boolean aPressed) {
        RuntimeInputSnapshot.Joystick pad = new RuntimeInputSnapshot.Joystick(
                0,
                true,
                "Xbox",
                1,
                new double[6],
                aPressed ? 1L : 0L,
                new int[] {-1, -1, -1, -1});
        return new RuntimeInputSnapshot(List.of(pad), new RuntimeInputSnapshot.DriverStationState(
                true, false, false, false, true, "", 120.0, "", "NONE", 0, ""));
    }

    /** Robot code: the button given to the constructor runs the intake motor at 50 percent while held. */
    private static final class Shooter implements Mechanism {
        final Gamepad driver = new Gamepad(new XboxController(0));
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 91);
        final Action spin = motor.percent(0.5);

        Shooter(int button) {
            if (button == XboxController.Button.kA.value) {
                driver.a().whileHeld(spin);
            } else {
                driver.b().whileHeld(spin);
            }
        }
    }
}
