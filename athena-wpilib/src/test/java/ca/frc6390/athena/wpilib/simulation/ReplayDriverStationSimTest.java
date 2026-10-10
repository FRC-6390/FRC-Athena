package ca.frc6390.athena.wpilib.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import ca.frc6390.athena.robot.RuntimeInputs;
import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.ReplayRecorder;
import ca.frc6390.athena.robot.replay.ReplaySimulation;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import java.nio.file.Path;
import java.time.Duration;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Replay drives the simulated WPILib driver station, so robot code that reads {@link DriverStation} statics directly
 * behaves as it did on the robot.
 */
class ReplayDriverStationSimTest {
    private static final int CYCLES = 60;
    private static final double DT = 0.02;

    @TempDir
    Path directory;

    @BeforeAll
    static void initializeHal() {
        // The driver station sim writes through HAL sim; HAL must be initialized first.
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void resetDriverStation() {
        ReplayDriverStationSim.reset();
    }

    @Test
    void teleopGatedRobotCodeReplaysIdenticallyWithTheRecordedDriverStation() throws Exception {
        Path file = recordTeleop();
        // The replay starts from a disabled driver station, so the recorded state has to come from the recording.
        ReplayDriverStationSim.reset();
        assertFalse(DriverStation.isTeleopEnabled());

        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .driverStation(ReplayDriverStationSim.sink())
                .runAgainst(session -> RobotRuntime.simulated(session).register(new TeleopGatedIntake()))
                .report();

        assertEquals(CYCLES, report.cyclesReplayed());
        assertTrue(report.commandsCompared() > 0);
        assertTrue(report.identical(), () -> "differences: " + report.differences());
    }

    @Test
    void withoutTheDriverStationSinkTheSameCodeSeesTheLiveDisabledState() throws Exception {
        Path file = recordTeleop();
        ReplayDriverStationSim.reset();

        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .runAgainst(session -> RobotRuntime.simulated(session).register(new TeleopGatedIntake()))
                .report();

        assertFalse(report.identical(), "DriverStation statics are not injected without the sink");
        assertEquals(0.5, report.differences().get(0).recordedValue(), 1.0e-9);
        assertEquals(0.0, report.differences().get(0).replayedValue(), 1.0e-9);
    }

    @Test
    void sinkSetsEveryRecordedDriverStationField() {
        RuntimeInputSnapshot.Joystick pad = new RuntimeInputSnapshot.Joystick(
                2, true, "Pad", 1, new double[] {0.25, -0.5}, (1L << 4) | 1L, new int[] {90});
        RuntimeInputSnapshot input = new RuntimeInputSnapshot(List.of(pad),
                new RuntimeInputSnapshot.DriverStationState(true, true, false, true, true, "blue", 87.5,
                        "2026test", "Qualification", 42, "RLB"));

        ReplayDriverStationSim.apply(input);

        assertTrue(DriverStation.isEnabled());
        assertTrue(DriverStation.isAutonomousEnabled());
        assertFalse(DriverStation.isTest());
        assertTrue(DriverStation.isFMSAttached());
        assertEquals(DriverStation.Alliance.Blue, DriverStation.getAlliance().orElseThrow());
        assertEquals(87.5, DriverStation.getMatchTime(), 1.0e-9);
        assertEquals("2026test", DriverStation.getEventName());
        assertEquals(DriverStation.MatchType.Qualification, DriverStation.getMatchType());
        assertEquals(42, DriverStation.getMatchNumber());
        assertEquals("RLB", DriverStation.getGameSpecificMessage());
        assertTrue(DriverStation.isJoystickConnected(2));
        assertEquals(0.25, DriverStation.getStickAxis(2, 0), 1.0e-6);
        assertEquals(-0.5, DriverStation.getStickAxis(2, 1), 1.0e-6);
        assertEquals(90, DriverStation.getStickPOV(2, 0));
        assertEquals((1 << 4) | 1, DriverStation.getStickButtons(2));
        assertFalse(DriverStation.isJoystickConnected(0));
    }

    /**
     * Records a teleop run where the driver station is enabled for every cycle. The recording drives the simulated
     * driver station with the same sink the replay uses, so the robot code sees the same state while recording.
     */
    private Path recordTeleop() throws Exception {
        RuntimeInputSnapshot enabled = new RuntimeInputSnapshot(List.of(),
                new RuntimeInputSnapshot.DriverStationState(true, false, false, true, true, "red", 120.0,
                        "", "NONE", 0, ""));
        SimulationSession session = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(session).register(new TeleopGatedIntake());
        runtime.replay(ReplayPolicy.of(), directory);
        runtime.inputs(() -> enabled);
        for (int cycle = 0; cycle < CYCLES; cycle++) {
            double now = cycle * DT;
            ReplayDriverStationSim.apply(enabled);
            RuntimeInputs.inject(enabled);
            try {
                runtime.periodic(
                        new MechanismContext(now, 0.0, DT, true, false, true),
                        new EventContext(now, DT, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
            } finally {
                RuntimeInputs.release();
            }
            runtime.simulationPeriodic(now, DT);
        }
        ReplayRecorder recorder = runtime.replayRecorder().orElseThrow();
        recorder.flush().orElseThrow();
        assertTrue(recorder.awaitFlushed(Duration.ofSeconds(10)));
        return recorder.lastFile().orElseThrow();
    }

    /** Robot code that only runs its intake while DriverStation reports teleop enabled. */
    private static final class TeleopGatedIntake implements Mechanism {
        final MotorDevice roller = MotorDevice.of(MotorKinds.KRAKEN_X60, 93);
        final Action intake = roller.percent(0.5);
        final HookBinding gate = Events.when(DriverStation::isTeleopEnabled).active().whileActive(intake);
    }
}
