package ca.frc6390.athena.robot.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.nio.file.Path;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** Recorded IMU readings are written back into the simulated IMU during replay. */
class ReplayImuTest {
    private static final double DT = 0.02;
    private static final int CYCLES = 40;

    @TempDir
    Path directory;

    @Test
    void recordedImuReadingsAreInjectedIntoTheSimulatedImu() throws Exception {
        SimulationSession session = SimulationSession.create();
        ImuMechanism recordedMechanism = new ImuMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(session).register(recordedMechanism);
        runtime.replay(ReplayPolicy.of(), directory);
        for (int cycle = 0; cycle < CYCLES; cycle++) {
            double now = cycle * DT;
            session.imu(recordedMechanism.imu).yawDegrees(cycle * 1.5).pitchDegrees(-2.0)
                    .linearAccelerationG(0.0, 0.0, 1.0);
            runtime.periodic(
                    new MechanismContext(now, 0.0, DT, true, false, true),
                    new EventContext(now, DT, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, true));
            runtime.simulationPeriodic(now, DT);
        }
        Path file = ReplayRecorderTest.flushed(runtime.replayRecorder().orElseThrow());

        AtomicReference<SimulationSession> replaySession = new AtomicReference<>();
        ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
                .runAgainst(replayed -> {
                    replaySession.set(replayed);
                    return RobotRuntime.simulated(replayed).register(new ImuMechanism());
                })
                .report();

        assertEquals(CYCLES, report.cyclesReplayed());
        assertTrue(report.identical());
        // Replay does not step physics, so the simulated IMU keeps the last recorded reading.
        ImuDevice device = new ImuMechanism().imu;
        assertEquals((CYCLES - 1) * 1.5, replaySession.get().imu(device).yawDegrees(), 1.0e-9);
        assertEquals(-2.0, replaySession.get().imu(device).pitchDegrees(), 1.0e-9);
        assertEquals(1.0, replaySession.get().imu(device).linearAccelerationZG(), 1.0e-9);
    }

    private static final class ImuMechanism implements Mechanism {
        final ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 42);
    }
}
