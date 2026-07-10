package ca.frc6390.athena.benchmarks;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.util.concurrent.TimeUnit;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Fork;
import org.openjdk.jmh.annotations.Measurement;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.annotations.Warmup;
import org.openjdk.jmh.infra.Blackhole;

/**
 * Baseline provider-neutral simulation stepping benchmark.
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
@Warmup(iterations = 3, time = 1)
@Measurement(iterations = 5, time = 1)
@Fork(1)
public class SimSteppingBenchmark {
    /**
     * Benchmark state.
     */
    @State(Scope.Thread)
    public static class SimState {
        private SimulationSession session;
        private EncoderDevice lastEncoder;

        /**
         * Creates modeled and unmodeled simulated hardware.
         */
        @Setup
        public void setup() {
            session = SimulationSession.create();
            for (int i = 0; i < 48; i++) {
                MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, i + 1);
                EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, i + 1);
                DigitalInputDevice limit = DigitalInputDevice.rio(i);
                session.model("axis-" + i, SimModel.flywheel(motor)
                        .encoder(encoder)
                        .gearRatio(GearRatio.reduction(12.0, 1.0))
                        .momentOfInertia(0.002)
                        .limit(limit, 3.0, 0.25));
                MotorHandle handle = session.hardwareGraph().motor(motor);
                handle.setPercentOutput(i % 2 == 0 ? 0.6 : -0.4);
                lastEncoder = encoder;
            }
            for (int i = 0; i < 48; i++) {
                MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, i + 101);
                MotorHandle handle = session.hardwareGraph().motor(motor);
                handle.setVelocityTargetRotationsPerSecond(2.0);
            }
            for (int i = 0; i < 4; i++) {
                session.imu(ImuDevice.of(ImuKinds.PIGEON_2, i + 1))
                        .yawRateDegreesPerSecond(45.0);
            }
            session.step(0.02);
        }
    }

    /**
     * Advances simulated hardware.
     *
     * @param state benchmark state
     * @param blackhole blackhole
     */
    @Benchmark
    public void step(SimState state, Blackhole blackhole) {
        state.session.step(0.02);
        blackhole.consume(state.session.encoder(state.lastEncoder).positionRotations());
    }
}
