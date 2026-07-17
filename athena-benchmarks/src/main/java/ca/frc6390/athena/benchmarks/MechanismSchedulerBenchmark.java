package ca.frc6390.athena.benchmarks;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Fork;
import org.openjdk.jmh.annotations.Measurement;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Param;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.annotations.Warmup;
import org.openjdk.jmh.infra.Blackhole;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismScheduler;
import ca.frc6390.athena.mechanism.core.MechanismTraceLevel;
import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.sim.runtime.SimulationSession;

/**
 * Baseline mechanism scheduler benchmark for Athena runtime optimization work.
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
@Warmup(iterations = 3, time = 1)
@Measurement(iterations = 5, time = 1)
@Fork(1)
public class MechanismSchedulerBenchmark {
    /**
     * Benchmark state.
     */
    @State(Scope.Thread)
    public static class RuntimeState {
        private MechanismScheduler runtime;
        private double nowSeconds;

        @Param({"OFF", "SUMMARY", "CAPTURE"})
        public MechanismTraceLevel traceLevel;

        /**
         * Creates the runtime and warms handle caches.
         */
        @Setup
        public void setup() {
            SimulationSession session = SimulationSession.create();
            runtime = MechanismScheduler.create(session.hardwareGraph()).traceLevel(traceLevel);
            for (int i = 0; i < 32; i++) {
                MotorMechanism mechanism = new MotorMechanism(i * 8);
                runtime.register(mechanism);
                runtime.request(mechanism.drive);
            }
            runtime.robotPeriodic(0.0, 0.02);
        }
    }

    /**
     * Runs one steady-state robot periodic tick.
     *
     * @param state benchmark state
     * @param blackhole blackhole
     */
    @Benchmark
    public void robotPeriodic(RuntimeState state, Blackhole blackhole) {
        state.nowSeconds += 0.02;
        List<ResolvedOutput> outputs = state.runtime.robotPeriodic(state.nowSeconds, 0.02);
        blackhole.consume(outputs.size());
    }

    private static final class MotorMechanism implements Mechanism {
        private final MotorDevice a;
        private final MotorDevice b;
        private final MotorDevice c;
        private final MotorDevice d;
        public final Action drive;

        private MotorMechanism(int baseId) {
            a = MotorDevice.of(MotorKinds.KRAKEN_X60, baseId + 1);
            b = MotorDevice.of(MotorKinds.KRAKEN_X60, baseId + 2);
            c = MotorDevice.of(MotorKinds.KRAKEN_X60, baseId + 3);
            d = MotorDevice.of(MotorKinds.KRAKEN_X60, baseId + 4);
            drive = Actions.parallel(
                    a.percent(0.35),
                    b.percent(0.35),
                    c.percent(-0.35),
                    d.percent(-0.35));
        }
    }
}
