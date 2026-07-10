package ca.frc6390.athena.benchmarks;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraDevice;
import java.util.List;
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
 * Baseline full Athena root runtime benchmark.
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
@Warmup(iterations = 3, time = 1)
@Measurement(iterations = 5, time = 1)
@Fork(1)
public class FullRobotPeriodicBenchmark {
    /**
     * Benchmark state.
     */
    @State(Scope.Thread)
    public static class RuntimeState {
        private RobotRuntime runtime;
        private double nowSeconds;

        /**
         * Creates a representative robot runtime.
         */
        @Setup
        public void setup() {
            runtime = RobotRuntime.simulated(SimulationSession.create());
            for (int i = 0; i < 12; i++) {
                MotorMechanism mechanism = new MotorMechanism(i * 4);
                runtime.register(mechanism);
                runtime.request(mechanism.drive);
            }
            ca.frc6390.athena.runtime.measurement.Measurement target = Measurements.custom("target", null);
            CameraDevice camera = Cameras.photonVision("front").bindTargets(() -> List.of(target));
            Localization localization = Localizations.latestValid()
                    .input(() -> List.of(Measurements.pose(new PoseSnapshot(1.0, 2.0, 0.25))));
            runtime.cameras(camera).localization(localization);
            runtime.robotPeriodic(0.0, 0.02);
        }
    }

    /**
     * Runs one root robot periodic tick.
     *
     * @param state benchmark state
     * @param blackhole blackhole
     */
    @Benchmark
    public void rootRobotPeriodic(RuntimeState state, Blackhole blackhole) {
        state.nowSeconds += 0.02;
        List<ResolvedOutput> outputs = state.runtime.robotPeriodic(state.nowSeconds, 0.02);
        blackhole.consume(outputs.size());
        blackhole.consume(state.runtime.localizationMeasurements().size());
    }

    private static final class MotorMechanism implements Mechanism {
        private final MotorDevice a;
        private final MotorDevice b;
        private final Action drive;

        private MotorMechanism(int baseId) {
            a = MotorDevice.of(MotorKinds.KRAKEN_X60, baseId + 1);
            b = MotorDevice.of(MotorKinds.KRAKEN_X60, baseId + 2);
            drive = Actions.parallel(a.percent(0.25), b.percent(-0.25));
        }
    }
}
