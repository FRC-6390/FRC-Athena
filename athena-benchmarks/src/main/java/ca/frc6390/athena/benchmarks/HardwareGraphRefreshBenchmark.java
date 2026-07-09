package ca.frc6390.athena.benchmarks;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
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

/**
 * Baseline cached hardware graph input refresh benchmark.
 */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
@Warmup(iterations = 3, time = 1)
@Measurement(iterations = 5, time = 1)
@Fork(1)
public class HardwareGraphRefreshBenchmark {
    /**
     * Benchmark state.
     */
    @State(Scope.Thread)
    public static class GraphState {
        private HardwareGraph graph;

        /**
         * Materializes a medium-sized graph.
         */
        @Setup
        public void setup() {
            graph = SimulationSession.create().hardwareGraph();
            for (int i = 0; i < 96; i++) {
                MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, i + 1);
                graph.motor(motor);
                graph.encoder(motor.encoder());
            }
            for (int i = 0; i < 24; i++) {
                graph.encoder(EncoderDevice.of(EncoderKinds.CANCODER, i + 1));
            }
            for (int i = 0; i < 4; i++) {
                graph.imu(ImuDevice.of(ImuKinds.PIGEON_2, i + 1));
            }
            graph.refreshInputs();
        }
    }

    /**
     * Refreshes all cached graph handles.
     *
     * @param state benchmark state
     */
    @Benchmark
    public void refreshInputs(GraphState state) {
        state.graph.refreshInputs();
    }
}
