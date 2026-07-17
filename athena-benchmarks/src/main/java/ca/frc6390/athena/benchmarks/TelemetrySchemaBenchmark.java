package ca.frc6390.athena.benchmarks;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismScheduler;
import ca.frc6390.athena.mechanism.core.MechanismTraceLevel;
import ca.frc6390.athena.mechanism.core.TelemetrySchema;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.mechanism.motion.MotionProfiles;
import ca.frc6390.athena.sim.runtime.SimulationSession;
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

/** Measures the cached telemetry schema and nested trace costs independently of NT4. */
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
@Warmup(iterations = 3, time = 1)
@Measurement(iterations = 5, time = 1)
@Fork(1)
public class TelemetrySchemaBenchmark {
    /** Representative mechanism tree with device, control, value, and action telemetry. */
    @State(Scope.Thread)
    public static class TelemetryState {
        private MechanismScheduler scheduler;
        private TelemetrySchema schema;
        private double timestampSeconds;

        /** Builds and warms the cached schema and runtime trace. */
        @Setup
        public void setup() {
            scheduler = MechanismScheduler.create(SimulationSession.create().hardwareGraph());
            RobotMechanism robot = new RobotMechanism();
            scheduler.register(robot);
            scheduler.bindInMemoryRuntime();
            robot.mechanisms.forEach(mechanism -> scheduler.request(mechanism.hold));
            scheduler.robotPeriodic(0.0, 0.02);
            schema = scheduler.telemetrySchema();
        }
    }

    /** Runtime state parameterized by the production trace detail level. */
    @State(Scope.Thread)
    public static class TraceState {
        @Param({"OFF", "SUMMARY", "CAPTURE"})
        private MechanismTraceLevel traceLevel;
        private MechanismScheduler scheduler;
        private double timestampSeconds;

        /** Builds a representative nested runtime using the selected trace level. */
        @Setup
        public void setup() {
            scheduler = MechanismScheduler.create(SimulationSession.create().hardwareGraph());
            RobotMechanism robot = new RobotMechanism();
            scheduler.register(robot).traceLevel(traceLevel).bindInMemoryRuntime();
            robot.mechanisms.forEach(mechanism -> scheduler.request(mechanism.hold));
            scheduler.robotPeriodic(0.0, 0.02);
        }
    }

    /** Measures the intended allocation-free cached schema lookup. */
    @Benchmark
    public void cachedSchema(TelemetryState state, Blackhole blackhole) {
        blackhole.consume(state.scheduler.telemetrySchema());
    }

    /** Samples every endpoint reader without NetworkTables transport overhead. */
    @Benchmark
    public void sampleAllValues(TelemetryState state, Blackhole blackhole) {
        for (TelemetryValue value : state.schema.values().values()) {
            blackhole.consume(value.value());
        }
    }

    /** Materializes root and child traces after a representative runtime tick. */
    @Benchmark
    public void materializeChildTraces(TraceState state, Blackhole blackhole) {
        state.timestampSeconds += 0.02;
        state.scheduler.robotPeriodic(state.timestampSeconds, 0.02);
        blackhole.consume(state.scheduler.traceSnapshots());
    }

    /** Isolates child trace filtering and copying from the scheduler tick itself. */
    @Benchmark
    public void materializeCachedChildTraces(TraceState state, Blackhole blackhole) {
        blackhole.consume(state.scheduler.traceSnapshots());
    }

    private static final class RobotMechanism implements Mechanism {
        private final List<AxisMechanism> mechanisms = java.util.stream.IntStream.range(0, 12)
                .mapToObj(AxisMechanism::new)
                .toList();
    }

    private static final class AxisMechanism implements Mechanism {
        private final MotorDevice motor;
        private final EncoderDevice encoder;
        private final ControlBinding position;
        public final Action hold;

        private final TelemetryValue testOutput = TelemetryValue.number(0.25);

        private AxisMechanism(int index) {
            motor = MotorDevice.of(MotorKinds.KRAKEN_X60, index + 1);
            encoder = motor.encoder().conversion(360.0);
            position = Controls.position(motor)
                    .feedback(encoder)
                    .pid(0.1, 0.0, 0.0)
                    .constraint(Constraints.range(Range.of(-180.0, 180.0)))
                    .profile(MotionProfiles.trapezoid(180.0, 360.0));
            hold = position.position(0.0);
        }
    }
}
