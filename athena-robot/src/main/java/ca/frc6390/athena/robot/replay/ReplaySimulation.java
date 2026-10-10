package ca.frc6390.athena.robot.replay;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.ResolvedOutput;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.sim.runtime.SimulationInjector;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

/**
 * Replays a recorded {@code .athrp} run against robot code in simulation.
 *
 * <p>For each recorded mechanism cycle the recorded hardware inputs (motor position, velocity and electrical
 * readings, encoders, digital inputs) are written into a fresh {@link SimulationSession} instead of simulated
 * physics. The robot is then run for the same timestamp, lifecycle mode and phase, and its motor commands are
 * compared with the recorded commands.</p>
 *
 * <pre>{@code
 * ReplaySimulation.ReplayReport report = ReplaySimulation.replay(file)
 *         .runAgainst(session -> RobotRuntime.simulated(session).register(new MyMechanism()))
 *         .report();
 * }</pre>
 *
 * <p>Not replayed: IMU inputs, motor connection and failure flags, and localization measurements. Replay
 * cycles that run no mechanisms (physics-only frames) are skipped.</p>
 */
public final class ReplaySimulation {
    /** Differences beyond this count are counted but not listed. */
    public static final int MAX_LISTED_DIFFERENCES = 1000;
    static final double VALUE_TOLERANCE = 1.0e-9;

    private static final String LIFECYCLE = "lifecycle/";
    private static final String MOTOR_PREFIX = "hardware/motor/";
    private static final String ENCODER_PREFIX = "hardware/encoder/";
    private static final String DIGITAL_PREFIX = "digital/";
    private static final String COMMAND_PREFIX = "command/";

    private ReplaySimulation() {
    }

    /**
     * Starts a replay of a file.
     *
     * @param file replay file
     * @return replay that can be run against robot code
     */
    public static Replay replay(Path file) {
        return new Replay(Objects.requireNonNull(file, "file"));
    }

    /** A replay file ready to run against a robot. */
    public static final class Replay {
        private final Path file;

        private Replay(Path file) {
            this.file = file;
        }

        /**
         * Runs the recorded inputs through a new robot runtime.
         *
         * @param robot builds a simulated runtime on the session it is given; the session carries the
         *     injected inputs, so the robot must be created with {@code RobotRuntime.simulated(session)}
         * @return the completed run
         * @throws UncheckedIOException when the file cannot be read
         */
        public ReplayRun runAgainst(Function<SimulationSession, RobotRuntime> robot) {
            Objects.requireNonNull(robot, "robot");
            try (ReplayReader reader = ReplayReader.open(file)) {
                return new ReplayRun(run(reader, robot));
            } catch (IOException exception) {
                throw new UncheckedIOException("Cannot replay " + file + ".", exception);
            }
        }
    }

    /** The result of a completed replay. */
    public static final class ReplayRun {
        private final ReplayReport report;

        private ReplayRun(ReplayReport report) {
            this.report = report;
        }

        /** Returns the comparison report. */
        public ReplayReport report() {
            return report;
        }
    }

    /**
     * Comparison result.
     *
     * @param frames frames read from the file
     * @param cyclesReplayed mechanism cycles run against the robot
     * @param commandsCompared motor command comparisons made
     * @param differenceCount total differences found
     * @param differences the first {@link #MAX_LISTED_DIFFERENCES} differences
     */
    public record ReplayReport(
            long frames,
            long cyclesReplayed,
            long commandsCompared,
            long differenceCount,
            List<CommandDifference> differences) {
        public ReplayReport {
            differences = List.copyOf(differences);
        }

        /** True when every compared command matched the recording. */
        public boolean identical() {
            return differenceCount == 0;
        }
    }

    /**
     * One motor command that differs between the recording and the re-run.
     *
     * @param sequence recorded frame sequence
     * @param timestampSeconds recorded timestamp
     * @param motor motor name
     * @param recordedMode recorded command mode
     * @param recordedValue recorded command value
     * @param replayedMode command mode from the re-run
     * @param replayedValue command value from the re-run
     */
    public record CommandDifference(
            long sequence,
            double timestampSeconds,
            String motor,
            String recordedMode,
            double recordedValue,
            String replayedMode,
            double replayedValue) {
    }

    private static ReplayReport run(ReplayReader reader, Function<SimulationSession, RobotRuntime> robot)
            throws IOException {
        SimulationSession session = SimulationSession.create();
        RobotRuntime runtime = Objects.requireNonNull(robot.apply(session), "robot runtime");
        SimulationInjector injector = session.injector();
        ReplayReader.Cursor cursor = reader.cursor();
        Inputs inputs = new Inputs();
        long frames = 0;
        long cycles = 0;
        long compared = 0;
        long differenceCount = 0;
        List<CommandDifference> differences = new ArrayList<>();
        while (cursor.next()) {
            if (!cursor.isFrame()) {
                continue;
            }
            frames++;
            inputs.refresh(cursor);
            inject(injector, cursor, inputs);
            if (!cursor.bool(LIFECYCLE + "mechanismsRan")) {
                continue;
            }
            cycles++;
            double timestamp = cursor.f64(LIFECYCLE + "timestampSeconds");
            double dt = cursor.f64(LIFECYCLE + "dtSeconds");
            boolean enabled = cursor.bool(LIFECYCLE + "enabled");
            boolean autonomous = cursor.bool(LIFECYCLE + "autonomous");
            LifecycleMode mode = replayMode(LifecycleMode.valueOf(cursor.string(LIFECYCLE + "mode")),
                    enabled, autonomous);
            LifecyclePhase phase = LifecyclePhase.valueOf(cursor.string(LIFECYCLE + "phase"));
            List<ResolvedOutput> outputs = runtime.periodic(
                    new MechanismContext(timestamp, 0.0, dt, enabled, autonomous, true),
                    new EventContext(timestamp, dt, mode, phase, enabled, true));
            Map<String, ResolvedCommand> replayed = commands(outputs);
            for (String motor : inputs.commandMotors) {
                String recordedMode = cursor.string(COMMAND_PREFIX + motor + "/mode");
                double recordedValue = cursor.f64(COMMAND_PREFIX + motor + "/value");
                ResolvedCommand rerun = replayed.getOrDefault(motor, ResolvedCommand.NEUTRAL);
                compared++;
                if (!recordedMode.equals(rerun.mode()) || !sameValue(recordedValue, rerun.value())) {
                    differenceCount++;
                    if (differences.size() < MAX_LISTED_DIFFERENCES) {
                        differences.add(new CommandDifference(cursor.sequence(), timestamp, motor,
                                recordedMode, recordedValue, rerun.mode(), rerun.value()));
                    }
                }
            }
        }
        return new ReplayReport(frames, cycles, compared, differenceCount, differences);
    }

    private static void inject(SimulationInjector injector, ReplayReader.Cursor cursor, Inputs inputs) {
        for (String motor : inputs.motors) {
            String base = MOTOR_PREFIX + motor + "/";
            injector.motor(motor,
                    cursor.f64(base + "positionRotations"),
                    cursor.f64(base + "velocityRotationsPerSecond"),
                    cursor.f64(base + "appliedVoltage"),
                    cursor.f64(base + "supplyCurrentAmps"),
                    cursor.f64(base + "statorCurrentAmps"));
        }
        for (String encoder : inputs.encoders) {
            String base = ENCODER_PREFIX + encoder + "/";
            injector.encoder(encoder,
                    cursor.f64(base + "positionRotations"),
                    cursor.f64(base + "absolutePositionRotations"),
                    cursor.f64(base + "velocityRotationsPerSecond"));
        }
        for (String digital : inputs.digitalInputs) {
            injector.digitalInput(digital, cursor.bool(DIGITAL_PREFIX + digital + "/active"));
        }
    }

    static LifecycleMode replayMode(LifecycleMode recorded, boolean enabled, boolean autonomous) {
        if (recorded != LifecycleMode.SIMULATION) {
            return recorded;
        }
        if (!enabled) {
            return LifecycleMode.DISABLED;
        }
        return autonomous ? LifecycleMode.AUTONOMOUS : LifecycleMode.TELEOP;
    }

    private static Map<String, ResolvedCommand> commands(List<ResolvedOutput> outputs) {
        Map<String, ResolvedCommand> commands = new HashMap<>();
        for (ResolvedOutput resolved : outputs) {
            for (ca.frc6390.athena.hardware.device.MotorDevice motor : resolved.request().motors()) {
                commands.put(motor.defaultName(), new ResolvedCommand(
                        ReplayEncoder.outputMode(resolved.output()),
                        ReplayEncoder.outputValue(resolved.output())));
            }
        }
        return commands;
    }

    private static boolean sameValue(double recorded, double replayed) {
        if (Double.isNaN(recorded) || Double.isNaN(replayed)) {
            return Double.isNaN(recorded) && Double.isNaN(replayed);
        }
        return Math.abs(recorded - replayed) <= VALUE_TOLERANCE;
    }

    private record ResolvedCommand(String mode, double value) {
        static final ResolvedCommand NEUTRAL = new ResolvedCommand("neutral", 0.0);
    }

    /** Device names found in the dictionary, refreshed when it changes. */
    private static final class Inputs {
        private int version = -1;
        private final List<String> motors = new ArrayList<>();
        private final List<String> encoders = new ArrayList<>();
        private final List<String> digitalInputs = new ArrayList<>();
        private final List<String> commandMotors = new ArrayList<>();

        void refresh(ReplayReader.Cursor cursor) {
            if (cursor.dictionaryVersion() == version) {
                return;
            }
            version = cursor.dictionaryVersion();
            motors.clear();
            encoders.clear();
            digitalInputs.clear();
            commandMotors.clear();
            for (ReplaySignal signal : cursor.signals()) {
                String path = signal.path();
                if (path.startsWith(MOTOR_PREFIX) && path.endsWith("/positionRotations")) {
                    motors.add(between(path, MOTOR_PREFIX, "/positionRotations"));
                } else if (path.startsWith(ENCODER_PREFIX) && path.endsWith("/positionRotations")) {
                    encoders.add(between(path, ENCODER_PREFIX, "/positionRotations"));
                } else if (path.startsWith(DIGITAL_PREFIX) && path.endsWith("/active")) {
                    digitalInputs.add(between(path, DIGITAL_PREFIX, "/active"));
                } else if (path.startsWith(COMMAND_PREFIX) && path.endsWith("/mode")) {
                    commandMotors.add(between(path, COMMAND_PREFIX, "/mode"));
                }
            }
        }

        private static String between(String path, String prefix, String suffix) {
            return path.substring(prefix.length(), path.length() - suffix.length());
        }
    }
}
