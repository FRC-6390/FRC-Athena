package ca.frc6390.athena.robot.explore;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.MechanismTraceLevel;
import ca.frc6390.athena.mechanism.core.MechanismTraceSnapshot;
import ca.frc6390.athena.mechanism.core.RuntimeEvents;
import ca.frc6390.athena.mechanism.staging.Staging;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.RuntimeFailureReporter;
import ca.frc6390.athena.robot.RuntimeInputSnapshot;
import ca.frc6390.athena.robot.RuntimeInputs;
import ca.frc6390.athena.robot.explore.ExplorationReport.Category;
import ca.frc6390.athena.robot.replay.ReplayMarkers;
import ca.frc6390.athena.robot.replay.ReplayPolicy;
import ca.frc6390.athena.robot.replay.ReplayRecorder;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.nio.file.Path;
import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;

/**
 * Executes one exploration run: a fresh simulation, a fresh robot, a fixed mode sequence and a fixed input
 * script. Input reaches the robot exactly as a replay delivers it: {@link RuntimeInputs#inject} for the
 * controller reads of each cycle, and {@link RobotRuntime#inputs} so the replay recorder captures the same
 * snapshot. Package-private; {@link SimExplorer} owns seeds, shrinking and reporting.
 */
final class ExplorationRun {
    /** Robot cycle period in seconds. */
    static final double DT = 0.02;
    /** Maximum history lines kept per finding. */
    static final int HISTORY_LINES = 40;

    private ExplorationRun() {
    }

    /** One run's configuration. */
    record Config(
            Function<SimulationSession, RobotRuntime> robot,
            List<ModeSchedule.Segment> modes,
            List<InputEvent> inputs,
            int[] ports,
            double durationSeconds,
            double starvationSeconds,
            Path recordDirectory) {
    }

    /** One observed condition, before grouping. */
    record Observation(Category category, String subject, double timeSeconds, String message) {
    }

    /** All occurrences of one category and subject. */
    record Grouped(Observation first, int occurrences) {
    }

    /** Result of a run. */
    record Outcome(List<Grouped> grouped, long cycles, Path replay) {
    }

    static Outcome execute(Config config) {
        return new Engine(config).run();
    }

    /** Mutable state of one run. */
    private static final class Engine {
        private final Config config;
        private final Map<String, Grouped> groups = new LinkedHashMap<>();
        private final Map<String, Double> starvedSince = new HashMap<>();
        private final Set<String> reportedStarvation = new HashSet<>();
        private final InputState inputState;
        private double now;
        private RobotRuntime runtime;
        private RuntimeInputSnapshot current = RuntimeInputSnapshot.empty();

        Engine(Config config) {
            this.config = config;
            this.inputState = new InputState(config.ports());
        }

        Outcome run() {
            long cycles = 0;
            Path replay = null;
            RuntimeEvents.Subscription subscription = RuntimeEvents.listen(this::onRuntimeEvent);
            try {
                cycles = loop();
                if (runtime != null && runtime.replayRecorder().isPresent()) {
                    replay = flush(runtime.replayRecorder().orElseThrow());
                }
            } finally {
                subscription.close();
                if (runtime != null) {
                    runtime.close();
                }
            }
            return new Outcome(List.copyOf(groups.values()), cycles, replay);
        }

        private long loop() {
            SimulationSession session = SimulationSession.create();
            try {
                runtime = Objects.requireNonNull(config.robot().apply(session), "robot runtime");
            } catch (RuntimeException exception) {
                exception(exception);
                return 0;
            }
            runtime.failureReporter(new RuntimeFailureReporter() {
                @Override
                public void error(String message, Throwable cause) {
                    String subject = message.contains(" (") ? message.substring(0, message.indexOf(" (")) : message;
                    observe(Category.FAULT, subject, message);
                }

                @Override
                public void warning(String message, Throwable cause) {
                    // Warnings are rate limited on the wall clock, so they are not part of a deterministic run.
                }
            });
            // Starvation and control findings need per-cycle candidates and controls, which only CAPTURE builds.
            runtime.mechanismTraceLevel(MechanismTraceLevel.CAPTURE);
            runtime.mechanismTracePeriodSeconds(DT);
            runtime.inputs(() -> current);
            if (config.recordDirectory() != null) {
                runtime.replay(ReplayPolicy.of(), config.recordDirectory());
            }

            List<ModeSchedule.Segment> modes = config.modes();
            List<InputEvent> inputs = config.inputs();
            int steps = (int) Math.round(config.durationSeconds() / DT);
            int eventIndex = 0;
            int segmentIndex = 0;
            double segmentStart = 0.0;
            LifecycleMode previous = null;
            long cycles = 0;
            for (int cycle = 0; cycle < steps; cycle++) {
                now = cycle * DT;
                while (segmentIndex < modes.size() - 1
                        && now >= segmentStart + modes.get(segmentIndex).seconds() - 1e-9) {
                    segmentStart += modes.get(segmentIndex).seconds();
                    segmentIndex++;
                }
                LifecycleMode mode = modes.get(segmentIndex).mode();
                boolean enabled = mode != LifecycleMode.DISABLED;
                boolean autonomous = mode == LifecycleMode.AUTONOMOUS;
                LifecyclePhase phase = mode == previous ? LifecyclePhase.PERIODIC : LifecyclePhase.INIT;
                previous = mode;

                while (eventIndex < inputs.size() && inputs.get(eventIndex).timeSeconds() <= now + 1e-9) {
                    inputState.apply(inputs.get(eventIndex++));
                }
                current = inputState.snapshot(mode == LifecycleMode.TEST, enabled, autonomous,
                        now - segmentStart, mode);

                RuntimeException failure = null;
                RuntimeInputs.inject(current);
                try {
                    runtime.periodic(
                            new MechanismContext(now, 0.0, DT, enabled, autonomous, true),
                            new EventContext(now, DT, mode, phase, enabled, true));
                } catch (RuntimeException exception) {
                    failure = exception;
                } finally {
                    RuntimeInputs.release();
                }
                if (failure == null) {
                    try {
                        runtime.simulationPeriodic(now, DT);
                    } catch (RuntimeException exception) {
                        failure = exception;
                    }
                }
                cycles++;
                if (failure != null) {
                    exception(failure);
                    break;
                }
                inspect();
            }
            return cycles;
        }

        private void inspect() {
            Set<String> rejectedThisCycle = new HashSet<>();
            for (MechanismTraceSnapshot trace : runtime.mechanismTraces()) {
                String mechanism = trace.mechanism();
                checkStarvation(trace, rejectedThisCycle);
                for (MechanismTraceSnapshot.Control control : trace.controls()) {
                    String subject = mechanism + "/" + control.name();
                    if (control.blocked()) {
                        observe(Category.GUARD_BLOCK, subject, subject + " output held by a guard, requested "
                                + format(control.requestedValue()));
                    }
                    if (control.constrained()) {
                        observe(Category.CONSTRAINT_VIOLATION, subject, subject + " requested "
                                + format(control.requestedValue()) + " limited to " + format(control.appliedValue()));
                    }
                }
            }
            starvedSince.keySet().retainAll(rejectedThisCycle);
            reportedStarvation.retainAll(rejectedThisCycle);
            for (Staging.Change<?> change : runtime.drainStageChanges()) {
                observe(Category.STAGE_ENTRY, change.from() + "->" + change.to(),
                        "stage " + change.from() + " -> " + change.to());
            }
        }

        /**
         * A candidate is one lease's output request for a cycle. It is starved when the scheduler rejects it
         * because a newer lease reserves its resources, and it stays rejected without output for longer than
         * the limit. A request held back by a commitment has no candidate yet, so it is not counted here.
         */
        private void checkStarvation(MechanismTraceSnapshot trace, Set<String> rejectedThisCycle) {
            for (MechanismTraceSnapshot.ActionCandidate candidate : trace.candidates()) {
                if (candidate.selected() || !candidate.decision().contains("reserved by newer")) {
                    continue;
                }
                String key = trace.mechanism() + " " + candidate.actionType() + " " + candidate.motors()
                        + " " + candidate.outputMode();
                rejectedThisCycle.add(key);
                double since = starvedSince.computeIfAbsent(key, ignored -> now);
                if (now - since >= config.starvationSeconds() - 1e-9 && reportedStarvation.add(key)) {
                    observe(Category.STARVATION, key, "'" + candidate.actionType() + "' on " + candidate.motors()
                            + " in " + trace.mechanism() + " got no output for " + format(now - since)
                            + " s while a newer lease reserved it");
                }
            }
        }

        private void onRuntimeEvent(RuntimeEvents.Event event) {
            switch (event.category()) {
                case RuntimeEvents.TIMEOUT -> observe(Category.COMMITMENT_TIMEOUT, event.message(),
                        "commitment " + event.message() + " timed out");
                case RuntimeEvents.PATH_MARKERS_PENDING -> observe(Category.AUTO_MARKER_PENDING, event.message(),
                        "path finished with unfired markers: " + event.message());
                case RuntimeEvents.EFFORT_CEILING -> observe(Category.EFFORT_CEILING, event.message(),
                        "effort ceiling hit: " + event.message());
                default -> {
                }
            }
        }

        private void exception(RuntimeException exception) {
            observe(Category.EXCEPTION, exception.getClass().getName(),
                    "uncaught " + exception.getClass().getName() + ": " + exception.getMessage());
        }

        private void observe(Category category, String subject, String message) {
            String key = category + "|" + subject;
            Grouped existing = groups.get(key);
            if (existing != null) {
                groups.put(key, new Grouped(existing.first(), existing.occurrences() + 1));
                return;
            }
            groups.put(key, new Grouped(new Observation(category, subject, now, message), 1));
            if (runtime != null) {
                runtime.replayRecorder().ifPresent(recorder ->
                        recorder.mark(markerKind(category), category + ": " + message));
            }
        }

        private static String format(double value) {
            return String.format(Locale.ROOT, "%.3f", value);
        }

        private static int markerKind(Category category) {
            return switch (category) {
                case STARVATION -> ReplayMarkers.USER_BASE + 1;
                case COMMITMENT_TIMEOUT -> ReplayMarkers.USER_BASE + 2;
                case AUTO_MARKER_PENDING -> ReplayMarkers.USER_BASE + 3;
                case EFFORT_CEILING -> ReplayMarkers.USER_BASE + 4;
                case STAGE_ENTRY -> ReplayMarkers.STAGE_CHANGE;
                case CONSTRAINT_VIOLATION, GUARD_BLOCK -> ReplayMarkers.CHECK;
                case FAULT, EXCEPTION -> ReplayMarkers.FAULT;
            };
        }

        private static Path flush(ReplayRecorder recorder) {
            recorder.flush();
            try {
                recorder.awaitFlushed(Duration.ofSeconds(60));
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
            }
            return recorder.lastFile().orElse(null);
        }
    }

    /** Controller state built by applying input events in time order. */
    private static final class InputState {
        private final double[][] axes = new double[RuntimeInputSnapshot.JOYSTICK_PORTS][RuntimeInputSnapshot.AXES];
        private final int[][] povs = new int[RuntimeInputSnapshot.JOYSTICK_PORTS][RuntimeInputSnapshot.POVS];
        private final long[] buttons = new long[RuntimeInputSnapshot.JOYSTICK_PORTS];
        private final boolean[] connected = new boolean[RuntimeInputSnapshot.JOYSTICK_PORTS];

        InputState(int[] ports) {
            for (int[] pov : povs) {
                java.util.Arrays.fill(pov, -1);
            }
            for (int port : ports) {
                connected[port] = true;
            }
        }

        void apply(InputEvent event) {
            int port = event.port();
            switch (event.kind()) {
                case BUTTON -> {
                    long bit = 1L << (event.index() - 1);
                    buttons[port] = event.value() != 0.0 ? buttons[port] | bit : buttons[port] & ~bit;
                }
                case AXIS -> axes[port][event.index()] = event.value();
                case POV -> povs[port][event.index()] = (int) event.value();
            }
        }

        RuntimeInputSnapshot snapshot(boolean test, boolean enabled, boolean autonomous, double matchTime,
                LifecycleMode mode) {
            List<RuntimeInputSnapshot.Joystick> joysticks = new ArrayList<>();
            for (int port = 0; port < connected.length; port++) {
                if (connected[port]) {
                    joysticks.add(new RuntimeInputSnapshot.Joystick(port, true, "Explorer" + port, 1,
                            axes[port], buttons[port], povs[port]));
                }
            }
            RuntimeInputSnapshot.DriverStationState state = new RuntimeInputSnapshot.DriverStationState(
                    enabled, autonomous, test, false, true, "red", matchTime, "EXPLORE", "NONE", 1, mode.name());
            return new RuntimeInputSnapshot(joysticks, state);
        }
    }
}
