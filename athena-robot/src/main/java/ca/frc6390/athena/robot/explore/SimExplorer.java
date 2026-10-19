package ca.frc6390.athena.robot.explore;

import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Seconds;
import ca.frc6390.athena.robot.RobotRuntime;
import ca.frc6390.athena.robot.explore.ExplorationReport.Category;
import ca.frc6390.athena.robot.explore.ExplorationReport.Finding;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.function.Function;

/**
 * Drives a robot with random but legal driver station modes and controller input for a fixed seed, and reports
 * what went wrong.
 *
 * <pre>{@code
 * ExplorationReport report = SimExplorer.of(session -> RobotRuntime.simulated(session).register(new Robot()))
 *         .seed(7)
 *         .duration(150)
 *         .controllers(0)
 *         .modes(ModeSchedule.random())
 *         .inputs(InputModel.standard())
 *         .save(Path.of("build/explore"))
 *         .run();
 * }</pre>
 *
 * <p>The seed fixes the mode sequence and the input script. Robot code and the simulation are deterministic, so
 * the same seed and options give the same report. For each category with a problem, the first finding is shrunk:
 * the input script is cut at the finding and delta debugging removes events while the category still
 * reproduces. The minimal script is then run again with replay recording, the finding markers are planted in
 * the recording, and the file is saved when {@link #save(Path)} is set. The file can be replayed with
 * {@code ReplaySimulation}, or opened in the dashboard.</p>
 *
 * <p>Thread safety: a run owns its simulation and robot. Runs are sequential; {@code RuntimeEvents} listens
 * process-wide while a run executes.</p>
 */
public final class SimExplorer {
    private static final double SHRINK_MARGIN_SECONDS = 0.1;

    private final Function<SimulationSession, RobotRuntime> robot;
    private long seed = 1L;
    private double duration = 60.0;
    private int[] ports = {0};
    private ModeSchedule modes = ModeSchedule.random();
    private InputModel inputs = InputModel.standard();
    private double starvationSeconds = 2.0;
    private boolean shrink = true;
    private Path saveDirectory;

    private SimExplorer(Function<SimulationSession, RobotRuntime> robot) {
        this.robot = Objects.requireNonNull(robot, "robotFactory");
    }

    /**
     * Starts an exploration of a robot. The factory must create the runtime on the session it is given, as
     * {@code ReplaySimulation} does: {@code session -> RobotRuntime.simulated(session).register(...)}.
     *
     * @param robotFactory builds a fresh simulated runtime for every run
     * @return explorer with default options
     */
    public static SimExplorer of(Function<SimulationSession, RobotRuntime> robotFactory) {
        return new SimExplorer(robotFactory);
    }

    /** Sets the seed that fixes the mode sequence and the input script. Default 1. */
    public SimExplorer seed(long seed) {
        this.seed = seed;
        return this;
    }

    /** Sets the run length in seconds. Default 60. */
    public SimExplorer duration(Time length) {
        double seconds = Objects.requireNonNull(length, "length").in(Seconds);
        if (!(seconds > 0.0) || !Double.isFinite(seconds)) {
            throw new IllegalArgumentException("duration must be positive and finite");
        }
        this.duration = seconds;
        return this;
    }

    /** Sets the joystick ports that receive random input (0 to 5). Default port 0. */
    public SimExplorer controllers(int... ports) {
        if (ports == null || ports.length == 0) {
            throw new IllegalArgumentException("at least one controller port is required");
        }
        for (int index = 0; index < ports.length; index++) {
            if (ports[index] < 0 || ports[index] > 5) {
                throw new IllegalArgumentException("controller port must be 0..5: " + ports[index]);
            }
            for (int other = 0; other < index; other++) {
                if (ports[other] == ports[index]) {
                    throw new IllegalArgumentException("duplicate controller port " + ports[index]);
                }
            }
        }
        this.ports = ports.clone();
        return this;
    }

    /** Sets the mode sequence. Default {@link ModeSchedule#random()}. */
    public SimExplorer modes(ModeSchedule schedule) {
        this.modes = Objects.requireNonNull(schedule, "schedule");
        return this;
    }

    /** Sets the input rates. Default {@link InputModel#standard()}. */
    public SimExplorer inputs(InputModel model) {
        this.inputs = Objects.requireNonNull(model, "model");
        return this;
    }

    /**
     * Sets how long a requested Action may get no output while a newer lease holds its resources before it counts
     * as starved. Default 2 s.
     */
    public SimExplorer starvation(Time limit) {
        double seconds = Objects.requireNonNull(limit, "limit").in(Seconds);
        if (!(seconds > 0.0) || !Double.isFinite(seconds)) {
            throw new IllegalArgumentException("starvation must be positive and finite");
        }
        this.starvationSeconds = seconds;
        return this;
    }

    /** Enables or disables shrinking of findings. Default enabled. */
    public SimExplorer shrink(boolean enabled) {
        this.shrink = enabled;
        return this;
    }

    /**
     * Saves the minimized replay of each shrunk finding into a directory as
     * {@code explore-<category>-seed<seed>.athrp}. Without this, no replay files are written.
     */
    public SimExplorer save(Path directory) {
        this.saveDirectory = Objects.requireNonNull(directory, "directory");
        return this;
    }

    /**
     * Runs the exploration.
     *
     * @return the report
     */
    public ExplorationReport run() {
        List<ModeSchedule.Segment> segments = modes.materialize(seed, duration);
        List<InputEvent> script = inputs.generate(seed, ports, duration);
        ExplorationRun.Outcome base = ExplorationRun.execute(config(segments, script, duration, null));

        List<Finding> findings = new ArrayList<>();
        List<Category> shrunkCategories = new ArrayList<>();
        for (ExplorationRun.Grouped group : base.grouped()) {
            ExplorationRun.Observation first = group.first();
            Category category = first.category();
            boolean shrinkThis = shrink && !category.informational() && !shrunkCategories.contains(category);
            if (shrinkThis) {
                shrunkCategories.add(category);
                findings.add(shrinkFinding(group, segments, script));
            } else {
                findings.add(new Finding(category, first.subject(), first.timeSeconds(), group.occurrences(),
                        first.message(), history(segments, script, first.timeSeconds()), false, List.of(),
                        0.0, null));
            }
        }
        return new ExplorationReport(seed, duration, base.cycles(), findings);
    }

    private Finding shrinkFinding(
            ExplorationRun.Grouped group,
            List<ModeSchedule.Segment> segments,
            List<InputEvent> script) {
        ExplorationRun.Observation first = group.first();
        Category category = first.category();
        double shortDuration = Math.min(duration, first.timeSeconds() + SHRINK_MARGIN_SECONDS);
        List<InputEvent> cut = new ArrayList<>();
        for (InputEvent event : script) {
            if (event.timeSeconds() < shortDuration) {
                cut.add(event);
            }
        }
        List<ModeSchedule.Segment> cutSegments = truncate(segments, shortDuration);
        List<InputEvent> minimal = Shrinker.minimize(cut, candidate -> reproduces(cutSegments, candidate,
                shortDuration, category));
        Path replay = null;
        if (saveDirectory != null) {
            replay = save(cutSegments, minimal, shortDuration, category);
        }
        return new Finding(category, first.subject(), first.timeSeconds(), group.occurrences(), first.message(),
                history(segments, script, first.timeSeconds()), true, minimal, shortDuration, replay);
    }

    private boolean reproduces(
            List<ModeSchedule.Segment> segments,
            List<InputEvent> candidate,
            double shortDuration,
            Category category) {
        ExplorationRun.Outcome outcome = ExplorationRun.execute(config(segments, candidate, shortDuration, null));
        return outcome.grouped().stream().anyMatch(group -> group.first().category() == category);
    }

    private Path save(List<ModeSchedule.Segment> segments, List<InputEvent> minimal, double shortDuration,
            Category category) {
        try {
            Files.createDirectories(saveDirectory);
            ExplorationRun.Outcome recorded = ExplorationRun.execute(
                    config(segments, minimal, shortDuration, saveDirectory));
            if (recorded.replay() == null) {
                return null;
            }
            Path target = saveDirectory.resolve("explore-" + category.name().toLowerCase(Locale.ROOT)
                    + "-seed" + seed + ".athrp");
            Files.move(recorded.replay(), target, StandardCopyOption.REPLACE_EXISTING);
            return target;
        } catch (IOException exception) {
            throw new UncheckedIOException("Cannot save exploration replay in " + saveDirectory + ".", exception);
        }
    }

    private ExplorationRun.Config config(
            List<ModeSchedule.Segment> segments,
            List<InputEvent> script,
            double seconds,
            Path recordDirectory) {
        return new ExplorationRun.Config(robot, segments, script, ports, seconds, starvationSeconds,
                recordDirectory);
    }

    private static List<ModeSchedule.Segment> truncate(List<ModeSchedule.Segment> segments, double seconds) {
        List<ModeSchedule.Segment> out = new ArrayList<>();
        double elapsed = 0.0;
        for (ModeSchedule.Segment segment : segments) {
            if (elapsed >= seconds) {
                break;
            }
            out.add(segment);
            elapsed += segment.duration().in(Seconds);
        }
        return out;
    }

    /** The last input events and mode changes up to a time, oldest first. */
    static List<String> history(List<ModeSchedule.Segment> segments, List<InputEvent> script, double until) {
        List<Entry> entries = new ArrayList<>();
        double start = 0.0;
        for (ModeSchedule.Segment segment : segments) {
            if (start > until) {
                break;
            }
            entries.add(new Entry(start, 0, String.format(Locale.ROOT, "t=%.3f mode %s for %.1f s",
                    start, segment.mode(), segment.duration().in(Seconds))));
            start += segment.duration().in(Seconds);
        }
        for (InputEvent event : script) {
            if (event.timeSeconds() > until) {
                break;
            }
            entries.add(new Entry(event.timeSeconds(), 1, event.describe()));
        }
        entries.sort(Comparator.comparingDouble(Entry::time).thenComparingInt(Entry::order));
        int from = Math.max(0, entries.size() - ExplorationRun.HISTORY_LINES);
        List<String> lines = new ArrayList<>();
        for (Entry entry : entries.subList(from, entries.size())) {
            lines.add(entry.text());
        }
        return lines;
    }

    private record Entry(double time, int order, String text) {
    }
}
