package ca.frc6390.athena.robot.explore;

import edu.wpi.first.units.measure.Time;
import static edu.wpi.first.units.Units.Seconds;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Random;

/**
 * Driver station mode sequence for an exploration run.
 *
 * <p>Two shapes exist. {@link #matchPlay()} and {@link #of(Segment...)} are fixed. {@link #random()} draws a
 * sequence of legal transitions from the run seed: disabled is followed by autonomous, teleop or test, and
 * every enabled mode is followed by disabled (FMS never switches between enabled modes directly). Dwell times
 * come from {@link #dwell}.</p>
 */
public final class ModeSchedule {
    /** One mode and how long it lasts. */
    public record Segment(LifecycleMode mode, Time duration) {
        public Segment {
            Objects.requireNonNull(mode, "mode");
            double seconds = Objects.requireNonNull(duration, "duration").in(Seconds);
            if (!(seconds > 0.0) || !Double.isFinite(seconds)) {
                throw new IllegalArgumentException("Segment duration must be positive and finite.");
            }
        }
    }

    private static final LifecycleMode[] ENABLED = {
            LifecycleMode.AUTONOMOUS, LifecycleMode.TELEOP, LifecycleMode.TEST};

    private final List<Segment> fixed;
    private final Map<LifecycleMode, double[]> dwell;

    private ModeSchedule(List<Segment> fixed, Map<LifecycleMode, double[]> dwell) {
        this.fixed = fixed;
        this.dwell = dwell;
    }

    /** Disabled 1 s, autonomous 15 s, disabled 3 s, teleop 135 s; then disabled. */
    public static ModeSchedule matchPlay() {
        return new ModeSchedule(List.of(
                new Segment(LifecycleMode.DISABLED, Seconds.of(1.0)),
                new Segment(LifecycleMode.AUTONOMOUS, Seconds.of(15.0)),
                new Segment(LifecycleMode.DISABLED, Seconds.of(3.0)),
                new Segment(LifecycleMode.TELEOP, Seconds.of(135.0))), null);
    }

    /** A fixed sequence of segments; the robot stays disabled after the last one. */
    public static ModeSchedule of(Segment... segments) {
        return new ModeSchedule(List.of(segments), null);
    }

    /** Random legal transitions with the default dwell times. */
    public static ModeSchedule random() {
        Map<LifecycleMode, double[]> defaults = new EnumMap<>(LifecycleMode.class);
        defaults.put(LifecycleMode.DISABLED, new double[] {1.0, 4.0});
        defaults.put(LifecycleMode.AUTONOMOUS, new double[] {3.0, 15.0});
        defaults.put(LifecycleMode.TELEOP, new double[] {10.0, 60.0});
        defaults.put(LifecycleMode.TEST, new double[] {3.0, 10.0});
        return new ModeSchedule(null, defaults);
    }

    /**
     * Overrides the dwell range of one mode on a {@link #random()} schedule.
     *
     * @return a new schedule
     */
    public ModeSchedule dwell(LifecycleMode mode, Time min, Time max) {
        if (dwell == null) {
            throw new IllegalStateException("dwell applies only to random schedules");
        }
        double minSeconds = Objects.requireNonNull(min, "min").in(Seconds);
        double maxSeconds = Objects.requireNonNull(max, "max").in(Seconds);
        if (!(minSeconds > 0.0) || maxSeconds < minSeconds) {
            throw new IllegalArgumentException("Dwell range must be positive and ordered.");
        }
        Map<LifecycleMode, double[]> updated = new EnumMap<>(dwell);
        updated.put(mode, new double[] {minSeconds, maxSeconds});
        return new ModeSchedule(null, updated);
    }

    /**
     * Returns the concrete segments for a run. Segments that start after {@code durationSeconds} are dropped; a
     * shorter sequence is padded with disabled.
     */
    List<Segment> materialize(long seed, double durationSeconds) {
        List<Segment> out = new ArrayList<>();
        double elapsed = 0.0;
        if (fixed != null) {
            for (Segment segment : fixed) {
                if (elapsed >= durationSeconds) break;
                out.add(segment);
                elapsed += segment.duration().in(Seconds);
            }
        } else {
            Random random = new Random(seed * 0x9E3779B97F4A7C15L + 0x5DEECE66DL);
            LifecycleMode mode = LifecycleMode.DISABLED;
            while (elapsed < durationSeconds) {
                double[] range = dwell.get(mode);
                double seconds = range[0] + (range[1] - range[0]) * random.nextDouble();
                out.add(new Segment(mode, Seconds.of(seconds)));
                elapsed += seconds;
                mode = mode == LifecycleMode.DISABLED
                        ? ENABLED[random.nextInt(ENABLED.length)]
                        : LifecycleMode.DISABLED;
            }
        }
        if (elapsed < durationSeconds) {
            out.add(new Segment(LifecycleMode.DISABLED, Seconds.of(durationSeconds - elapsed)));
        }
        return List.copyOf(out);
    }
}
