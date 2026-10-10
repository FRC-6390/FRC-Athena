package ca.frc6390.athena.robot.replay;

import java.nio.file.Path;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

/**
 * Typed robot-wide replay policy. Declare one field of this type on the robot class; {@code AthenaRobot}
 * discovers it and applies it. Without a field the defaults apply: the RAM buffer is enabled, streaming is
 * disabled, and files are written on match end, disable after a match, and brownout recovery.
 *
 * <p>Instances are immutable; every setter returns a new policy.</p>
 */
public final class ReplayPolicy {
    /** Events that cause the buffer to be written to a file. */
    public enum Flush {
        /** Teleop ended (the end of a match period). Practice and FMS matches both flush. */
        MATCH_END,
        /** Robot disabled after an autonomous or teleop period that has not been flushed yet. */
        DISABLE_AFTER_MATCH,
        /** Battery voltage recovered from a brownout. */
        BROWNOUT_RECOVERY
    }

    public static final long DEFAULT_BUFFER_BYTES = 48L * 1024 * 1024;

    private final boolean stream;
    private final long bufferBytes;
    private final Set<Flush> flushes;
    private final Retention retention;
    private final Path directory;

    private ReplayPolicy(boolean stream, long bufferBytes, Set<Flush> flushes, Retention retention, Path directory) {
        this.stream = stream;
        this.bufferBytes = bufferBytes;
        this.flushes = Set.copyOf(flushes);
        this.retention = Objects.requireNonNull(retention, "retention");
        this.directory = directory;
    }

    /**
     * Returns the default policy.
     *
     * @return default policy
     */
    public static ReplayPolicy of() {
        return new ReplayPolicy(
                false,
                DEFAULT_BUFFER_BYTES,
                EnumSet.of(Flush.MATCH_END, Flush.DISABLE_AFTER_MATCH, Flush.BROWNOUT_RECOVERY),
                Retention.defaults(),
                null);
    }

    /**
     * Enables or disables streaming. Streaming also needs a {@link ReplayStreamSink}; without one nothing is streamed.
     *
     * @param enabled whether frames are published to the stream sink
     * @return new policy
     */
    public ReplayPolicy stream(boolean enabled) {
        return new ReplayPolicy(enabled, bufferBytes, flushes, retention, directory);
    }

    /**
     * Sets the in-RAM buffer size.
     *
     * @param bytes buffer size in bytes; 0 keeps only the newest segment, which is enough for a flush of
     *     the most recent cycles
     * @return new policy
     */
    public ReplayPolicy buffer(long bytes) {
        if (bytes < 0) throw new IllegalArgumentException("Buffer size must not be negative.");
        return new ReplayPolicy(stream, bytes, flushes, retention, directory);
    }

    /**
     * Sets the in-RAM buffer size in megabytes (2^20 bytes).
     *
     * @param megabytes buffer size
     * @return new policy
     */
    public ReplayPolicy bufferMegabytes(int megabytes) {
        return buffer((long) megabytes * 1024 * 1024);
    }

    /**
     * Replaces the flush events.
     *
     * @param events flush events; none disables automatic flushing (explicit flushes still work)
     * @return new policy
     */
    public ReplayPolicy flushOn(Flush... events) {
        Set<Flush> selected = events == null || events.length == 0
                ? EnumSet.noneOf(Flush.class)
                : EnumSet.copyOf(Arrays.asList(events));
        return new ReplayPolicy(stream, bufferBytes, selected, retention, directory);
    }

    /**
     * Sets file retention.
     *
     * @param retention retention rules
     * @return new policy
     */
    public ReplayPolicy retention(Retention retention) {
        return new ReplayPolicy(stream, bufferBytes, flushes, Objects.requireNonNull(retention, "retention"), directory);
    }

    /**
     * Overrides the replay directory. When unset, {@code AthenaRobot} uses {@code /home/lvuser/athena/replays}
     * on the roboRIO and a temporary directory in simulation.
     *
     * @param directory replay directory
     * @return new policy
     */
    public ReplayPolicy directory(Path directory) {
        return new ReplayPolicy(stream, bufferBytes, flushes, retention, Objects.requireNonNull(directory, "directory"));
    }

    public boolean streamEnabled() {
        return stream;
    }

    public long bufferBytes() {
        return bufferBytes;
    }

    public boolean flushes(Flush event) {
        return flushes.contains(event);
    }

    public Retention retention() {
        return retention;
    }

    /** Returns the configured directory, if one was set. */
    public Optional<Path> directory() {
        return Optional.ofNullable(directory);
    }
}
