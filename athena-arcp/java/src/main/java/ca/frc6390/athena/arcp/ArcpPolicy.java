package ca.frc6390.athena.arcp;

import java.nio.file.Path;
import java.time.Duration;
import java.util.Objects;
import java.util.Optional;

/**
 * Typed robot-wide ARCP policy. Declare one field of this type on the robot class; the robot runtime
 * discovers it. Without a field the defaults apply: ARCP is enabled on the real robot and in simulation,
 * with TCP control port {@value #DEFAULT_CONTROL_PORT} and realtime UDP port {@value #DEFAULT_REALTIME_PORT} (both
 * inside the FRC team-use range 5800-5810; NT4 owns 5810), no bandwidth budget, 1200-byte datagrams and 1 second keyframes.
 *
 * <pre>{@code
 * public final ArcpPolicy arcp = ArcpPolicy.of()
 *         .controlPort(5808)
 *         .realtimePort(5809)
 *         .bandwidthBudgetBytesPerSecond(400_000);
 * }</pre>
 *
 * <p>Instances are immutable; every setter returns a new policy. Replay files follow the replay policy's
 * directory; the persistent store follows its own directory; the directory overrides here only select the
 * roots the file service exposes.</p>
 */
public final class ArcpPolicy {

    public static final int DEFAULT_MTU = 1200;
    public static final Duration DEFAULT_KEYFRAME_INTERVAL = Duration.ofSeconds(1);

    private final boolean enabled;
    /** Default TCP control port; inside the FRC team-use range 5800-5810 and the dashboard's default. */
    public static final int DEFAULT_CONTROL_PORT = 5808;
    /** Default UDP port the realtime frames are sent from; inside the FRC team-use range. */
    public static final int DEFAULT_REALTIME_PORT = 5809;

    private final int controlPort;
    private final int realtimePort;
    private final long budgetBytesPerSecond;
    private final int mtu;
    private final Duration keyframeInterval;
    private final Path storeDirectory;
    private final Path modelDirectory;
    private final Path deployedTuningFile;

    private ArcpPolicy(boolean enabled, int controlPort, int realtimePort, long budgetBytesPerSecond, int mtu,
            Duration keyframeInterval, Path storeDirectory, Path modelDirectory, Path deployedTuningFile) {
        this.enabled = enabled;
        this.controlPort = controlPort;
        this.realtimePort = realtimePort;
        this.budgetBytesPerSecond = budgetBytesPerSecond;
        this.mtu = mtu;
        this.keyframeInterval = keyframeInterval;
        this.storeDirectory = storeDirectory;
        this.modelDirectory = modelDirectory;
        this.deployedTuningFile = deployedTuningFile;
    }

    /** The default policy: ARCP enabled with the defaults described on the class. */
    public static ArcpPolicy of() {
        return new ArcpPolicy(true, DEFAULT_CONTROL_PORT, DEFAULT_REALTIME_PORT, 0L, DEFAULT_MTU, DEFAULT_KEYFRAME_INTERVAL,
                null, null, null);
    }

    /** A policy that turns ARCP off. The robot then publishes nothing and serves no files. */
    public static ArcpPolicy disabled() {
        return new ArcpPolicy(false, 0, 0, 0L, DEFAULT_MTU, DEFAULT_KEYFRAME_INTERVAL, null, null, null);
    }

    /**
     * Sets the TCP control port ({@value #DEFAULT_CONTROL_PORT} by default, which the dashboard connects to; 0 picks a
     * free port, useful in tests).
     *
     * @param port control port, 0 to 65535
     * @return new policy
     */
    public ArcpPolicy controlPort(int port) {
        if (port < 0 || port > 65_535) {
            throw new IllegalArgumentException("control port out of range: " + port);
        }
        return new ArcpPolicy(enabled, port, realtimePort, budgetBytesPerSecond, mtu, keyframeInterval, storeDirectory,
                modelDirectory, deployedTuningFile);
    }

    /**
     * Sets the UDP port the realtime frames are sent from ({@value #DEFAULT_REALTIME_PORT} by default; 0 picks a free
     * port, useful in tests). On a real robot a port that is already in use fails ARCP loudly; in simulation ARCP falls
     * back to ephemeral ports and reports it.
     *
     * @param port realtime UDP port, 0 to 65535
     * @return new policy
     */
    public ArcpPolicy realtimePort(int port) {
        if (port < 0 || port > 65_535) {
            throw new IllegalArgumentException("realtime port out of range: " + port);
        }
        return new ArcpPolicy(enabled, controlPort, port, budgetBytesPerSecond, mtu, keyframeInterval, storeDirectory,
                modelDirectory, deployedTuningFile);
    }

    /**
     * Sets the realtime bandwidth budget. When traffic exceeds it, the newest subscription's rate is lowered one
     * step per second until the budget is met.
     *
     * @param bytesPerSecond budget, 0 for none
     * @return new policy
     */
    public ArcpPolicy bandwidthBudgetBytesPerSecond(long bytesPerSecond) {
        if (bytesPerSecond < 0) {
            throw new IllegalArgumentException("bandwidth budget must be >= 0");
        }
        return new ArcpPolicy(enabled, controlPort, realtimePort, bytesPerSecond, mtu, keyframeInterval, storeDirectory,
                modelDirectory, deployedTuningFile);
    }

    /**
     * Sets the largest realtime datagram.
     *
     * @param bytes 128 to 65507
     * @return new policy
     */
    public ArcpPolicy mtu(int bytes) {
        if (bytes < 128 || bytes > 65_507) {
            throw new IllegalArgumentException("mtu out of range: " + bytes);
        }
        return new ArcpPolicy(enabled, controlPort, realtimePort, budgetBytesPerSecond, bytes, keyframeInterval,
                storeDirectory, modelDirectory, deployedTuningFile);
    }

    /**
     * Sets the keyframe cadence per subscriber.
     *
     * @param interval positive interval
     * @return new policy
     */
    public ArcpPolicy keyframeInterval(Duration interval) {
        Objects.requireNonNull(interval, "interval");
        if (interval.isNegative() || interval.isZero()) {
            throw new IllegalArgumentException("keyframe interval must be positive");
        }
        return new ArcpPolicy(enabled, controlPort, realtimePort, budgetBytesPerSecond, mtu, interval, storeDirectory,
                modelDirectory, deployedTuningFile);
    }

    /** Overrides the persistent store directory, which the {@code store/} file category exposes. */
    public ArcpPolicy storeDirectory(Path directory) {
        return new ArcpPolicy(enabled, controlPort, realtimePort, budgetBytesPerSecond, mtu, keyframeInterval,
                Objects.requireNonNull(directory, "directory"), modelDirectory, deployedTuningFile);
    }

    /** Sets the characterization and model directory, which the {@code models/} file category exposes. */
    public ArcpPolicy modelDirectory(Path directory) {
        return new ArcpPolicy(enabled, controlPort, realtimePort, budgetBytesPerSecond, mtu, keyframeInterval,
                storeDirectory, Objects.requireNonNull(directory, "directory"), deployedTuningFile);
    }

    /**
     * Overrides the deployed tuning file (the middle layer of robot store, deployed file, code default). The
     * default is {@code /home/lvuser/deploy/athena/tuning} on the roboRIO and
     * {@code src/main/deploy/athena/tuning} in simulation.
     */
    public ArcpPolicy deployedTuningFile(Path file) {
        return new ArcpPolicy(enabled, controlPort, realtimePort, budgetBytesPerSecond, mtu, keyframeInterval,
                storeDirectory, modelDirectory, Objects.requireNonNull(file, "file"));
    }

    public boolean enabled() {
        return enabled;
    }

    public int controlPort() {
        return controlPort;
    }

    public int realtimePort() {
        return realtimePort;
    }

    public long bandwidthBudgetBytesPerSecond() {
        return budgetBytesPerSecond;
    }

    public int mtu() {
        return mtu;
    }

    public Duration keyframeInterval() {
        return keyframeInterval;
    }

    public Optional<Path> storeDirectory() {
        return Optional.ofNullable(storeDirectory);
    }

    public Optional<Path> modelDirectory() {
        return Optional.ofNullable(modelDirectory);
    }

    public Optional<Path> deployedTuningFile() {
        return Optional.ofNullable(deployedTuningFile);
    }
}
