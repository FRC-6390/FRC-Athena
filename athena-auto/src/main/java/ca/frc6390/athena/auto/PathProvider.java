package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import java.util.Optional;

/**
 * External provider for path-backed autonomous Actions and mechanism path runtimes.
 */
public interface PathProvider extends AutoCloseable {
    /** Source key owned by this provider, such as {@code choreo}. */
    String source();

    /** Returns whether this provider owns the supplied path Action. */
    default boolean owns(PathAction path) {
        return path != null && source().equals(path.source());
    }

    /**
     * Creates the mechanism path runtime that executes provider path Actions.
     *
     * @return path runtime
     */
    PathRuntime runtime();

    /** Queues any loading/preview preparation for a registered path without waiting on the caller. */
    default void prepare(PathAction path) { }

    /**
     * Startup-only preparation, after robot-init configuration and before the first periodic cycle. Providers
     * that construct WPILib command compositions do so on the setup thread. AthenaRobot invokes this through
     * RobotRuntime.preparePathsForStartup after init hooks; headless runtimes call that fence explicitly.
     */
    default void prepareForStartup(PathAction path) { prepare(path); }

    /** Releases provider workers during explicit runtime shutdown. */
    @Override default void close() { }

    /** Returns display geometry when this provider owns the supplied path. */
    default Optional<PathPreview> preview(PathAction path) { return Optional.empty(); }

    /** Changes when dynamic preview inputs such as alliance mirroring change. */
    default long previewRevision() { return 0L; }
}
