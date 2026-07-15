package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import java.util.Optional;

/**
 * External provider for path-backed autonomous Actions and mechanism path runtimes.
 */
public interface PathProvider {
    /**
     * Creates a path Action for the provider.
     *
     * @param pathName provider-specific path name
     * @return path Action
     */
    PathAction path(String pathName);

    /**
     * Creates the mechanism path runtime that executes provider path Actions.
     *
     * @return path runtime
     */
    PathRuntime runtime();

    /** Returns display geometry when this provider owns the supplied path. */
    default Optional<PathPreview> preview(PathAction path) { return Optional.empty(); }

    /** Changes when dynamic preview inputs such as alliance mirroring change. */
    default long previewRevision() { return 0L; }
}
