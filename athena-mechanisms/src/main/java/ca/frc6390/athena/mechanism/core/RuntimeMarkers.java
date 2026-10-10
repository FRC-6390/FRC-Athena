package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.runtime.wire.Wire;

/**
 * Service-provider interface through which mechanism code plants replay markers and requests full
 * capture. The default implementation does nothing. The replay stream installs the real implementation
 * with {@link #install(RuntimeMarkers)}.
 */
public interface RuntimeMarkers {
    /** Marker kind for persisted tuning writes ({@link Wire#MARKER_PERSISTED_WRITE}). */
    int PERSISTED_WRITE = Wire.MARKER_PERSISTED_WRITE;
    /** Marker kind for interpolation records ({@link Wire#MARKER_RECORD}). */
    int RECORD = Wire.MARKER_RECORD;

    /** The no-op implementation used when nothing is installed. */
    RuntimeMarkers NONE = new RuntimeMarkers() {
        @Override
        public void mark(int kind, String message) {
        }

        @Override
        public void requestFullCapture() {
        }
    };

    /**
     * Plants a marker.
     *
     * @param kind marker kind, such as {@link #RECORD}
     * @param message human-readable details
     */
    void mark(int kind, String message);

    /** Asks the recorder to capture every signal for the current cycle regardless of rate limits. */
    void requestFullCapture();

    /**
     * Returns the metadata attached to a recorded interpolation point. The default has no match or
     * replay identity and uses {@link #clockSeconds()} for the timestamp.
     *
     * @return record metadata
     */
    default RecordMetadata recordMetadata() {
        return RecordMetadata.at(clockSeconds());
    }

    /**
     * Returns the timestamp used for markers and records. The default uses the JVM monotonic clock.
     *
     * @return seconds
     */
    default double clockSeconds() {
        return System.nanoTime() * 1.0e-9;
    }

    /** Returns the installed implementation. */
    static RuntimeMarkers current() {
        return Installed.markers;
    }

    /**
     * Installs the implementation used by all mechanisms. Passing null restores the no-op default.
     *
     * @param markers implementation, or null
     */
    static void install(RuntimeMarkers markers) {
        Installed.markers = markers == null ? NONE : markers;
    }

    /** Holder so the installed reference is a single volatile read. */
    final class Installed {
        private static volatile RuntimeMarkers markers = NONE;

        private Installed() {
        }
    }
}
