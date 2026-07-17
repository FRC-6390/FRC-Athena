package ca.frc6390.athena.wpilib.system;

/**
 * Selects Athena's roboRIO memory monitoring and operating-system tuning policy.
 * Automatic tuning is the default used by {@code AthenaRobot}.
 */
public final class SystemTuning {
    /** Available tuning policies. */
    public enum Profile {
        /** Tune positively identified low-memory roboRIOs and monitor everything else. */
        AUTOMATIC,
        /** Publish system health without changing the operating system. */
        STANDARD,
        /** Request low-memory tuning on any real Linux robot target. */
        LOW_MEMORY,
        /** Disable both tuning and periodic system telemetry. */
        DISABLED
    }

    private static final long MIB = 1024L * 1024L;

    private final Profile profile;
    private final long swapBytes;
    private final double warningAvailableFraction;
    private final double criticalAvailableFraction;
    private final boolean stopNiWebServer;

    private SystemTuning(
            Profile profile,
            long swapBytes,
            double warningAvailableFraction,
            double criticalAvailableFraction,
            boolean stopNiWebServer) {
        this.profile = profile;
        this.swapBytes = swapBytes;
        this.warningAvailableFraction = warningAvailableFraction;
        this.criticalAvailableFraction = criticalAvailableFraction;
        this.stopNiWebServer = stopNiWebServer;
    }

    public static SystemTuning automatic() {
        return defaults(Profile.AUTOMATIC);
    }

    public static SystemTuning standard() {
        return defaults(Profile.STANDARD);
    }

    public static SystemTuning lowMemory() {
        return defaults(Profile.LOW_MEMORY);
    }

    public static SystemTuning disabled() {
        return defaults(Profile.DISABLED);
    }

    private static SystemTuning defaults(Profile profile) {
        return new SystemTuning(profile, 32L * MIB, 0.12, 0.06, true);
    }

    /** Returns a copy with a bounded fallback swap-file size. */
    public SystemTuning swapMegabytes(int megabytes) {
        if (megabytes < 16 || megabytes > 128) {
            throw new IllegalArgumentException("Swap size must be between 16 and 128 MiB.");
        }
        return new SystemTuning(
                profile, megabytes * MIB, warningAvailableFraction, criticalAvailableFraction, stopNiWebServer);
    }

    /** Returns a copy with memory-pressure thresholds expressed as available-RAM fractions. */
    public SystemTuning pressureThresholds(double warningFraction, double criticalFraction) {
        if (!Double.isFinite(warningFraction) || !Double.isFinite(criticalFraction)
                || criticalFraction <= 0.0 || warningFraction <= criticalFraction || warningFraction >= 1.0) {
            throw new IllegalArgumentException(
                    "Pressure thresholds require 0 < critical < warning < 1.");
        }
        return new SystemTuning(profile, swapBytes, warningFraction, criticalFraction, stopNiWebServer);
    }

    /** Returns a copy controlling whether low-memory tuning stops NI's web server. */
    public SystemTuning stopNiWebServer(boolean enabled) {
        return new SystemTuning(profile, swapBytes, warningAvailableFraction, criticalAvailableFraction, enabled);
    }

    public Profile profile() {
        return profile;
    }

    long swapBytes() {
        return swapBytes;
    }

    double warningAvailableFraction() {
        return warningAvailableFraction;
    }

    double criticalAvailableFraction() {
        return criticalAvailableFraction;
    }

    boolean stopNiWebServer() {
        return stopNiWebServer;
    }
}
