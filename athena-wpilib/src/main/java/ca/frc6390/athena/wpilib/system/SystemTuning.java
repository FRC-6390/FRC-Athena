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
        /** Restore settings captured before Athena first tuned the target. */
        RESTORE_DEFAULTS,
        /** Disable both tuning and periodic system telemetry. */
        DISABLED
    }

    private static final long MIB = 1024L * 1024L;

    private final Profile profile;
    private final long zramBytes;
    private final long swapBytes;
    private final long minimumFreeDiskBytes;
    private final double warningAvailableFraction;
    private final double criticalAvailableFraction;
    private final int escalationSamples;
    private final int recoverySamples;
    private final boolean stopNiWebServer;

    private SystemTuning(
            Profile profile,
            long zramBytes,
            long swapBytes,
            long minimumFreeDiskBytes,
            double warningAvailableFraction,
            double criticalAvailableFraction,
            int escalationSamples,
            int recoverySamples,
            boolean stopNiWebServer) {
        this.profile = profile;
        this.zramBytes = zramBytes;
        this.swapBytes = swapBytes;
        this.minimumFreeDiskBytes = minimumFreeDiskBytes;
        this.warningAvailableFraction = warningAvailableFraction;
        this.criticalAvailableFraction = criticalAvailableFraction;
        this.escalationSamples = escalationSamples;
        this.recoverySamples = recoverySamples;
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

    public static SystemTuning restoreDefaults() {
        return defaults(Profile.RESTORE_DEFAULTS);
    }

    public static SystemTuning disabled() {
        return defaults(Profile.DISABLED);
    }

    private static SystemTuning defaults(Profile profile) {
        return new SystemTuning(profile, 64L * MIB, 32L * MIB, 24L * MIB, 0.14, 0.07, 3, 6, true);
    }

    /** Returns a copy with a bounded compressed zram size. */
    public SystemTuning zramMegabytes(int megabytes) {
        if (megabytes < 16 || megabytes > 128) {
            throw new IllegalArgumentException("Zram size must be between 16 and 128 MiB.");
        }
        return copy(megabytes * MIB, swapBytes, minimumFreeDiskBytes,
                warningAvailableFraction, criticalAvailableFraction, escalationSamples, recoverySamples,
                stopNiWebServer);
    }

    /** Returns a copy with a bounded fallback swap-file size. */
    public SystemTuning swapMegabytes(int megabytes) {
        if (megabytes < 16 || megabytes > 128) {
            throw new IllegalArgumentException("Swap size must be between 16 and 128 MiB.");
        }
        return copy(zramBytes, megabytes * MIB, minimumFreeDiskBytes,
                warningAvailableFraction, criticalAvailableFraction, escalationSamples, recoverySamples,
                stopNiWebServer);
    }

    /** Returns a copy with the disk headroom retained after creating fallback swap. */
    public SystemTuning minimumFreeDiskMegabytes(int megabytes) {
        if (megabytes < 16 || megabytes > 256) {
            throw new IllegalArgumentException("Minimum free disk space must be between 16 and 256 MiB.");
        }
        return copy(zramBytes, swapBytes, megabytes * MIB,
                warningAvailableFraction, criticalAvailableFraction, escalationSamples, recoverySamples,
                stopNiWebServer);
    }

    /** Returns a copy with memory-pressure thresholds expressed as available-RAM fractions. */
    public SystemTuning pressureThresholds(double warningFraction, double criticalFraction) {
        if (!Double.isFinite(warningFraction) || !Double.isFinite(criticalFraction)
                || criticalFraction <= 0.0 || warningFraction <= criticalFraction || warningFraction >= 1.0) {
            throw new IllegalArgumentException("Pressure thresholds require 0 < critical < warning < 1.");
        }
        return copy(zramBytes, swapBytes, minimumFreeDiskBytes, warningFraction, criticalFraction,
                escalationSamples, recoverySamples, stopNiWebServer);
    }

    /** Returns a copy with consecutive-sample requirements for escalation and recovery. */
    public SystemTuning pressureHysteresis(int escalationSamples, int recoverySamples) {
        if (escalationSamples < 1 || escalationSamples > 30
                || recoverySamples < escalationSamples || recoverySamples > 120) {
            throw new IllegalArgumentException(
                    "Pressure hysteresis requires 1 <= escalation <= recovery <= 120 samples.");
        }
        return copy(zramBytes, swapBytes, minimumFreeDiskBytes,
                warningAvailableFraction, criticalAvailableFraction, escalationSamples, recoverySamples,
                stopNiWebServer);
    }

    /** Returns a copy controlling whether low-memory tuning stops NI's web server. */
    public SystemTuning stopNiWebServer(boolean enabled) {
        return copy(zramBytes, swapBytes, minimumFreeDiskBytes,
                warningAvailableFraction, criticalAvailableFraction, escalationSamples, recoverySamples, enabled);
    }

    private SystemTuning copy(
            long zramBytes,
            long swapBytes,
            long minimumFreeDiskBytes,
            double warningAvailableFraction,
            double criticalAvailableFraction,
            int escalationSamples,
            int recoverySamples,
            boolean stopNiWebServer) {
        return new SystemTuning(profile, zramBytes, swapBytes, minimumFreeDiskBytes,
                warningAvailableFraction, criticalAvailableFraction, escalationSamples, recoverySamples,
                stopNiWebServer);
    }

    public Profile profile() { return profile; }
    long zramBytes() { return zramBytes; }
    long swapBytes() { return swapBytes; }
    long minimumFreeDiskBytes() { return minimumFreeDiskBytes; }
    double warningAvailableFraction() { return warningAvailableFraction; }
    double criticalAvailableFraction() { return criticalAvailableFraction; }
    int escalationSamples() { return escalationSamples; }
    int recoverySamples() { return recoverySamples; }
    boolean stopNiWebServer() { return stopNiWebServer; }
}
