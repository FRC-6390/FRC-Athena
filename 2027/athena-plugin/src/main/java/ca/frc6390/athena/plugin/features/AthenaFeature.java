package ca.frc6390.athena.plugin.features;

/**
 * Built-in Athena feature artifacts that are not vendor adapters.
 */
public enum AthenaFeature {
    /** Public API artifact. */
    API("athena-api", true),

    /** Runtime artifact. */
    RUNTIME("athena-runtime", true),

    /** Hardware abstraction artifact. */
    HARDWARE("athena-hardware", true),

    /** Mechanism declaration artifact. */
    MECHANISMS("athena-mechanisms", true),

    /** Command integration artifact. */
    COMMANDS("athena-commands", true),

    /** Telemetry artifact. */
    TELEMETRY("athena-telemetry", true),

    /** Gradle/plugin support artifact. */
    PLUGIN("athena-plugin", true),

    /** Drivetrain artifact. */
    DRIVETRAIN("athena-drivetrain", false),

    /** Superstructure artifact. */
    SUPERSTRUCTURE("athena-superstructure", false),

    /** Generic vision artifact. */
    VISION("athena-vision", false),

    /** Autonomous routine artifact. */
    AUTO("athena-auto", false),

    /** Localization artifact. */
    LOCALIZATION("athena-localization", false),

    /** WPILib adapter artifact. */
    WPILIB("athena-wpilib", false),

    /** Dashboard/control bridge artifact. */
    DASHBOARD("athena-dashboard", false),

    /** Simulation artifact. */
    SIMULATION("athena-simulation", false);

    private final String artifactId;
    private final boolean defaultEnabled;

    AthenaFeature(String artifactId, boolean defaultEnabled) {
        this.artifactId = artifactId;
        this.defaultEnabled = defaultEnabled;
    }

    /**
     * Returns artifact id.
     *
     * @return artifact id
     */
    public String artifactId() {
        return artifactId;
    }

    /**
     * Returns whether this feature is part of the default Athena install.
     *
     * @return true if default-enabled
     */
    public boolean defaultEnabled() {
        return defaultEnabled;
    }

    /**
     * Returns a dependency coordinate.
     *
     * @param group group id
     * @param version version
     * @return full dependency coordinate
     */
    public String coordinate(String group, String version) {
        return group + ":" + artifactId + ":" + version;
    }
}
