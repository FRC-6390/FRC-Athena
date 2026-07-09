package ca.frc6390.athena.plugin.features;

/**
 * Internal catalog of Athena module artifacts selected by the Gradle plugin.
 */
enum ModuleArtifact {
    API("api", "athena-api", true),
    RUNTIME("runtime", "athena-runtime", true),
    HARDWARE("hardware", "athena-hardware", true),
    MECHANISMS("mechanisms", "athena-mechanisms", true),
    COMMANDS("commands", "athena-commands", true),
    DRIVETRAIN("drivetrain", "athena-drivetrain", false),
    VISION("vision", "athena-vision", false),
    AUTO("auto", "athena-auto", false),
    LOCALIZATION("localization", "athena-localization", false),
    WPILIB("wpilib", "athena-wpilib", false),
    SIMULATION("simulation", "athena-simulation", false);

    private final String requestName;
    private final String artifactId;
    private final boolean defaultEnabled;

    ModuleArtifact(String requestName, String artifactId, boolean defaultEnabled) {
        this.requestName = requestName;
        this.artifactId = artifactId;
        this.defaultEnabled = defaultEnabled;
    }

    String requestName() {
        return requestName;
    }

    String artifactId() {
        return artifactId;
    }

    boolean defaultEnabled() {
        return defaultEnabled;
    }

    String coordinate(String group, String version) {
        return group + ":" + artifactId + ":" + version;
    }

    static ModuleArtifact fromRequestName(String name) {
        String normalized = normalize(name);
        for (ModuleArtifact module : values()) {
            if (module.requestName.equals(normalized)) {
                return module;
            }
        }
        throw new IllegalArgumentException("Unknown Athena module '" + normalized + "'.");
    }

    static String normalize(String name) {
        return name == null ? "" : name.trim().replace('_', '-').toLowerCase(java.util.Locale.ROOT);
    }
}
