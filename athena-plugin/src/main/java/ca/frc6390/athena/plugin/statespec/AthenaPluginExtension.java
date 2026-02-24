package ca.frc6390.athena.plugin.statespec;

public class AthenaPluginExtension {
    private boolean enabled = true;
    private boolean selfArtifactClasspathEnabled = true;

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isSelfArtifactClasspathEnabled() {
        return selfArtifactClasspathEnabled;
    }

    public void setSelfArtifactClasspathEnabled(boolean selfArtifactClasspathEnabled) {
        this.selfArtifactClasspathEnabled = selfArtifactClasspathEnabled;
    }
}
