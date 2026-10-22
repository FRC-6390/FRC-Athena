package ca.frc6390.athena.hardware.backend;

/** Cached acknowledgement of the latest required hardware configuration. No method performs I/O. */
public record ConfigurationStatus(long requestedGeneration, long appliedGeneration, boolean pending, String error) {
    public static final ConfigurationStatus READY = new ConfigurationStatus(0, 0, false, "");

    public ConfigurationStatus {
        error = error == null ? "" : error;
    }

    public boolean ready() { return !pending && error.isEmpty() && requestedGeneration == appliedGeneration; }
    public boolean failed() { return !error.isEmpty(); }
}
