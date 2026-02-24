package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.logging.Telemetry;
import ca.frc6390.athena.logging.TelemetryDestination;
import ca.frc6390.athena.logging.TelemetryRegistry;

/**
 * Example model + registry wiring for annotation-based telemetry publishing.
 */
public final class TelemetryRegistryExamples {
    private TelemetryRegistryExamples() {}

    public static final class DriveTelemetryModel {
        @Telemetry(key = "Drive/Velocity", destination = TelemetryDestination.BOTH, periodMs = 20, epsilon = 0.01)
        public double velocityMps = 0.0;

        @Telemetry(key = "Drive/Enabled", destination = TelemetryDestination.SHUFFLEBOARD, periodMs = 20)
        public boolean enabled = true;

        @Telemetry(key = "Drive/State", destination = TelemetryDestination.DISK, periodMs = 50)
        public String state = "IDLE";
    }

    public static TelemetryRegistry createNoSinkRegistry() {
        TelemetryRegistry.TelemetryConfig config = TelemetryRegistry.TelemetryConfig.defaults()
                .diskEnabled(false)
                .networkTablesEnabled(false)
                .defaultPeriodMs(20);
        return TelemetryRegistry.create(config);
    }

    public static void registerAndTick(TelemetryRegistry registry, Object model, long nowMs) {
        registry.register(model);
        registry.tick(nowMs);
    }
}
