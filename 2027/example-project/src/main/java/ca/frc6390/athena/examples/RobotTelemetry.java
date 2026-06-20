package ca.frc6390.athena.examples;

import ca.frc6390.athena.telemetry.TelemetryKey;
import ca.frc6390.athena.telemetry.TelemetryRegistry;
import ca.frc6390.athena.telemetry.networktables.InMemoryNetworkTableWriter;
import ca.frc6390.athena.telemetry.networktables.NetworkTablesTelemetrySink;

/**
 * Example telemetry declarations.
 */
public final class RobotTelemetry {
    /** Intake running state. */
    public static final TelemetryKey INTAKE_RUNNING = TelemetryKey.bool("intake/running");

    /** Shooter target velocity. */
    public static final TelemetryKey SHOOTER_TARGET_RPM = TelemetryKey.number("shooter/targetRpm");

    private RobotTelemetry() {
    }

    /**
     * Creates a telemetry registry for the example robot.
     *
     * @param intakeRunning current intake state
     * @param shooterTargetRpm current shooter target
     * @return telemetry registry
     */
    public static TelemetryRegistry create(boolean intakeRunning, double shooterTargetRpm) {
        return new TelemetryRegistry()
                .booleanValue(INTAKE_RUNNING, () -> intakeRunning)
                .numberValue(SHOOTER_TARGET_RPM, () -> shooterTargetRpm);
    }

    /**
     * Publishes example telemetry through a NetworkTables-shaped sink.
     *
     * @param intakeRunning current intake state
     * @param shooterTargetRpm current shooter target
     * @return in-memory writer containing published paths
     */
    public static InMemoryNetworkTableWriter publishToNetworkTables(boolean intakeRunning, double shooterTargetRpm) {
        InMemoryNetworkTableWriter writer = new InMemoryNetworkTableWriter();
        create(intakeRunning, shooterTargetRpm).publishAll(new NetworkTablesTelemetrySink(writer));
        return writer;
    }
}
