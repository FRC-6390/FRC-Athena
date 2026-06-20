package ca.frc6390.athena.dashboard;

import java.time.Instant;
import java.util.List;

import ca.frc6390.athena.runtime.diagnostics.DiagnosticsSnapshot;
import ca.frc6390.athena.telemetry.TelemetrySnapshot;

/**
 * Snapshot packet sent from robot code to a dashboard transport.
 *
 * @param timestamp packet timestamp
 * @param telemetry telemetry snapshot
 * @param diagnostics diagnostics snapshots
 */
public record DashboardPacket(
        Instant timestamp,
        TelemetrySnapshot telemetry,
        List<DiagnosticsSnapshot> diagnostics) {
    public DashboardPacket {
        timestamp = timestamp == null ? Instant.EPOCH : timestamp;
        telemetry = telemetry == null ? new TelemetrySnapshot(java.util.Map.of()) : telemetry;
        diagnostics = diagnostics == null ? List.of() : List.copyOf(diagnostics);
    }

    /**
     * Creates a packet with the current clock.
     *
     * @param telemetry telemetry snapshot
     * @param diagnostics diagnostics snapshots
     * @return dashboard packet
     */
    public static DashboardPacket now(TelemetrySnapshot telemetry, List<DiagnosticsSnapshot> diagnostics) {
        return new DashboardPacket(Instant.now(), telemetry, diagnostics);
    }

    /**
     * Returns true when any diagnostics snapshot contains an error.
     *
     * @return true if unhealthy
     */
    public boolean hasErrors() {
        return diagnostics.stream().anyMatch(DiagnosticsSnapshot::hasErrors);
    }
}
