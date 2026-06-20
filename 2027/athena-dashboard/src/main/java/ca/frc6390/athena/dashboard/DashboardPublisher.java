package ca.frc6390.athena.dashboard;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.runtime.diagnostics.DiagnosticsSnapshot;
import ca.frc6390.athena.telemetry.TelemetrySnapshot;

/**
 * Publishes telemetry and diagnostics snapshots to a dashboard sink.
 */
public final class DashboardPublisher {
    private final DashboardSink sink;

    /**
     * Creates a publisher.
     *
     * @param sink dashboard sink
     */
    public DashboardPublisher(DashboardSink sink) {
        this.sink = Objects.requireNonNull(sink, "sink");
    }

    /**
     * Publishes one dashboard packet.
     *
     * @param telemetry telemetry snapshot
     * @param diagnostics diagnostics snapshots
     * @return published packet
     */
    public DashboardPacket publish(TelemetrySnapshot telemetry, List<DiagnosticsSnapshot> diagnostics) {
        DashboardPacket packet = DashboardPacket.now(telemetry, diagnostics);
        sink.publish(packet);
        return packet;
    }
}
