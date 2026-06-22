package ca.frc6390.athena.examples;

import ca.frc6390.athena.runtime.diagnostics.DiagnosticsChannel;
import ca.frc6390.athena.runtime.diagnostics.DiagnosticsSnapshot;

/**
 * Example diagnostics channel for bounded event logs and health summaries.
 */
public final class DiagnosticsExample {
    private DiagnosticsExample() {
    }

    /**
     * Creates a shooter diagnostics snapshot with summary fields and recent
     * events.
     *
     * @return diagnostics snapshot
     */
    public static DiagnosticsSnapshot shooterSnapshot() {
        DiagnosticsChannel channel = new DiagnosticsChannel("shooter", 8)
                .summary("state", "speaker")
                .summary("targetRpm", 4600);

        channel.info("enabled");
        channel.warn("below target velocity");
        channel.error("flywheel encoder disconnected");

        return channel.snapshot(4);
    }
}
