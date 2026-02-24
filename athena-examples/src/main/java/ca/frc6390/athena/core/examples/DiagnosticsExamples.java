package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.diagnostics.DiagnosticsChannel;
import java.util.Map;

/**
 * Example diagnostics logging conventions.
 */
public final class DiagnosticsExamples {
    private DiagnosticsExamples() {}

    public static DiagnosticsChannel createSubsystemChannel(String subsystem, int capacity) {
        return new DiagnosticsChannel("robot/" + subsystem, capacity);
    }

    public static void publishLoopTiming(DiagnosticsChannel channel, double periodMs, double jitterMs) {
        channel.field("loopPeriodMs", periodMs);
        channel.field("loopJitterMs", jitterMs);
        if (jitterMs > 2.0) {
            channel.warn("timing", "Loop jitter exceeded threshold");
        } else {
            channel.info("timing", "Loop timing nominal");
        }
    }

    public static Map<String, Object> snapshot(DiagnosticsChannel channel, int limit) {
        return channel.snapshot(limit);
    }
}
