package ca.frc6390.athena.core;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;

public final class LoopTiming {
    private static final double LOOP_BUDGET_SECONDS = 0.02;
    private static final long REPORT_PERIOD_MS = 1000;
    private static volatile boolean sampleMechanismsNext = false;
    private static volatile boolean samplingMechanisms = false;
    private static volatile boolean debugAlways = false;
    private static long lastReportMs = 0;
    private static final Map<String, Double> mechanismDurationsMs = new HashMap<>();

    private LoopTiming() {}

    static void beginCycle() {
        // Cache a single "now" timestamp for this robot loop.
        RobotTime.updateNowSeconds(Timer.getFPGATimestamp());
        samplingMechanisms = debugAlways || sampleMechanismsNext;
        sampleMechanismsNext = false;
        if (samplingMechanisms) {
            mechanismDurationsMs.clear();
        }
    }

    public static void setDebugAlways(boolean enabled) {
        debugAlways = enabled;
    }

    public static boolean shouldSampleMechanisms() {
        return samplingMechanisms;
    }

    public static void recordMechanism(String name, double durationMs) {
        if (!samplingMechanisms || name == null) {
            return;
        }
        Double previous = mechanismDurationsMs.get(name);
        mechanismDurationsMs.put(name, previous != null ? previous + durationMs : durationMs);
    }

    static void endCycle(double t0, double t1, double t2, double t3, double t4) {
        double total = t4 - t0;
        boolean overBudget = total > LOOP_BUDGET_SECONDS;
        if (!debugAlways && !overBudget) {
            return;
        }
        long nowMs = (long) (t4 * 1000.0);
        if (nowMs - lastReportMs < REPORT_PERIOD_MS) {
            return;
        }
        lastReportMs = nowMs;

        double schedMs = (t1 - t0) * 1000.0;
        double telemetryMs = (t2 - t1) * 1000.0;
        double localMs = (t3 - t2) * 1000.0;
        double userMs = (t4 - t3) * 1000.0;
        double totalMs = total * 1000.0;

        StringBuilder message = new StringBuilder(128);
        message.append("Loop overrun: total=");
        appendMs(message, totalMs);
        message.append(" (scheduler=");
        appendMs(message, schedMs);
        message.append(", telemetry=");
        appendMs(message, telemetryMs);
        message.append(", localization=");
        appendMs(message, localMs);
        message.append(", user=");
        appendMs(message, userMs);
        message.append(')');

        if (samplingMechanisms && !mechanismDurationsMs.isEmpty()) {
            message.append(" | Top mechanisms: ");
            appendTopMechanisms(message);
        } else if (!debugAlways) {
            message.append(" | Sampling mechanisms next cycle.");
            sampleMechanismsNext = true;
        }

        String text = message.toString();
        if (overBudget) {
            DriverStation.reportWarning(text, false);
        }
        if (debugAlways || overBudget) {
            System.out.println("[Athena][LoopTiming] " + text);
        }
    }

    private static void appendMs(StringBuilder message, double ms) {
        if (!Double.isFinite(ms)) {
            message.append("NaNms");
            return;
        }
        long tenths = Math.round(ms * 10.0);
        long whole = tenths / 10;
        long fractional = Math.abs(tenths % 10);
        message.append(whole).append('.').append(fractional).append("ms");
    }

    private static void appendTopMechanisms(StringBuilder message) {
        String top1 = null;
        String top2 = null;
        String top3 = null;
        double top1Ms = -1.0;
        double top2Ms = -1.0;
        double top3Ms = -1.0;
        for (Map.Entry<String, Double> entry : mechanismDurationsMs.entrySet()) {
            double value = entry.getValue();
            if (value > top1Ms) {
                top3 = top2;
                top3Ms = top2Ms;
                top2 = top1;
                top2Ms = top1Ms;
                top1 = entry.getKey();
                top1Ms = value;
            } else if (value > top2Ms) {
                top3 = top2;
                top3Ms = top2Ms;
                top2 = entry.getKey();
                top2Ms = value;
            } else if (value > top3Ms) {
                top3 = entry.getKey();
                top3Ms = value;
            }
        }
        boolean wrote = false;
        if (top1 != null) {
            message.append(top1).append('=');
            appendMs(message, top1Ms);
            wrote = true;
        }
        if (top2 != null) {
            message.append(wrote ? ", " : "").append(top2).append('=');
            appendMs(message, top2Ms);
            wrote = true;
        }
        if (top3 != null) {
            message.append(wrote ? ", " : "").append(top3).append('=');
            appendMs(message, top3Ms);
        }
    }
}
