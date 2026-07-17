package ca.frc6390.athena.wpilib.system;

import java.util.List;

final class JvmRecommendations {
    private static final long MIB = 1024L * 1024L;

    record Recommendation(List<String> arguments, boolean healthy) {
        Recommendation {
            arguments = List.copyOf(arguments);
        }
    }

    static Recommendation forTarget(boolean realRobot, String target, long totalMemory, long heapMaximum) {
        if (!realRobot) return new Recommendation(List.of(), true);
        boolean rioOne = target.toLowerCase(java.util.Locale.ROOT).contains("roborio")
                && totalMemory > 0 && totalMemory <= 512L * MIB;
        if (!rioOne) return new Recommendation(List.of(), true);
        List<String> arguments = List.of(
                "-Xms16m",
                "-Xmx96m",
                "-Xss512k",
                "-XX:MaxDirectMemorySize=24m",
                "-XX:ReservedCodeCacheSize=32m",
                "-XX:MinHeapFreeRatio=5",
                "-XX:MaxHeapFreeRatio=20");
        boolean healthy = heapMaximum >= 72L * MIB && heapMaximum <= 112L * MIB;
        return new Recommendation(arguments, healthy);
    }
}
