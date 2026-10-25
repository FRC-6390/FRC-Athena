package ca.frc6390.athena.wpilib.system;

import java.util.List;

final class JvmRecommendations {
    private static final long MIB = 1024L * 1024L;

    record Recommendation(List<String> arguments, boolean healthy) {
        Recommendation {
            arguments = List.copyOf(arguments);
        }
    }

    /**
     * Only the fast C1 JIT compiler. With the default tiered C1 + C2 compilation the C2 compiler keeps a roboRIO
     * core busy for the first minutes of a boot; C1 alone finishes in seconds, and robot code runs about as fast.
     */
    static final String C1_ONLY = "-XX:TieredStopAtLevel=1";

    static Recommendation forTarget(boolean realRobot, String target, long totalMemory, long heapMaximum) {
        if (!realRobot) return new Recommendation(List.of(), true);
        boolean rio = target.toLowerCase(java.util.Locale.ROOT).contains("roborio");
        boolean rioOne = rio && totalMemory > 0 && totalMemory <= 512L * MIB;
        if (!rioOne) return new Recommendation(rio ? List.of(C1_ONLY) : List.of(), true);
        List<String> arguments = List.of(
                C1_ONLY,
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
