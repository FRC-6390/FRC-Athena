package ca.frc6390.athena.wpilib.system;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class JvmRecommendationsTest {
    private static final long MIB = 1024L * 1024L;

    @Test
    void recommendsBoundedLowOverheadJvmOnlyForRoborioOneClassTargets() {
        JvmRecommendations.Recommendation rioOne = JvmRecommendations.forTarget(
                true, "NI roboRIO 1.0", 256 * MIB, 256 * MIB);
        assertFalse(rioOne.arguments().isEmpty());
        assertTrue(rioOne.arguments().contains("-Xmx96m"));
        assertTrue(rioOne.arguments().contains("-Xss512k"));
        assertTrue(rioOne.arguments().contains("-XX:MaxDirectMemorySize=24m"));
        assertTrue(rioOne.arguments().contains("-XX:ReservedCodeCacheSize=32m"));
        assertTrue(rioOne.arguments().contains("-XX:MinHeapFreeRatio=5"));
        assertTrue(rioOne.arguments().contains("-XX:MaxHeapFreeRatio=20"));
        assertFalse(rioOne.healthy());

        assertTrue(JvmRecommendations.forTarget(
                true, "NI roboRIO 2.0", 2_048 * MIB, 256 * MIB).arguments().isEmpty());
        assertTrue(JvmRecommendations.forTarget(
                false, "Simulation", 256 * MIB, 256 * MIB).healthy());
    }
}
