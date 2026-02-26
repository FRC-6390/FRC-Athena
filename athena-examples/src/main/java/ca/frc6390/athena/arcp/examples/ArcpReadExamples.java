package ca.frc6390.athena.arcp.examples;

import ca.frc6390.athena.core.arcp.ARCP;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalLong;

/**
 * Read/cache examples for ARCP.
 */
public final class ArcpReadExamples {
    private ArcpReadExamples() {}

    public record DriveSnapshot(
            boolean enabled,
            double velocityMps,
            double outputVolts,
            long profileIndex,
            String mode,
            String faultReason) {}

    public static DriveSnapshot readDriveSnapshot(ARCP arcp) {
        Optional<Boolean> enabled = arcp.getBoolean("Athena/Drive/Enabled");
        OptionalDouble velocityMps = arcp.getDouble("Athena/Drive/VelocityMps");
        OptionalDouble outputVolts = arcp.getDouble("Athena/Drive/OutputVolts");
        OptionalLong profileIndex = arcp.getI64("Athena/Drive/ProfileIndex");
        Optional<String> mode = arcp.getString("Athena/Drive/Mode");
        Optional<String> faultReason = arcp.get("Athena/Drive/FaultReason", String.class);

        return new DriveSnapshot(
                enabled.orElse(false),
                velocityMps.orElse(0.0),
                outputVolts.orElse(0.0),
                profileIndex.orElse(-1L),
                mode.orElse("unknown"),
                faultReason.orElse(""));
    }

    public static Optional<Double> readAsGenericDouble(ARCP arcp, String signalPath) {
        return arcp.get(signalPath, Double.class);
    }

    public static Optional<Long> readAsGenericI64(ARCP arcp, String signalPath) {
        return arcp.get(signalPath, Long.class);
    }

    public static Optional<Boolean> readAsGenericBool(ARCP arcp, String signalPath) {
        return arcp.get(signalPath, Boolean.class);
    }

    public static Optional<String> readAsGenericString(ARCP arcp, String signalPath) {
        return arcp.get(signalPath, String.class);
    }

    public static Map<String, Object> metadataSnapshot(ARCP arcp, String signalPath) {
        return arcp.meta(signalPath).snapshot();
    }
}
