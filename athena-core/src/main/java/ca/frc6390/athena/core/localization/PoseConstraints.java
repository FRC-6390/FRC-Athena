package ca.frc6390.athena.core.localization;

public record PoseConstraints(
        double minPeriodSeconds,
        double maxLatencySeconds,
        boolean requireVision,
        boolean allowSlip) {

    public PoseConstraints {
        if (!Double.isFinite(minPeriodSeconds) || minPeriodSeconds < 0.0) {
            minPeriodSeconds = 0.0;
        }
        if (!Double.isFinite(maxLatencySeconds) || maxLatencySeconds < 0.0) {
            maxLatencySeconds = 0.0;
        }
    }

    public boolean canUpdate(
            double nowSeconds,
            double lastUpdateSeconds,
            boolean hasFreshVision,
            boolean slipActive) {
        if (minPeriodSeconds > 0.0 && (nowSeconds - lastUpdateSeconds) < minPeriodSeconds) {
            return false;
        }
        if (requireVision && !hasFreshVision) {
            return false;
        }
        if (!allowSlip && slipActive) {
            return false;
        }
        return true;
    }

    public static PoseConstraints defaults() {
        return new PoseConstraints(0.0, 0.0, false, true);
    }
}
