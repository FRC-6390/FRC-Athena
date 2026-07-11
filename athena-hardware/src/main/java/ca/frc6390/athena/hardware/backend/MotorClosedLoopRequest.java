package ca.frc6390.athena.hardware.backend;

/**
 * Per-cycle closed-loop request sent to a motor controller.
 */
public record MotorClosedLoopRequest(
        ControlRoute route,
        MotorClosedLoopConfig config,
        double arbitraryFeedforwardVolts) {
    public MotorClosedLoopRequest {
        route = route == null ? ControlRoute.DEVICE_CLOSED_LOOP : route;
        config = config == null ? MotorClosedLoopConfig.empty() : config;
        if (!Double.isFinite(arbitraryFeedforwardVolts)) {
            throw new IllegalArgumentException("Arbitrary feedforward voltage must be finite.");
        }
        if (route == ControlRoute.DEVICE_CLOSED_LOOP && arbitraryFeedforwardVolts != 0.0) {
            throw new IllegalArgumentException(
                    "Device closed-loop requests cannot include arbitrary feedforward voltage.");
        }
    }

    public static MotorClosedLoopRequest device(MotorClosedLoopConfig config) {
        return new MotorClosedLoopRequest(ControlRoute.DEVICE_CLOSED_LOOP, config, 0.0);
    }

    public static MotorClosedLoopRequest hybrid(MotorClosedLoopConfig config, double arbitraryFeedforwardVolts) {
        return new MotorClosedLoopRequest(ControlRoute.HYBRID_CLOSED_LOOP, config, arbitraryFeedforwardVolts);
    }
}
