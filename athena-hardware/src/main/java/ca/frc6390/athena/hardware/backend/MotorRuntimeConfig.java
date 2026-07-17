package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import java.util.Objects;

/**
 * Portable motor settings that Athena may override while the robot program is running.
 *
 * <p>The {@link MotorDevice} remains the immutable deployment declaration. Runtime telemetry and
 * commissioning tools apply this value to its already-created {@link MotorHandle}.</p>
 */
public record MotorRuntimeConfig(
        MotorNeutralMode neutralMode,
        boolean inverted,
        int supplyCurrentLimitAmps,
        int statorCurrentLimitAmps) {
    public MotorRuntimeConfig {
        neutralMode = Objects.requireNonNull(neutralMode, "neutralMode");
        if (supplyCurrentLimitAmps < 0 || statorCurrentLimitAmps < 0) {
            throw new IllegalArgumentException("Runtime motor current limits cannot be negative.");
        }
    }

    /** Returns the portable runtime configuration declared in robot code. */
    public static MotorRuntimeConfig declared(MotorDevice device) {
        MotorDevice safeDevice = Objects.requireNonNull(device, "device");
        var limits = safeDevice.currentLimits();
        return new MotorRuntimeConfig(
                safeDevice.neutralMode(),
                safeDevice.isInverted(),
                limits.supplyAmps(),
                limits.statorAmps());
    }
}
