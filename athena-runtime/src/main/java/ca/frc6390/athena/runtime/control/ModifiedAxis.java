package ca.frc6390.athena.runtime.control;

import java.util.function.DoubleSupplier;

/**
 * Controller axis shaper with deadzone, optional squaring, and inversion.
 */
public final class ModifiedAxis implements DoubleSupplier {
    private final DoubleSupplier input;
    private double deadzone;
    private boolean squared;
    private boolean inverted;

    /**
     * Creates an axis shaper.
     *
     * @param input raw axis supplier
     * @param deadzone deadzone magnitude in [0, 1)
     */
    public ModifiedAxis(DoubleSupplier input, double deadzone) {
        this.input = input == null ? () -> 0.0 : input;
        this.deadzone = sanitizeDeadzone(deadzone);
    }

    /**
     * Sets deadzone.
     *
     * @param deadzone deadzone magnitude
     * @return this axis
     */
    public ModifiedAxis deadzone(double deadzone) {
        this.deadzone = sanitizeDeadzone(deadzone);
        return this;
    }

    /**
     * Sets whether output should be squared while preserving sign.
     *
     * @param squared true to square
     * @return this axis
     */
    public ModifiedAxis squared(boolean squared) {
        this.squared = squared;
        return this;
    }

    /**
     * Sets whether output should be inverted.
     *
     * @param inverted true to invert
     * @return this axis
     */
    public ModifiedAxis inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    @Override
    public double getAsDouble() {
        double value = applyDeadzone(input.getAsDouble());
        if (squared) {
            value = Math.copySign(value * value, value);
        }
        return inverted ? -value : value;
    }

    private double applyDeadzone(double value) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        double clamped = Math.max(-1.0, Math.min(1.0, value));
        if (Math.abs(clamped) <= deadzone) {
            return 0.0;
        }
        return clamped > 0.0
                ? (clamped - deadzone) / (1.0 - deadzone)
                : (clamped + deadzone) / (1.0 - deadzone);
    }

    private double sanitizeDeadzone(double requested) {
        if (!Double.isFinite(requested)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(0.99, Math.abs(requested)));
    }
}
