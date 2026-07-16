package ca.frc6390.athena.hardware.signal;

/** Latest command Athena applied to a motor. */
public record MotorCommandSnapshot(Mode mode, double value) {
    /** Motor command modes relevant to physical movement intent. */
    public enum Mode {
        NEUTRAL,
        PERCENT,
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    private static final MotorCommandSnapshot NEUTRAL = new MotorCommandSnapshot(Mode.NEUTRAL, 0.0);

    public MotorCommandSnapshot {
        if (mode == null) {
            throw new NullPointerException("mode");
        }
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException("Motor command value must be finite.");
        }
    }

    /** Returns the shared neutral command snapshot. */
    public static MotorCommandSnapshot neutral() {
        return NEUTRAL;
    }

    /** Returns whether this command asks the motor to produce movement. */
    public boolean requestsMovement(double minimumOpenLoopVolts, double velocityThreshold) {
        return switch (mode) {
            case NEUTRAL -> false;
            case PERCENT -> Math.abs(value) * 12.0 >= minimumOpenLoopVolts;
            case VOLTAGE -> Math.abs(value) >= minimumOpenLoopVolts;
            case VELOCITY -> Math.abs(value) > velocityThreshold;
            case POSITION -> true;
        };
    }
}
