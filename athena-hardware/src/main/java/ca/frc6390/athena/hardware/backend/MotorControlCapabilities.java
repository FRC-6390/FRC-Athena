package ca.frc6390.athena.hardware.backend;

/**
 * Closed-loop features a motor handle can execute on-device.
 */
public record MotorControlCapabilities(
        boolean positionClosedLoop,
        boolean velocityClosedLoop,
        boolean arbitraryVoltageFeedforward,
        boolean motionProfile,
        boolean focVoltage,
        boolean focTorqueCurrent,
        int closedLoopSlots) {
    /**
     * No onboard closed-loop support.
     */
    public static final MotorControlCapabilities OPEN_LOOP_ONLY =
            new MotorControlCapabilities(false, false, false, false, false, false, 0);

    /**
     * Generic onboard position and velocity PID with voltage feedforward.
     */
    public static MotorControlCapabilities voltageClosedLoop(int slots) {
        return new MotorControlCapabilities(true, true, true, false, false, false, slots);
    }

    public MotorControlCapabilities {
        closedLoopSlots = Math.max(0, closedLoopSlots);
    }

    public boolean supportsPosition() {
        return positionClosedLoop;
    }

    public boolean supportsVelocity() {
        return velocityClosedLoop;
    }

    public boolean supportsSlot(int slot) {
        return slot >= 0 && slot < closedLoopSlots;
    }
}
