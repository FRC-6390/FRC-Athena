package ca.frc6390.athena.mechanism.runtime;

import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.mechanism.spec.ControlMode;
import ca.frc6390.athena.mechanism.spec.MechanismSpec;
import ca.frc6390.athena.mechanism.spec.MechanismStateSpec;
import java.util.List;
import java.util.Objects;

/**
 * Runtime controller that applies named mechanism states to motor devices.
 */
public final class MechanismController {
    private final MechanismSpec spec;
    private final List<MotorDevice> motors;

    private MechanismController(MechanismSpec spec, List<MotorDevice> motors) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.motors = List.copyOf(motors);
        if (this.motors.size() != spec.motors().size()) {
            throw new IllegalArgumentException(
                    "Mechanism " + spec.name() + " expected " + spec.motors().size()
                            + " motors, got " + this.motors.size() + ".");
        }
    }

    /**
     * Creates a controller for an already-instantiated mechanism.
     *
     * @param spec mechanism spec
     * @param motors motor devices in spec order
     * @return controller
     */
    public static MechanismController of(MechanismSpec spec, List<MotorDevice> motors) {
        return new MechanismController(spec, motors);
    }

    /**
     * Returns the mechanism spec.
     *
     * @return mechanism spec
     */
    public MechanismSpec spec() {
        return spec;
    }

    /**
     * Applies a named state to every motor in this mechanism.
     *
     * @param stateName state name
     */
    public void applyState(String stateName) {
        MechanismStateSpec state = spec.states().stream()
                .filter(candidate -> candidate.name().equals(stateName))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "Mechanism " + spec.name() + " does not declare state " + stateName + "."));
        double target = state.targetValue()
                .orElseThrow(() -> new IllegalArgumentException(
                        "Mechanism state " + spec.name() + "." + state.name() + " does not declare a target."));
        applyTarget(target);
    }

    /**
     * Applies an explicit target using this mechanism's declared control mode.
     *
     * @param target target value interpreted by the control mode
     */
    public void applyTarget(double target) {
        double finiteTarget = Double.isFinite(target) ? target : 0.0;
        for (MotorDevice motor : motors) {
            applyToMotor(motor, finiteTarget);
        }
    }

    /**
     * Stops every motor in this mechanism.
     */
    public void stop() {
        for (MotorDevice motor : motors) {
            motor.stop();
        }
    }

    private void applyToMotor(MotorDevice motor, double target) {
        ControlMode mode = spec.controlMode();
        switch (mode) {
            case PERCENT_OUTPUT -> motor.setPercentOutput(target);
            case POSITION -> motor.setPositionTargetRotations(target);
            case VELOCITY -> motor.setVelocityTargetRotationsPerSecond(target);
            case NONE -> throw new IllegalStateException(
                    "Mechanism " + spec.name() + " does not declare a runtime control mode.");
            default -> throw new IllegalStateException("Unhandled mechanism control mode " + mode + ".");
        }
    }
}
