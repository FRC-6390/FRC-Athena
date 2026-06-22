package ca.frc6390.athena.sim.mechanism;

import java.util.Objects;

import ca.frc6390.athena.mechanism.spec.ControlMode;
import ca.frc6390.athena.mechanism.spec.MechanismSpec;
import ca.frc6390.athena.mechanism.spec.MechanismStateSpec;
import ca.frc6390.athena.sim.world.SimMotorState;
import ca.frc6390.athena.sim.world.SimWorld;

/**
 * Simulation binding between a mechanism spec and a {@link SimWorld}.
 */
public final class SimMechanism {
    private final SimWorld world;
    private final MechanismSpec spec;
    private double percentOutputVelocityScale = 1.0;

    /**
     * Creates a mechanism simulation binding.
     *
     * @param world simulation world
     * @param spec mechanism spec
     */
    public SimMechanism(SimWorld world, MechanismSpec spec) {
        this.world = Objects.requireNonNull(world, "world");
        this.spec = Objects.requireNonNull(spec, "spec");
    }

    /**
     * Sets the velocity used when a percent-output mechanism is driven at full
     * output.
     *
     * @param velocityScale velocity per second at full output
     * @return this mechanism binding
     */
    public SimMechanism percentOutputVelocityScale(double velocityScale) {
        percentOutputVelocityScale = Double.isFinite(velocityScale) ? velocityScale : 0.0;
        return this;
    }

    /**
     * Applies a named mechanism state to all simulated motors in the mechanism.
     *
     * @param stateName mechanism state name
     * @return this mechanism binding
     */
    public SimMechanism applyState(String stateName) {
        MechanismStateSpec state = spec.states().stream()
                .filter(candidate -> candidate.name().equals(stateName))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "Unknown state " + stateName + " in mechanism " + spec.name() + "."));
        applyTarget(state.targetValue().orElse(0.0));
        return this;
    }

    /**
     * Applies a raw target using the mechanism control mode.
     *
     * @param target requested target
     * @return this mechanism binding
     */
    public SimMechanism applyTarget(double target) {
        double safeTarget = Double.isFinite(target) ? target : 0.0;
        spec.motors().forEach(motor -> {
            SimMotorState state = world.motor(motor.path());
            if (spec.controlMode() == ControlMode.PERCENT_OUTPUT) {
                state.percentOutput(safeTarget).velocityPerSecond(state.percentOutput() * percentOutputVelocityScale);
            } else if (spec.controlMode() == ControlMode.VELOCITY) {
                state.velocityPerSecond(safeTarget);
            } else if (spec.controlMode() == ControlMode.POSITION) {
                state.position(safeTarget).velocityPerSecond(0.0);
            } else {
                state.percentOutput(0.0).velocityPerSecond(0.0);
            }
        });
        return this;
    }

    /**
     * Returns the simulated state for a named motor.
     *
     * @param motorName motor name inside the mechanism
     * @return simulated motor state
     */
    public SimMotorState motor(String motorName) {
        return spec.motors().stream()
                .filter(motor -> motor.name().equals(motorName))
                .findFirst()
                .map(motor -> world.motor(motor.path()))
                .orElseThrow(() -> new IllegalArgumentException(
                        "Unknown motor " + motorName + " in mechanism " + spec.name() + "."));
    }
}
