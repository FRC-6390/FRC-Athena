package ca.frc6390.athena.mechanisms.examples;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateGraph;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Example patterns for guarded state-machine transitions.
 */
public final class StateMachineExamples {
    private StateMachineExamples() {}

    public enum IntakeState implements SetpointProvider<Double> {
        STOW(0.0),
        CLEARANCE(0.3),
        INTAKE(1.0);

        private final double setpoint;

        IntakeState(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    public static StateGraph<IntakeState> createIntakeGraph(BooleanSupplier clearanceReady) {
        Objects.requireNonNull(clearanceReady, "clearanceReady");
        return StateGraph
                .create(IntakeState.class)
                .path(IntakeState.STOW, IntakeState.CLEARANCE, IntakeState.INTAKE)
                .path(IntakeState.INTAKE, IntakeState.CLEARANCE, IntakeState.STOW)
                .guard(IntakeState.STOW, IntakeState.CLEARANCE, clearanceReady);
    }

    public static StateMachine<Double, IntakeState> createIntakeMachine(
            BooleanSupplier atStateSupplier,
            BooleanSupplier clearanceReady) {
        Objects.requireNonNull(atStateSupplier, "atStateSupplier");
        StateMachine<Double, IntakeState> machine = new StateMachine<>(IntakeState.STOW, atStateSupplier);
        machine.setAtStateDelay(0.0);
        machine.setStateGraph(createIntakeGraph(clearanceReady != null ? clearanceReady : () -> true));
        // Consume the constructor-seeded initial queue entry.
        machine.update();
        return machine;
    }

    public static void forceIntake(StateMachine<Double, IntakeState> machine) {
        machine.force(IntakeState.INTAKE);
    }

    public static void forceStow(StateMachine<Double, IntakeState> machine, boolean append) {
        machine.force(IntakeState.STOW, append);
    }
}
