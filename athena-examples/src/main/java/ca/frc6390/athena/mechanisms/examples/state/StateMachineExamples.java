package ca.frc6390.athena.mechanisms.examples;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateGraph;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.statespec.AthenaState;

/**
 * Example patterns for guarded state-machine transitions.
 */
public final class StateMachineExamples {
    private StateMachineExamples() {}

    @AthenaState(Double.class)
    public enum IntakeState implements SetpointProvider<Double> {
        STOW,
        CLEARANCE,
        INTAKE;

        @Override
        public Double getSetpoint() {
            return switch (this) {
                case STOW -> 0.0;
                case CLEARANCE -> 0.3;
                case INTAKE -> 1.0;
            };
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
