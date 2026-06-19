package ca.frc6390.athena.mechanisms.examples.state;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateGraph;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.statespec.StateBuilder;
import ca.frc6390.athena.mechanisms.statespec.StateDsl;
import ca.frc6390.athena.mechanisms.statespec.StateSeed;
import ca.frc6390.athena.mechanisms.statespec.StateSeedProvider;
import ca.frc6390.athena.mechanisms.statespec.StateSeedRuntime;

/**
 * State DSL example that combines:
 * - explicit state contracts visible to the IDE
 * - tagged DSL methods wired through normal Java references
 * - guarded {@link StateGraph} path expansion
 * - DSL-driven auto transition chaining ({@code until(...).then(...)}).
 */
public final class StateDslExamples {
    private StateDslExamples() {}

    public enum DslState implements SetpointProvider<Double>, StateSeedProvider<DslState> {
        STOW,
        FEED,
        LATCH,
        UNJAM,
        SCORE;

        @Override
        public Double getSetpoint() {
            return switch (this) {
                case STOW -> 0.0;
                case FEED -> 0.45;
                case LATCH -> 0.2;
                case UNJAM -> -0.25;
                case SCORE -> 0.85;
            };
        }

        @Override
        public StateSeed<DslState> seed() {
            return switch (this) {
                case STOW -> StateSeed.auto();
                case FEED -> logicSeed(StateDslExamples::feedLogic);
                case LATCH -> StateSeed.setpoint(0.2);
                case UNJAM -> logicSeed(StateDslExamples::unjamLogic);
                case SCORE -> logicSeed(StateDslExamples::scoreLogic);
            };
        }
    }

    public static StateBuilder<DslState> feedLogic(StateBuilder<DslState> builder) {
        return builder
                .setpoint(0.45)
                .manualPercent(0.35)
                .until(ctx -> ctx.timeInState() >= 0.0)
                .then(DslState.LATCH);
    }

    public static StateBuilder<DslState> unjamLogic(StateBuilder<DslState> builder) {
        return builder
                .setpoint(-0.25)
                .manualPercent(-0.4)
                .until(ctx -> ctx.timeInState() >= 0.0)
                .then(DslState.FEED);
    }

    public static StateBuilder<DslState> scoreLogic(StateBuilder<DslState> builder) {
        return builder
                .setpoint(0.85)
                .until(ctx -> ctx.timeInState() >= 0.0)
                .then(DslState.STOW);
    }

    public static StateGraph<DslState> createGraph(
            BooleanSupplier feedAllowed,
            BooleanSupplier scoreAllowed) {
        Objects.requireNonNull(feedAllowed, "feedAllowed");
        Objects.requireNonNull(scoreAllowed, "scoreAllowed");

        return StateGraph
                .create(DslState.class)
                .path(DslState.STOW, DslState.FEED, DslState.LATCH, DslState.SCORE)
                .path(DslState.SCORE, DslState.LATCH, DslState.FEED, DslState.STOW)
                .guard(DslState.STOW, DslState.FEED, feedAllowed)
                .guard(DslState.LATCH, DslState.SCORE, scoreAllowed);
    }

    public static StateMachine<Double, DslState> createMachine(
            BooleanSupplier atStateSupplier,
            BooleanSupplier feedAllowed,
            BooleanSupplier scoreAllowed) {
        Objects.requireNonNull(atStateSupplier, "atStateSupplier");

        StateMachine<Double, DslState> machine = new StateMachine<>(DslState.STOW, atStateSupplier);
        machine.setAtStateDelay(0.0);
        machine.setStateGraph(createGraph(
                feedAllowed != null ? feedAllowed : () -> true,
                scoreAllowed != null ? scoreAllowed : () -> true));
        // Consume the constructor-seeded initial queue entry.
        machine.update();
        return machine;
    }

    public static void forceScore(StateMachine<Double, DslState> machine) {
        Objects.requireNonNull(machine, "machine");
        machine.force(DslState.SCORE);
    }

    public static void forceUnjam(StateMachine<Double, DslState> machine) {
        Objects.requireNonNull(machine, "machine");
        machine.force(DslState.UNJAM);
    }

    public static Double resolvedSetpoint(DslState state) {
        Objects.requireNonNull(state, "state");
        return StateSeedRuntime.doubleSetpoint(state.seed());
    }

    private static StateSeed<DslState> logicSeed(StateDsl<DslState> dsl) {
        return StateSeed.dsl(dsl);
    }
}
