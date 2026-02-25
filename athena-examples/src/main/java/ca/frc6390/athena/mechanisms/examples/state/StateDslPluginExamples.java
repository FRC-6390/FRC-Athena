package ca.frc6390.athena.mechanisms.examples.state;

import java.util.Objects;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateGraph;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.statespec.AthenaState;
import ca.frc6390.athena.mechanisms.statespec.AthenaStateLogic;
import ca.frc6390.athena.mechanisms.statespec.StateBuilder;
import ca.frc6390.athena.mechanisms.statespec.StateDsl;
import ca.frc6390.athena.mechanisms.statespec.StateSeed;
import ca.frc6390.athena.mechanisms.statespec.StateSeedProvider;
import ca.frc6390.athena.mechanisms.statespec.StateSeedRuntime;

/**
 * Plugin-first state example that combines:
 * - {@link AthenaState} enum annotation
 * - {@link AthenaStateLogic}-tagged DSL methods
 * - guarded {@link StateGraph} path expansion
 * - DSL-driven auto transition chaining ({@code until(...).then(...)}).
 *
 * <p>This file keeps explicit fallback methods so it can compile even if the
 * Athena compiler plugin is not enabled for the module.</p>
 */
public final class StateDslPluginExamples {
    private StateDslPluginExamples() {}

    @AthenaState(Double.class)
    public enum PluginState implements SetpointProvider<Double>, StateSeedProvider<PluginState> {
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
        public StateSeed<PluginState> seed() {
            return switch (this) {
                case STOW -> StateSeed.auto();
                case FEED -> logicSeed(StateDslPluginExamples::feedLogic);
                case LATCH -> StateSeed.setpoint(0.2);
                case UNJAM -> logicSeed(StateDslPluginExamples::unjamLogic);
                case SCORE -> logicSeed(StateDslPluginExamples::scoreLogic);
            };
        }
    }

    @AthenaStateLogic("FEED")
    public static StateBuilder<PluginState> feedLogic(StateBuilder<PluginState> builder) {
        return builder
                .setpoint(0.45)
                .manualPercent(0.35)
                .until(ctx -> ctx.timeInState() >= 0.0)
                .then(PluginState.LATCH);
    }

    @AthenaStateLogic("UNJAM")
    public static StateBuilder<PluginState> unjamLogic(StateBuilder<PluginState> builder) {
        return builder
                .setpoint(-0.25)
                .manualPercent(-0.4)
                .until(ctx -> ctx.timeInState() >= 0.0)
                .then(PluginState.FEED);
    }

    @AthenaStateLogic("SCORE")
    public static StateBuilder<PluginState> scoreLogic(StateBuilder<PluginState> builder) {
        return builder
                .setpoint(0.85)
                .until(ctx -> ctx.timeInState() >= 0.0)
                .then(PluginState.STOW);
    }

    public static StateGraph<PluginState> createGraph(
            BooleanSupplier feedAllowed,
            BooleanSupplier scoreAllowed) {
        Objects.requireNonNull(feedAllowed, "feedAllowed");
        Objects.requireNonNull(scoreAllowed, "scoreAllowed");

        return StateGraph
                .create(PluginState.class)
                .path(PluginState.STOW, PluginState.FEED, PluginState.LATCH, PluginState.SCORE)
                .path(PluginState.SCORE, PluginState.LATCH, PluginState.FEED, PluginState.STOW)
                .guard(PluginState.STOW, PluginState.FEED, feedAllowed)
                .guard(PluginState.LATCH, PluginState.SCORE, scoreAllowed);
    }

    public static StateMachine<Double, PluginState> createMachine(
            BooleanSupplier atStateSupplier,
            BooleanSupplier feedAllowed,
            BooleanSupplier scoreAllowed) {
        Objects.requireNonNull(atStateSupplier, "atStateSupplier");

        StateMachine<Double, PluginState> machine = new StateMachine<>(PluginState.STOW, atStateSupplier);
        machine.setAtStateDelay(0.0);
        machine.setStateGraph(createGraph(
                feedAllowed != null ? feedAllowed : () -> true,
                scoreAllowed != null ? scoreAllowed : () -> true));
        // Consume the constructor-seeded initial queue entry.
        machine.update();
        return machine;
    }

    public static void forceScore(StateMachine<Double, PluginState> machine) {
        Objects.requireNonNull(machine, "machine");
        machine.force(PluginState.SCORE);
    }

    public static void forceUnjam(StateMachine<Double, PluginState> machine) {
        Objects.requireNonNull(machine, "machine");
        machine.force(PluginState.UNJAM);
    }

    public static Double resolvedSetpoint(PluginState state) {
        Objects.requireNonNull(state, "state");
        return StateSeedRuntime.doubleSetpoint(state.seed());
    }

    private static StateSeed<PluginState> logicSeed(StateDsl<PluginState> dsl) {
        return StateSeed.dsl(dsl);
    }
}
