package ca.frc6390.athena.mechanisms.examples.state;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.statespec.AthenaState;
import ca.frc6390.athena.mechanisms.statespec.AthenaStateLogic;
import ca.frc6390.athena.mechanisms.statespec.StateBuilder;
import ca.frc6390.athena.mechanisms.statespec.StateDsl;
import ca.frc6390.athena.mechanisms.statespec.StateSeed;
import ca.frc6390.athena.mechanisms.statespec.StateSeedProvider;
import ca.frc6390.athena.mechanisms.statespec.StateSeedRuntime;

/**
 * Example usage of StateSeed and StateSeedRuntime helpers.
 */
public final class StateSeedExamples {
    private StateSeedExamples() {}

    @AthenaState(Double.class)
    public enum SeededState implements SetpointProvider<Double>, StateSeedProvider<SeededState> {
        IDLE,
        HOLD,
        TRACK;

        @Override
        public Double getSetpoint() {
            return switch (this) {
                case IDLE -> 0.0;
                case HOLD -> 1.2;
                case TRACK -> 2.0;
            };
        }

        @Override
        public StateSeed<SeededState> seed() {
            return switch (this) {
                case IDLE -> StateSeed.auto();
                case HOLD -> StateSeed.setpoint(1.2);
                case TRACK -> logicSeed(StateSeedExamples::trackLogic);
            };
        }
    }

    @AthenaStateLogic("TRACK")
    public static StateBuilder<SeededState> trackLogic(StateBuilder<SeededState> builder) {
        return builder.setpoint(2.75).then(SeededState.IDLE);
    }

    private static StateSeed<SeededState> logicSeed(StateDsl<SeededState> dsl) {
        return StateSeed.dsl(dsl);
    }

    public static Double resolvedSetpoint(SeededState state) {
        return StateSeedRuntime.doubleSetpoint(state.seed());
    }
}
