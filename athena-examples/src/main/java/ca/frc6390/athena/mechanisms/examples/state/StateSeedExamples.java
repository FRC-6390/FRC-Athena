package ca.frc6390.athena.mechanisms.examples.state;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.statespec.StateSeed;
import ca.frc6390.athena.mechanisms.statespec.StateSeedProvider;
import ca.frc6390.athena.mechanisms.statespec.StateSeedRuntime;

/**
 * Example usage of StateSeed and StateSeedRuntime helpers.
 */
public final class StateSeedExamples {
    private StateSeedExamples() {}

    public enum SeededState implements SetpointProvider<Double>, StateSeedProvider<SeededState> {
        IDLE(0.0, StateSeed.auto()),
        HOLD(1.2, StateSeed.setpoint(1.2)),
        TRACK(2.0, StateSeed.dsl(b -> b.setpoint(2.75).then(IDLE)));

        private final double setpoint;
        private final StateSeed<SeededState> seed;

        SeededState(double setpoint, StateSeed<SeededState> seed) {
            this.setpoint = setpoint;
            this.seed = seed;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }

        @Override
        public StateSeed<SeededState> seed() {
            return seed;
        }
    }

    public static Double resolvedSetpoint(SeededState state) {
        return StateSeedRuntime.doubleSetpoint(state.seed());
    }
}
