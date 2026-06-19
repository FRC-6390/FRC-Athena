package ca.frc6390.athena.mechanisms.examples.state;

import ca.frc6390.athena.mechanisms.statespec.StateId;
import ca.frc6390.athena.mechanisms.statespec.StateSeedRuntime;
import ca.frc6390.athena.mechanisms.statespec.StateSet;

/**
 * Plain-Java state declarations that do not require the Athena state DSL plugin.
 */
public final class StateSetExamples {
    private StateSetExamples() {
    }

    public static final class IntakeArmStates extends StateSet {
        public static final StateId Stow = state("Stow")
            .manualPercent(0.5)
            .until(ctx -> ctx.limitSwitch(0))
            .then("Out");

        public static final StateId StowPID = state("StowPID", 0.0);
        public static final StateId Out = state("Out", -42.33);
    }

    public static Double resolvedSetpoint(StateId state) {
        return StateSeedRuntime.doubleSetpoint(state.seed());
    }
}
