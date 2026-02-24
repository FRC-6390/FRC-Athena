package ca.frc6390.athena.mechanisms.statespec;

import ca.frc6390.athena.mechanisms.Mechanism;

public interface StateCtx<E extends Enum<E>> {
    E state();

    default double timeInState() {
        return 0.0;
    }

    default Mechanism mechanism() {
        return null;
    }

    default boolean limitSwitch(int index) {
        if (index < 0) {
            return false;
        }
        Mechanism mechanism = mechanism();
        if (mechanism == null) {
            return false;
        }
        var limitSwitches = mechanism.limitSwitches();
        if (limitSwitches == null || index >= limitSwitches.length || limitSwitches[index] == null) {
            return false;
        }
        return limitSwitches[index].getAsBoolean();
    }

    default boolean stalled() {
        Mechanism mechanism = mechanism();
        return mechanism != null && mechanism.motors().device().isStalling();
    }
}
