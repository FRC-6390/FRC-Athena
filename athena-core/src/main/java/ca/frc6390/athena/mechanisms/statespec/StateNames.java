package ca.frc6390.athena.mechanisms.statespec;

public final class StateNames {
    private StateNames() {
    }

    public static String name(Object state) {
        if (state == null) {
            return null;
        }
        if (state instanceof StateId stateId) {
            return stateId.name();
        }
        if (state instanceof Enum<?> enumState) {
            return enumState.name();
        }
        return state.toString();
    }
}
