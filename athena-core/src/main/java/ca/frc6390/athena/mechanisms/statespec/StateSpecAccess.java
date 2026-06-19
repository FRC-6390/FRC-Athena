package ca.frc6390.athena.mechanisms.statespec;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Runtime bridge for Athena state objects.
 */
public final class StateSpecAccess {
    private StateSpecAccess() {}

    @SuppressWarnings("unchecked")
    public static <T> T setpoint(Object state) {
        if (state == null) {
            return null;
        }
        if (state instanceof SetpointProvider<?> provider) {
            return (T) provider.getSetpoint();
        }
        StateSeed<?> seed = seed(state);
        if (seed != null && seed.kind() == StateSeed.Kind.SETPOINT && Double.isFinite(seed.setpoint())) {
            return (T) Double.valueOf(seed.setpoint());
        }
        return null;
    }

    @SuppressWarnings("unchecked")
    public static <E> StateSeed<E> seed(E state) {
        if (state == null) {
            return null;
        }
        if (state instanceof StateSeedProvider<?> provider) {
            return (StateSeed<E>) provider.seed();
        }
        return null;
    }

    @SuppressWarnings("unchecked")
    public static <E> E resolve(E context, String stateName) {
        if (context == null || stateName == null || stateName.isBlank()) {
            return null;
        }
        if (context instanceof StateId stateId) {
            return (E) stateId.owner().get(stateName);
        }
        if (!(context instanceof Enum<?> enumContext)) {
            return null;
        }
        Class<?> declaringClass = enumContext.getDeclaringClass();
        if (!Enum.class.isAssignableFrom(declaringClass)) {
            return null;
        }
        return (E) Enum.valueOf((Class<? extends Enum>) declaringClass.asSubclass(Enum.class), stateName);
    }
}
