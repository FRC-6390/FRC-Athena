package ca.frc6390.athena.mechanisms.statespec;

public final class StateSeedRuntime {
    private StateSeedRuntime() {
    }

    public static <E extends Enum<E>> Double doubleSetpoint(StateSeed<E> seed) {
        if (seed == null) {
            return null;
        }
        return switch (seed.kind()) {
            case AUTO -> null;
            case SETPOINT -> seed.setpoint();
            case DSL -> {
                StateDsl<E> dsl = seed.dsl();
                if (dsl == null) {
                    yield null;
                }
                StateBuilder<E> builder = new StateBuilder<>();
                StateBuilder<E> applied = dsl.apply(builder);
                StateBuilder<E> resolved = applied != null ? applied : builder;
                yield resolved.setpoint();
            }
        };
    }
}
