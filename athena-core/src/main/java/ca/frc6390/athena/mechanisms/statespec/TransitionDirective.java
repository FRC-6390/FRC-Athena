package ca.frc6390.athena.mechanisms.statespec;

public final class TransitionDirective<E extends Enum<E>> {
    public enum Kind {
        STAY,
        TRANSITION
    }

    private static final TransitionDirective<?> STAY = new TransitionDirective<>(Kind.STAY, null);

    private final Kind kind;
    private final E next;

    private TransitionDirective(Kind kind, E next) {
        this.kind = kind;
        this.next = next;
    }

    @SuppressWarnings("unchecked")
    public static <E extends Enum<E>> TransitionDirective<E> stay() {
        return (TransitionDirective<E>) STAY;
    }

    public static <E extends Enum<E>> TransitionDirective<E> to(E next) {
        return new TransitionDirective<>(Kind.TRANSITION, next);
    }

    public Kind kind() {
        return kind;
    }

    public E next() {
        return next;
    }
}
