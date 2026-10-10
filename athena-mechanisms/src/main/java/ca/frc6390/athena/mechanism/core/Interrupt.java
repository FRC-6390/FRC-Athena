package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Policy of a <em>running</em> root Action: whether a newer request for the same resources may
 * replace it.
 *
 * <ul>
 *   <li>{@link #ANYTIME}: newer requests take over immediately (default).</li>
 *   <li>{@link #AFTER_COMPLETE}: newer requests for overlapping resources are held pending until
 *       the running Action completes.</li>
 *   <li>{@link #after(BooleanSupplier)}: held until the supplier returns true.</li>
 * </ul>
 *
 * <p>{@link MechanismScheduler#cancel(Action)}, {@link Actions#abort(ca.frc6390.athena.mechanism.core.Mechanism)}
 * and robot disable always override this policy. Non-ANYTIME policies require a bounded Action
 * (see {@link Actions#isBounded(Action)}).</p>
 */
public final class Interrupt {
    /** Newer requests interrupt at any time. */
    public static final Interrupt ANYTIME = new Interrupt(Kind.ANYTIME, null);
    /** Newer requests wait until this Action completes. */
    public static final Interrupt AFTER_COMPLETE = new Interrupt(Kind.AFTER_COMPLETE, null);

    private final Kind kind;
    private final BooleanSupplier allowed;

    private Interrupt(Kind kind, BooleanSupplier allowed) {
        this.kind = kind;
        this.allowed = allowed;
    }

    /** Newer requests wait until {@code allowed} returns true. */
    public static Interrupt after(BooleanSupplier allowed) {
        return new Interrupt(Kind.CONDITION, Objects.requireNonNull(allowed, "allowed"));
    }

    /** Returns true when this policy requires the running Action to finish or allow it. */
    public boolean restrictive() {
        return kind != Kind.ANYTIME;
    }

    /** Returns whether a newer request may replace the running Action now. */
    public boolean allowsInterrupt(boolean complete) {
        return switch (kind) {
            case ANYTIME -> true;
            case AFTER_COMPLETE -> complete;
            case CONDITION -> allowed.getAsBoolean();
        };
    }

    @Override
    public String toString() {
        return switch (kind) {
            case ANYTIME -> "Interrupt.ANYTIME";
            case AFTER_COMPLETE -> "Interrupt.AFTER_COMPLETE";
            case CONDITION -> "Interrupt.after(...)";
        };
    }

    private enum Kind { ANYTIME, AFTER_COMPLETE, CONDITION }
}
