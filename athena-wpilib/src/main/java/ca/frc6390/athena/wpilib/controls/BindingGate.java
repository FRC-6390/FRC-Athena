package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.HookBinding;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Condition that gates every signal created on a binding-set view (see {@code Gamepad.in}).
 *
 * <p>Gates nest: a gate created inside another gate's view is active only while both
 * conditions are true. Both conditions are always sampled, so stateful conditions keep
 * advancing even while another condition is false.
 */
final class BindingGate {
    private final BindingGate parent;
    private final BooleanSupplier condition;

    private BindingGate(BindingGate parent, BooleanSupplier condition) {
        this.parent = parent;
        this.condition = Objects.requireNonNull(condition, "condition");
    }

    /** Creates a gate nested inside {@code parent}, which may be {@code null} for a top-level gate. */
    static BindingGate of(BindingGate parent, BooleanSupplier condition) {
        return new BindingGate(parent, condition);
    }

    boolean allows(EventContext context) {
        boolean outer = parent == null || parent.allows(context);
        boolean inner = ControlSignal.value(condition, context);
        return outer && inner;
    }

    /**
     * Wraps a raw evaluator so that its result is only true while the gate allows it. When a
     * conflict key is supplied, edges fired by the gated signal are reported to the key so that
     * simultaneous gated bindings on one physical input can be detected.
     */
    static final class Gated implements ControlSignal.Evaluator {
        private final BindingGate gate;
        private final BindingConflicts.Key key;
        private final ControlSignal.Evaluator source;
        private ControlSignal signal;
        private boolean previous;

        Gated(BindingGate gate, BindingConflicts.Key key, ControlSignal.Evaluator source) {
            this.gate = Objects.requireNonNull(gate, "gate");
            this.key = key;
            this.source = Objects.requireNonNull(source, "source");
        }

        void bind(ControlSignal signal) {
            this.signal = signal;
        }

        @Override
        public boolean evaluate(EventContext context) {
            boolean allowed = gate.allows(context);
            boolean raw = source.evaluate(context);
            boolean active = allowed && raw;
            if (key != null && signal != null) {
                if (active && !previous) {
                    observe(BindingConflicts.Edge.PRESS, context);
                }
                if (active) {
                    observe(BindingConflicts.Edge.HELD, context);
                } else if (previous) {
                    observe(BindingConflicts.Edge.RELEASE, context);
                }
            }
            previous = active;
            return active;
        }

        private void observe(BindingConflicts.Edge edge, EventContext context) {
            if (hasAction(edge)) {
                key.observe(edge, signal, context);
            }
        }

        private boolean hasAction(BindingConflicts.Edge edge) {
            HookBinding binding = signal.binding();
            List<HookBinding.HookAction> actions = binding.actions();
            for (int index = 0; index < actions.size(); index++) {
                if (actions.get(index).phase() == edge.phase()) {
                    return true;
                }
            }
            return false;
        }
    }
}
