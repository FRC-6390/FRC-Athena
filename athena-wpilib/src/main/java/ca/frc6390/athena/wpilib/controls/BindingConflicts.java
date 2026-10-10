package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.HookBinding;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;

/**
 * Detects gated bindings on one physical input that fire the same edge in the same runtime cycle.
 *
 * <p>Detection is best effort: it observes edges while the runtime samples the gated signals. Each
 * input and edge reports at most once per {@link #REPORT_INTERVAL_SECONDS}.
 */
final class BindingConflicts {
    static final double REPORT_INTERVAL_SECONDS = 5.0;

    /** Edge types that can conflict. Each maps to the lifecycle phase its action runs in. */
    enum Edge {
        PRESS(HookBinding.Phase.ON_START),
        RELEASE(HookBinding.Phase.ON_END),
        HELD(HookBinding.Phase.WHILE_ACTIVE);

        private final HookBinding.Phase phase;

        Edge(HookBinding.Phase phase) {
            this.phase = phase;
        }

        HookBinding.Phase phase() {
            return phase;
        }
    }

    private final Consumer<String> reporter;
    private final Map<String, Key> keys = new HashMap<>();

    BindingConflicts(Consumer<String> reporter) {
        this.reporter = Objects.requireNonNull(reporter, "reporter");
    }

    /** Returns the shared key for a physical input name. */
    Key key(String inputName) {
        return keys.computeIfAbsent(Objects.requireNonNull(inputName, "inputName"), Key::new);
    }

    /** One physical input, tracking the most recent gated signal to fire per edge. */
    final class Key {
        private final String inputName;
        private final Slot[] slots = new Slot[Edge.values().length];

        private Key(String inputName) {
            this.inputName = inputName;
            for (Edge edge : Edge.values()) {
                slots[edge.ordinal()] = new Slot();
            }
        }

        void observe(Edge edge, ControlSignal signal, EventContext context) {
            Slot slot = slots[edge.ordinal()];
            if (slot.context != context) {
                slot.context = context;
                slot.signal = signal;
                return;
            }
            if (slot.signal == signal) {
                return;
            }
            double now = context.nowSeconds();
            if (now - slot.lastReported < REPORT_INTERVAL_SECONDS) {
                return;
            }
            slot.lastReported = now;
            reporter.accept("Gated bindings on '" + inputName + "' fire on the same " + edge
                    + " edge in the same cycle: '" + slot.signal.name() + "' and '" + signal.name()
                    + "'. Make the conditions exclusive or move one binding out of its set.");
        }
    }

    private static final class Slot {
        private EventContext context;
        private ControlSignal signal;
        private double lastReported = Double.NEGATIVE_INFINITY;

    }
}
