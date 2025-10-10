package ca.frc6390.athena.mechanisms.superstructure;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.util.SupplierUtil;

/**
 * Runtime view of the superstructure context. In addition to mechanism lookups it exposes helpers for
 * queueing commands and deferring side effects until later in the update loop.
 */
public final class SuperstructureRuntime extends SuperstructureContext {

    private final Deque<Runnable> deferredActions = new ArrayDeque<>();

    SuperstructureRuntime(Map<String, SuperstructureMechanismBinding<?>> mechanisms) {
        mechanisms.forEach((key, binding) -> registerMechanism(key, binding));
    }

    public void runDeferred(Runnable action) {
        Objects.requireNonNull(action);
        deferredActions.addLast(action);
    }

    public void flushDeferred() {
        while (!deferredActions.isEmpty()) {
            deferredActions.removeFirst().run();
        }
    }

    public void command(String key, Enum<?> state) {
        command(key, state, () -> true);
    }

    public void command(String key, Enum<?> state, BooleanSupplier condition) {
        mechanism(key).queueRaw(state, SupplierUtil.wrapBoolean(condition, true));
    }
}
