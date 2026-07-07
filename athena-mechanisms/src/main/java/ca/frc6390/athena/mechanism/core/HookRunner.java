package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionContext;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Executes hooks against sampled event state.
 */
public final class HookRunner {
    private final Map<HookRef, Boolean> previousEventActive = new IdentityHashMap<>();
    private final Map<EventRef, Boolean> previousSourceActive = new IdentityHashMap<>();

    /**
     * Runs all hooks for one runtime tick.
     *
     * @param context event context
     * @param actionContext runtime action context
     * @param hooks hooks to evaluate
     */
    public void run(EventContext context, ActionContext actionContext, Collection<HookRef> hooks) {
        Objects.requireNonNull(actionContext, "actionContext");
        Objects.requireNonNull(hooks, "hooks");
        EventContext safeContext = context == null ? EventContext.empty() : context;
        for (HookRef hook : hooks) {
            run(safeContext, actionContext, hook);
        }
    }

    /**
     * Runs one hook for one runtime tick.
     *
     * @param context event context
     * @param actionContext runtime action context
     * @param hook hook to evaluate
     */
    public void run(EventContext context, ActionContext actionContext, HookRef hook) {
        Objects.requireNonNull(actionContext, "actionContext");
        Objects.requireNonNull(hook, "hook");
        EventContext safeContext = context == null ? EventContext.empty() : context;
        EventRef event = hook.event();
        boolean previousSource = previousSourceActive.getOrDefault(event, false);
        boolean active = event.active(safeContext, previousSource);
        boolean wasActive = previousEventActive.getOrDefault(hook, false);

        for (HookRef.HookBinding binding : hook.bindings()) {
            if (binding.trigger().shouldRun(event, wasActive, active)) {
                binding.action().apply(actionContext);
            }
        }

        previousSourceActive.put(event, event.sourceActive(safeContext));
        previousEventActive.put(hook, active);
    }

    /**
     * Clears remembered event state.
     */
    public void reset() {
        previousEventActive.clear();
        previousSourceActive.clear();
    }
}
