package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Executes hooks against sampled event Action.
 */
final class HookRuntime {
    private final Map<HookBinding, Boolean> previousEventActive = new IdentityHashMap<>();
    private final Map<EventBinding, Boolean> previousSourceActive = new IdentityHashMap<>();

    /**
     * Runs all hooks for one runtime tick.
     *
     * @param context event context
     * @param actionContext runtime action context
     * @param hooks hooks to evaluate
     */
    public void run(EventContext context, ActionContext actionContext, Collection<HookBinding> hooks) {
        Objects.requireNonNull(actionContext, "actionContext");
        Objects.requireNonNull(hooks, "hooks");
        EventContext safeContext = context == null ? EventContext.empty() : context;
        Map<EventBinding, Boolean> consumedEvents = new IdentityHashMap<>();
        Map<EventBinding, Boolean> sourceSamples = new IdentityHashMap<>();
        for (HookBinding hook : hooks) {
            runOne(safeContext, actionContext, hook, sourceSamples);
            consumedEvents.put(hook.event(), Boolean.TRUE);
        }
        for (EventBinding event : consumedEvents.keySet()) {
            previousSourceActive.put(event, sourceSamples.getOrDefault(event, false));
            event.afterRun(safeContext);
        }
    }

    /**
     * Runs one hook for one runtime tick.
     *
     * @param context event context
     * @param actionContext runtime action context
     * @param hook hook to evaluate
     */
    public void run(EventContext context, ActionContext actionContext, HookBinding hook) {
        Objects.requireNonNull(actionContext, "actionContext");
        Objects.requireNonNull(hook, "hook");
        EventContext safeContext = context == null ? EventContext.empty() : context;
        Map<EventBinding, Boolean> sourceSamples = new IdentityHashMap<>();
        runOne(safeContext, actionContext, hook, sourceSamples);
        previousSourceActive.put(hook.event(), sourceSamples.getOrDefault(hook.event(), false));
        hook.event().afterRun(safeContext);
    }

    private void runOne(
            EventContext safeContext,
            ActionContext actionContext,
            HookBinding hook,
            Map<EventBinding, Boolean> sourceSamples) {
        EventBinding event = hook.event();
        boolean previousSource = previousSourceActive.getOrDefault(event, false);
        boolean currentSource = sourceSamples.computeIfAbsent(event, value -> value.sourceActive(safeContext));
        boolean active = event.active(safeContext, previousSource, currentSource);
        boolean wasActive = previousEventActive.getOrDefault(hook, false);

        for (HookBinding.HookAction binding : hook.actions()) {
            if (binding.phase().shouldRun(event, wasActive, active)) {
                binding.action().apply(actionContext);
            }
        }

        previousEventActive.put(hook, active);
    }

    /**
     * Clears remembered event Action.
     */
    public void reset() {
        previousEventActive.clear();
        previousSourceActive.clear();
    }
}
