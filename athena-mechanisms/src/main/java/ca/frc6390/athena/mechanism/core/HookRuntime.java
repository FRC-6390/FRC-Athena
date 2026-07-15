package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Executes hooks against sampled event Action.
 */
final class HookRuntime {
    interface LeaseController {
        void activate(Object key, Action action, boolean restart);

        void release(Object key);
    }

    private final LeaseController leases;
    private final Runnable immediateMutation;
    private final Map<HookBinding, Boolean> previousEventActive = new IdentityHashMap<>();
    private final Map<EventBinding, Boolean> previousSourceActive = new IdentityHashMap<>();
    private final Map<EventBinding, Boolean> consumedEvents = new IdentityHashMap<>();
    private final Map<EventBinding, Boolean> sourceSamples = new IdentityHashMap<>();
    private final Map<HookBinding, Boolean> triggeredThisCycle = new IdentityHashMap<>();

    HookRuntime() {
        this(null, null);
    }

    HookRuntime(LeaseController leases) {
        this(leases, null);
    }

    HookRuntime(LeaseController leases, Runnable immediateMutation) {
        this.leases = leases;
        this.immediateMutation = immediateMutation == null ? () -> { } : immediateMutation;
    }

    /**
     * Runs all hooks for one runtime tick.
     *
     * @param context event context
     * @param actionContext runtime action context
     * @param hooks hooks to evaluate
     */
    public void run(EventContext context, ActionContext actionContext, Collection<HookBinding> hooks) {
        run(context, actionContext, hooks, true);
    }

    void run(
            EventContext context,
            ActionContext actionContext,
            Collection<HookBinding> hooks,
            boolean finishEvents) {
        Objects.requireNonNull(actionContext, "actionContext");
        Objects.requireNonNull(hooks, "hooks");
        EventContext safeContext = context == null ? EventContext.empty() : context;
        consumedEvents.clear();
        sourceSamples.clear();
        triggeredThisCycle.clear();
        for (HookBinding hook : hooks) {
            runOne(safeContext, actionContext, hook, sourceSamples);
            consumedEvents.put(hook.event(), Boolean.TRUE);
        }
        for (EventBinding event : consumedEvents.keySet()) {
            previousSourceActive.put(event, sourceSamples.getOrDefault(event, false));
            if (finishEvents) {
                event.afterRun(safeContext);
            }
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
        sourceSamples.clear();
        runOne(safeContext, actionContext, hook, sourceSamples);
        previousSourceActive.put(hook.event(), sourceSamples.getOrDefault(hook.event(), false));
        hook.event().afterRun(safeContext);
        sourceSamples.clear();
    }

    private void runOne(
            EventContext safeContext,
            ActionContext actionContext,
            HookBinding hook,
            Map<EventBinding, Boolean> sourceSamples) {
        if (hook.isDisabled()) {
            return;
        }
        EventBinding event = hook.event();
        boolean previousSource = previousSourceActive.getOrDefault(event, false);
        boolean currentSource = sourceSamples.computeIfAbsent(event, value -> value.sourceActive(safeContext));
        boolean active = event.active(safeContext, previousSource, currentSource);
        boolean wasActive = previousEventActive.getOrDefault(hook, false);
        triggeredThisCycle.put(hook, active != wasActive);

        for (HookBinding.HookAction binding : hook.actions()) {
            if (isImmediateDeviceMutation(binding.action())) {
                if (binding.phase().shouldRun(event, wasActive, active)) {
                    binding.action().apply(actionContext);
                    immediateMutation.run();
                }
                continue;
            }
            if (leases != null && binding.action() instanceof Action action) {
                runLeasedAction(event, binding, action, wasActive, active);
                continue;
            }
            if (leases != null && isHeldPhase(binding.phase())) {
                runCapturedBinding(event, binding, actionContext, wasActive, active);
                continue;
            }
            if (binding.phase().shouldRun(event, wasActive, active)) {
                binding.action().apply(actionContext);
            }
        }

        previousEventActive.put(hook, active);
    }

    HookStatus status(HookBinding hook) {
        Objects.requireNonNull(hook, "hook");
        return new HookStatus(
                previousSourceActive.getOrDefault(hook.event(), false),
                previousEventActive.getOrDefault(hook, false),
                triggeredThisCycle.getOrDefault(hook, false));
    }

    private void runLeasedAction(
            EventBinding event,
            HookBinding.HookAction binding,
            Action action,
            boolean wasActive,
            boolean active) {
        boolean shouldRun = binding.phase().shouldRun(event, wasActive, active);
        if (isHeldPhase(binding.phase())) {
            boolean held = binding.phase() == HookBinding.Phase.WHILE_ACTIVE ? active : !active;
            if (held) {
                leases.activate(binding, action, false);
            } else {
                leases.release(binding);
            }
        } else if (shouldRun) {
            leases.activate(binding, action, true);
        }
    }

    private void runCapturedBinding(
            EventBinding event,
            HookBinding.HookAction binding,
            ActionContext actionContext,
            boolean wasActive,
            boolean active) {
        boolean held = binding.phase() == HookBinding.Phase.WHILE_ACTIVE ? active : !active;
        boolean wasHeld = binding.phase() == HookBinding.Phase.WHILE_ACTIVE ? wasActive : !wasActive;
        if (held) {
            List<Action> captured = ActionRequests.capture(() -> binding.action().apply(actionContext));
            if (!captured.isEmpty()) {
                Action action = captured.size() == 1
                        ? captured.get(0)
                        : Actions.parallel(captured.toArray(Action[]::new));
                leases.activate(binding, action, false);
            }
        } else if (wasHeld) {
            leases.release(binding);
        }
    }

    private static boolean isHeldPhase(HookBinding.Phase phase) {
        return phase == HookBinding.Phase.WHILE_ACTIVE || phase == HookBinding.Phase.WHILE_INACTIVE;
    }

    private static boolean isImmediateDeviceMutation(ActionBinding action) {
        return action instanceof Actions.EncoderSetPosition || action instanceof Actions.ImuSetYaw;
    }

    /**
     * Clears remembered event Action.
     */
    public void reset() {
        previousEventActive.clear();
        previousSourceActive.clear();
        consumedEvents.clear();
        sourceSamples.clear();
        triggeredThisCycle.clear();
    }

    record HookStatus(boolean sourceActive, boolean active, boolean triggeredThisCycle) {
    }
}
