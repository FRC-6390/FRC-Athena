package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.ActionContext;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Minimal Athena-owned runtime loop for the new mechanism model.
 */
public final class MechanismRuntime {
    private final Mechanism mechanism;
    private final ActionContext actionContext;
    private final OutputResolver resolver;
    private final OutputApplier applier;
    private final AxisStateSource axisStates;
    private final HookRunner hooks = new HookRunner();
    private final StateScheduler scheduler;
    private final Map<PathRef, PathRuntime> pathRuntimes;
    private Runnable simulationStep = () -> {
    };
    private MechanismState state;
    private double stateStartSeconds = Double.NaN;

    private MechanismRuntime(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver,
            AxisStateSource axisStates,
            Map<PathRef, PathRuntime> pathRuntimes) {
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
        this.resolver = Objects.requireNonNull(resolver, "resolver");
        this.axisStates = Objects.requireNonNull(axisStates, "axisStates");
        this.applier = OutputApplier.using(actionContext);
        this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
        this.scheduler = new StateScheduler(actionContext, pathRuntimes);
        this.state = mechanism.initialState();
    }

    public static MechanismRuntime of(Mechanism mechanism, ActionContext actionContext) {
        return new MechanismRuntime(
                mechanism,
                actionContext,
                OutputResolver.empty(),
                AxisStateSource.from(actionContext),
                new HashMap<>());
    }

    public static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver,
            AxisStateSource axisStates) {
        return new MechanismRuntime(mechanism, actionContext, resolver, axisStates, new HashMap<>());
    }

    public static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver,
            AxisStateSource axisStates,
            Map<PathRef, PathRuntime> pathRuntimes) {
        return new MechanismRuntime(mechanism, actionContext, resolver, axisStates, pathRuntimes);
    }

    public MechanismRuntime path(PathRef ref, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(runtime, "runtime"));
        return this;
    }

    public MechanismRuntime simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        return this;
    }

    public MechanismState state() {
        return state;
    }

    public void set(MechanismState state) {
        this.state = Objects.requireNonNull(state, "state");
        this.stateStartSeconds = Double.NaN;
        this.scheduler.reset();
    }

    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        MechanismContext safeMechanismContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        EventContext safeEventContext = eventContext == null ? EventContext.empty() : eventContext;
        if (Double.isNaN(stateStartSeconds)) {
            stateStartSeconds = safeMechanismContext.nowSeconds();
        }
        MechanismContext timedContext = withTimeInState(
                safeMechanismContext,
                safeMechanismContext.nowSeconds() - stateStartSeconds);
        MechanismState updated = mechanism.update(state, timedContext);
        if (updated != state) {
            state = Objects.requireNonNull(updated, "updated");
            stateStartSeconds = safeMechanismContext.nowSeconds();
            scheduler.reset();
            timedContext = withTimeInState(safeMechanismContext, 0.0);
        }
        StateScheduler.Result active = scheduler.evaluate(state, timedContext);
        List<ResolvedOutput> outputs = resolver.resolve(mechanism, active.state(), active.context(), axisStates);
        applier.applyAll(outputs);
        hooks.run(safeEventContext, actionContext, HookIntrospector.inspect(mechanism).values());
        simulationStep.run();
        return outputs;
    }

    private static MechanismContext withTimeInState(MechanismContext context, double timeInStateSeconds) {
        return new MechanismContext(
                context.nowSeconds(),
                Math.max(0.0, timeInStateSeconds),
                context.dtSeconds(),
                context.enabled(),
                context.autonomous(),
                context.simulation());
    }

    private static final class StateScheduler {
        private final ActionContext actionContext;
        private final Map<PathRef, PathRuntime> pathRuntimes;
        private final Map<String, Node> nodes = new HashMap<>();

        private StateScheduler(ActionContext actionContext, Map<PathRef, PathRuntime> pathRuntimes) {
            this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
            this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
        }

        void reset() {
            nodes.clear();
        }

        Result evaluate(MechanismState state, MechanismContext context) {
            Evaluation evaluation = evaluate(state, "root", context);
            return new Result(evaluation.output() == null ? States.neutral() : evaluation.output(), evaluation.context());
        }

        private Evaluation evaluate(MechanismState state, String path, MechanismContext context) {
            Objects.requireNonNull(state, "state");
            Node node = node(path, context);
            MechanismContext local = withTimeInState(context, context.nowSeconds() - node.startSeconds);

            if (state instanceof States.ChildSet childSet) {
                States.ChildSet output = States.set();
                boolean complete = !childSet.targets().isEmpty();
                int index = 0;
                for (States.ChildTarget target : childSet.targets()) {
                    Evaluation child = evaluate(target.state(), path + ".child" + index, context);
                    if (child.output() != null) {
                        output.set(target.mechanism(), child.output());
                    }
                    complete &= child.complete();
                    index++;
                }
                return new Evaluation(output.targets().isEmpty() ? null : output, complete, local);
            }
            if (state instanceof States.Sequence sequence) {
                return evaluateSequence(sequence, path, context, local, node);
            }
            if (state instanceof States.Cycle cycle) {
                return evaluateCycle(cycle, path, context, node);
            }
            if (state instanceof States.Parallel parallel) {
                return evaluateGroup(parallel.states(), path, context, GroupMode.PARALLEL, -1);
            }
            if (state instanceof States.Race race) {
                return evaluateGroup(race.states(), path, context, GroupMode.RACE, -1);
            }
            if (state instanceof States.Deadline deadline) {
                return evaluateGroup(deadline.states(), path, context, GroupMode.DEADLINE, 0);
            }
            if (state instanceof States.Choice choice) {
                return evaluate(choice.choose(local), path + ".choice", context);
            }
            if (state instanceof States.WhenBranch branch) {
                return evaluate(branch.choose(local), path + ".when", context);
            }
            if (state instanceof States.Timeout timeout) {
                if (timeout.expired(local)) {
                    return new Evaluation(null, true, local);
                }
                Evaluation child = evaluate(timeout.state(), path + ".timeout", context);
                return new Evaluation(child.output(), child.complete(), child.context());
            }
            if (state instanceof States.Conditional conditional) {
                return evaluateConditional(
                        conditional.state(),
                        conditional.condition(),
                        conditional.next(),
                        path,
                        context,
                        local);
            }
            if (state instanceof MechanismState.Conditional conditional) {
                return evaluateConditional(
                        conditional.state(),
                        conditional.condition(),
                        conditional.next(),
                        path,
                        context,
                        local);
            }
            if (state instanceof States.Then then) {
                return evaluateThen(then.state(), then.next(), path, context);
            }
            if (state instanceof MechanismState.Then then) {
                return evaluateThen(then.state(), then.next(), path, context);
            }
            if (state instanceof States.Clamped clamped) {
                Evaluation child = evaluate(clamped.state(), path + ".clamped", context);
                return new Evaluation(child.output() == null ? null : States.clamp(child.output(), clamped.range()),
                        child.complete(), child.context());
            }
            if (state instanceof MechanismState.Clamped clamped) {
                Evaluation child = evaluate(clamped.state(), path + ".clamped", context);
                return new Evaluation(child.output() == null ? null : States.clamp(child.output(), clamped.range()),
                        child.complete(), child.context());
            }
            if (state instanceof States.Action action) {
                if (!node.entered) {
                    action.action().apply(actionContext);
                    node.entered = true;
                }
                return new Evaluation(null, true, local);
            }
            if (state instanceof States.DoOnce action) {
                if (!node.entered) {
                    action.action().run();
                    node.entered = true;
                }
                return new Evaluation(null, true, local);
            }
            if (state instanceof States.WaitSeconds wait) {
                return new Evaluation(null, wait.complete(local), local);
            }
            if (state instanceof States.WaitUntil wait) {
                return new Evaluation(null, wait.complete(local), local);
            }
            if (state instanceof PathRef pathRef) {
                PathRuntime runtime = pathRuntimes.get(pathRef);
                if (runtime == null) {
                    throw new IllegalStateException("No PathRuntime bound for path " + pathRef.key());
                }
                if (!node.entered) {
                    runtime.initialize(pathRef, local);
                    node.entered = true;
                }
                runtime.execute(pathRef, local);
                if (runtime.isFinished(pathRef, local)) {
                    runtime.end(pathRef, local, false);
                    return new Evaluation(null, true, local);
                }
                return new Evaluation(null, false, local);
            }
            return new Evaluation(state, false, local);
        }

        private Evaluation evaluateSequence(
                States.Sequence sequence,
                String path,
                MechanismContext context,
                MechanismContext local,
                Node node) {
            if (local.timeInStateSeconds() >= sequence.timeoutSeconds()) {
                return sequence.next() == null
                        ? new Evaluation(null, true, local)
                        : evaluate(sequence.next(), path + ".next", context);
            }
            List<States.SequenceStep> steps = sequence.steps();
            while (node.index < steps.size()) {
                States.SequenceStep step = steps.get(node.index);
                String stepPath = path + ".step" + node.index;
                Evaluation child = evaluate(step.state(), stepPath, context);
                if (child.complete() || step.complete().test(child.context())) {
                    clear(stepPath);
                    node.index++;
                    continue;
                }
                return child;
            }
            if (sequence.next() != null) {
                return evaluate(sequence.next(), path + ".next", context);
            }
            return new Evaluation(null, true, local);
        }

        private Evaluation evaluateCycle(
                States.Cycle cycle,
                String path,
                MechanismContext context,
                Node node) {
            List<States.CycleStep> steps = cycle.steps();
            if (steps.isEmpty()) {
                return new Evaluation(null, false, context);
            }
            if (node.index >= steps.size()) {
                node.index = 0;
            }
            for (int attempts = 0; attempts < steps.size(); attempts++) {
                States.CycleStep step = steps.get(node.index);
                String stepPath = path + ".cycle" + node.index;
                Evaluation child = evaluate(step.state(), stepPath, context);
                if (child.complete() || step.advance().test(child.context())) {
                    clear(stepPath);
                    node.index = (node.index + 1) % steps.size();
                    continue;
                }
                return child;
            }
            return new Evaluation(null, false, context);
        }

        private Evaluation evaluateGroup(
                List<MechanismState> states,
                String path,
                MechanismContext context,
                GroupMode mode,
                int deadlineIndex) {
            List<MechanismState> outputs = new ArrayList<>();
            boolean anyComplete = false;
            boolean allComplete = !states.isEmpty();
            for (int i = 0; i < states.size(); i++) {
                Evaluation child = evaluate(states.get(i), path + ".group" + i, context);
                anyComplete |= child.complete();
                allComplete &= child.complete();
                if (mode == GroupMode.DEADLINE && i == deadlineIndex && child.complete()) {
                    return new Evaluation(null, true, child.context());
                }
                if (child.output() != null) {
                    outputs.add(child.output());
                }
            }
            if (mode == GroupMode.RACE && anyComplete) {
                return new Evaluation(null, true, context);
            }
            if (mode == GroupMode.PARALLEL && allComplete) {
                return new Evaluation(null, true, context);
            }
            return new Evaluation(groupOutput(outputs), false, context);
        }

        private Evaluation evaluateConditional(
                MechanismState wrapped,
                StateCondition condition,
                MechanismState next,
                String path,
                MechanismContext context,
                MechanismContext local) {
            if (condition.test(local)) {
                return next == null ? new Evaluation(null, true, local) : evaluate(next, path + ".next", context);
            }
            Evaluation child = evaluate(wrapped, path + ".conditional", context);
            if (child.complete()) {
                return next == null ? child : evaluate(next, path + ".next", context);
            }
            return child;
        }

        private Evaluation evaluateThen(
                MechanismState wrapped,
                MechanismState next,
                String path,
                MechanismContext context) {
            Evaluation child = evaluate(wrapped, path + ".thenState", context);
            if (child.complete()) {
                return evaluate(next, path + ".thenNext", context);
            }
            return child;
        }

        private Node node(String path, MechanismContext context) {
            return nodes.computeIfAbsent(path, ignored -> new Node(context.nowSeconds()));
        }

        private void clear(String path) {
            Set<String> remove = new HashSet<>();
            for (String key : nodes.keySet()) {
                if (key.equals(path) || key.startsWith(path + ".")) {
                    remove.add(key);
                }
            }
            for (String key : remove) {
                nodes.remove(key);
            }
        }

        private static MechanismState groupOutput(List<MechanismState> states) {
            if (states.isEmpty()) {
                return null;
            }
            if (states.size() == 1) {
                return states.get(0);
            }
            return new States.Parallel(states);
        }

        private enum GroupMode {
            PARALLEL,
            RACE,
            DEADLINE
        }

        private record Evaluation(MechanismState output, boolean complete, MechanismContext context) {
        }

        private record Result(MechanismState state, MechanismContext context) {
        }

        private static final class Node {
            private final double startSeconds;
            private boolean entered;
            private int index;

            private Node(double startSeconds) {
                this.startSeconds = startSeconds;
            }
        }
    }
}
