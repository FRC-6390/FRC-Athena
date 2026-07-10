package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Minimal Athena-owned runtime loop for the new mechanism model.
 */
final class MechanismRuntime {
    private final Mechanism mechanism;
    private final MechanismNode node;
    private final ActionContext actionContext;
    private final OutputResolver resolver;
    private final OutputApplier applier;
    private final HookRuntime hookRuntime = new HookRuntime();
    private final List<HookBinding> hookBindings;
    private final StateScheduler scheduler;
    private final Map<PathAction, PathRuntime> pathRuntimes;
    private Runnable simulationStep = () -> {
    };
    private Action action;
    private double stateStartSeconds = Double.NaN;

    private MechanismRuntime(
            MechanismNode node,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathAction, PathRuntime> pathRuntimes) {
        this.node = Objects.requireNonNull(node, "node");
        this.mechanism = Objects.requireNonNull(node.mechanism(), "mechanism");
        this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
        this.resolver = Objects.requireNonNull(resolver, "resolver");
        this.applier = OutputApplier.using(actionContext);
        this.hookBindings = List.copyOf(node.hooks().values());
        this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
        this.scheduler = new StateScheduler(actionContext, pathRuntimes);
        this.action = Actions.neutral();
    }

    static MechanismRuntime of(Mechanism mechanism, ActionContext actionContext) {
        return new MechanismRuntime(
                MechanismIntrospector.inspect(mechanism),
                actionContext,
                OutputResolver.empty(),
                new HashMap<>());
    }

    static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver) {
        return new MechanismRuntime(MechanismIntrospector.inspect(mechanism), actionContext, resolver, new HashMap<>());
    }

    static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathAction, PathRuntime> pathRuntimes) {
        return new MechanismRuntime(MechanismIntrospector.inspect(mechanism), actionContext, resolver, pathRuntimes);
    }

    static MechanismRuntime of(
            MechanismNode node,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathAction, PathRuntime> pathRuntimes) {
        return new MechanismRuntime(node, actionContext, resolver, pathRuntimes);
    }

    MechanismRuntime path(PathAction ref, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(runtime, "runtime"));
        return this;
    }

    MechanismRuntime simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        return this;
    }

    public Action action() {
        return action;
    }

    public void set(Action action) {
        this.action = Objects.requireNonNull(action, "action");
        this.stateStartSeconds = Double.NaN;
        this.scheduler.reset();
    }

    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        List<ResolvedOutput> outputs = new ArrayList<>();
        periodicInto(mechanismContext, eventContext, outputs);
        return outputs;
    }

    void periodicInto(
            MechanismContext mechanismContext,
            EventContext eventContext,
            List<ResolvedOutput> outputs) {
        Objects.requireNonNull(outputs, "outputs");
        MechanismContext safeMechanismContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        EventContext safeEventContext = eventContext == null ? EventContext.empty() : eventContext;
        hookRuntime.run(safeEventContext, actionContext, hookBindings);
        if (!safeMechanismContext.enabled()) {
            applier.stopAll();
            simulationStep.run();
            return;
        }
        if (Double.isNaN(stateStartSeconds)) {
            stateStartSeconds = safeMechanismContext.nowSeconds();
        }
        MechanismContext timedContext = withTimeInState(
                safeMechanismContext,
                safeMechanismContext.nowSeconds() - stateStartSeconds);
        Action updated = mechanism.update(action, timedContext);
        if (updated != action) {
            action = Objects.requireNonNull(updated, "updated");
            stateStartSeconds = safeMechanismContext.nowSeconds();
            scheduler.reset();
            timedContext = withTimeInState(safeMechanismContext, 0.0);
        }
        StateScheduler.Result active = scheduler.evaluate(action, timedContext);
        int outputStart = outputs.size();
        resolver.resolveInto(mechanism, active.action(), active.context(), outputs);
        applier.applyAll(outputs, outputStart, active.context());
        simulationStep.run();
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
        private final Map<PathAction, PathRuntime> pathRuntimes;
        private final Result result = new Result();
        private SchedulerNode root;

        private StateScheduler(ActionContext actionContext, Map<PathAction, PathRuntime> pathRuntimes) {
            this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
            this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
        }

        void reset() {
            if (root != null) {
                root.reset();
            }
        }

        Result evaluate(Action action, MechanismContext context) {
            if (root == null || root.action() != action) {
                root = SchedulerNode.lower(action);
            }
            Evaluation evaluation = evaluate(root, context);
            return result.set(
                    evaluation.output() == null ? Actions.neutral() : evaluation.output(),
                    evaluation.context());
        }

        private Evaluation evaluate(SchedulerNode schedule, MechanismContext context) {
            Objects.requireNonNull(schedule, "schedule");
            Action action = schedule.action();
            Node node = schedule.runtime(context);
            MechanismContext local = withTimeInState(context, context.nowSeconds() - node.startSeconds);

            if (action instanceof Actions.Sequence sequence) {
                return evaluateSequence(sequence, schedule, context, local, node);
            }
            if (action instanceof Actions.Cycle cycle) {
                return evaluateCycle(cycle, schedule, context, node);
            }
            if (action instanceof Actions.Parallel parallel) {
                return evaluateGroup(parallel.Actions(), schedule, context, node, GroupMode.PARALLEL, -1);
            }
            if (action instanceof Actions.Race race) {
                return evaluateGroup(race.Actions(), schedule, context, node, GroupMode.RACE, -1);
            }
            if (action instanceof Actions.Deadline deadline) {
                return evaluateGroup(deadline.Actions(), schedule, context, node, GroupMode.DEADLINE, 0);
            }
            if (action instanceof Actions.Computed computed) {
                Evaluation child = evaluate(schedule.named("computed", computed.evaluate(local)), context);
                return schedule.result(child.output(), false, child.context());
            }
            if (action instanceof Actions.Choice choice) {
                return evaluate(schedule.named("choice", choice.choose(local)), context);
            }
            if (action instanceof Actions.WhenBranch branch) {
                return evaluate(schedule.named("when", branch.choose(local)), context);
            }
            if (action instanceof Actions.Timeout timeout) {
                if (timeout.expired(local)) {
                    return schedule.result(null, true, local);
                }
                Evaluation child = evaluate(schedule.named("timeout", timeout.action()), context);
                return schedule.result(child.output(), child.complete(), child.context());
            }
            if (action instanceof Actions.Conditional conditional) {
                return evaluateConditional(
                        conditional.action(),
                        conditional.condition(),
                        conditional.next(),
                        schedule,
                        context,
                        local);
            }
            if (action instanceof Action.Conditional conditional) {
                return evaluateConditional(
                        conditional.action(),
                        conditional.condition(),
                        conditional.next(),
                        schedule,
                        context,
                        local);
            }
            if (action instanceof Actions.Then then) {
                return evaluateThen(then.action(), then.next(), schedule, context);
            }
            if (action instanceof Action.Then then) {
                return evaluateThen(then.action(), then.next(), schedule, context);
            }
            if (action instanceof Actions.Clamped clamped) {
                Evaluation child = evaluate(schedule.named("clamped", clamped.action()), context);
                return schedule.result(child.output() == null ? null : Actions.clamp(child.output(), clamped.range()),
                        child.complete(), child.context());
            }
            if (action instanceof Action.Clamped clamped) {
                Evaluation child = evaluate(schedule.named("clamped", clamped.action()), context);
                return schedule.result(child.output() == null ? null : Actions.clamp(child.output(), clamped.range()),
                        child.complete(), child.context());
            }
            if (action instanceof Actions.RuntimeAction runtimeAction) {
                if (!node.entered) {
                    runtimeAction.action().apply(actionContext);
                    node.entered = true;
                }
                return schedule.result(null, true, local);
            }
            if (action instanceof Actions.EncoderSetPosition setPosition) {
                if (!node.entered) {
                    actionContext.encoder(setPosition.encoder()).setPositionRotations(
                            setPosition.encoder().rotationsFromPosition(setPosition.position()));
                    node.entered = true;
                }
                return schedule.result(null, true, local);
            }
            if (action instanceof Actions.DoOnce doOnce) {
                if (!node.entered) {
                    doOnce.action().run();
                    node.entered = true;
                }
                return schedule.result(null, true, local);
            }
            if (action instanceof Actions.WaitSeconds wait) {
                return schedule.result(null, wait.complete(local), local);
            }
            if (action instanceof Actions.WaitUntil wait) {
                return schedule.result(null, wait.complete(local), local);
            }
            if (action instanceof PathAction pathState) {
                if (node.pathRuntime == null || node.pathState != pathState) {
                    node.pathState = pathState;
                    node.pathRuntime = pathRuntimes.get(pathState);
                }
                PathRuntime runtime = node.pathRuntime;
                if (runtime == null) {
                    throw new IllegalStateException("No PathRuntime bound for path " + pathState.key());
                }
                if (!node.entered) {
                    runtime.initialize(pathState, local);
                    node.entered = true;
                }
                runtime.execute(pathState, local);
                if (runtime.isFinished(pathState, local)) {
                    runtime.end(pathState, local, false);
                    return schedule.result(null, true, local);
                }
                return schedule.result(null, false, local);
            }
            return schedule.result(action, false, local);
        }

        private Evaluation evaluateSequence(
                Actions.Sequence sequence,
                SchedulerNode schedule,
                MechanismContext context,
                MechanismContext local,
                Node node) {
            if (local.timeInStateSeconds() >= sequence.timeoutSeconds()) {
                return sequence.next() == null
                        ? schedule.result(null, true, local)
                        : evaluate(schedule.named("next", sequence.next()), context);
            }
            List<Actions.SequenceStep> steps = sequence.steps();
            while (node.index < steps.size()) {
                Actions.SequenceStep step = steps.get(node.index);
                SchedulerNode stepNode = schedule.indexed(node.index, step.action());
                Evaluation child = evaluate(stepNode, context);
                if (child.complete() || step.complete().test(child.context())) {
                    stepNode.reset();
                    node.index++;
                    continue;
                }
                return child;
            }
            if (sequence.next() != null) {
                return evaluate(schedule.named("next", sequence.next()), context);
            }
            return schedule.result(null, true, local);
        }

        private Evaluation evaluateCycle(
                Actions.Cycle cycle,
                SchedulerNode schedule,
                MechanismContext context,
                Node node) {
            List<Actions.CycleStep> steps = cycle.steps();
            if (steps.isEmpty()) {
                return schedule.result(null, false, context);
            }
            if (node.index >= steps.size()) {
                node.index = 0;
            }
            for (int attempts = 0; attempts < steps.size(); attempts++) {
                Actions.CycleStep step = steps.get(node.index);
                SchedulerNode stepNode = schedule.indexed(node.index, step.action());
                Evaluation child = evaluate(stepNode, context);
                if (child.complete() || step.advance().test(child.context())) {
                    stepNode.reset();
                    node.index = (node.index + 1) % steps.size();
                    continue;
                }
                return child;
            }
            return schedule.result(null, false, context);
        }

        private Evaluation evaluateGroup(
                List<Action> actions,
                SchedulerNode schedule,
                MechanismContext context,
                Node node,
                GroupMode mode,
                int deadlineIndex) {
            List<Action> outputs = node.groupOutputs;
            outputs.clear();
            boolean anyComplete = false;
            boolean allComplete = !actions.isEmpty();
            for (int i = 0; i < actions.size(); i++) {
                Evaluation child = evaluate(schedule.indexed(i, actions.get(i)), context);
                anyComplete |= child.complete();
                allComplete &= child.complete();
                if (mode == GroupMode.DEADLINE && i == deadlineIndex && child.complete()) {
                    return schedule.result(null, true, child.context());
                }
                if (child.output() != null) {
                    outputs.add(child.output());
                }
            }
            if (mode == GroupMode.RACE && anyComplete) {
                return schedule.result(null, true, context);
            }
            if (mode == GroupMode.PARALLEL && allComplete) {
                return schedule.result(null, true, context);
            }
            return schedule.result(groupOutput(outputs), false, context);
        }

        private Evaluation evaluateConditional(
                Action wrapped,
                ActionCondition condition,
                Action next,
                SchedulerNode schedule,
                MechanismContext context,
                MechanismContext local) {
            if (condition.test(local)) {
                return next == null ? schedule.result(null, true, local) : evaluate(schedule.named("next", next), context);
            }
            Evaluation child = evaluate(schedule.named("conditional", wrapped), context);
            if (child.complete()) {
                return next == null ? child : evaluate(schedule.named("next", next), context);
            }
            return child;
        }

        private Evaluation evaluateThen(
                Action wrapped,
                Action next,
                SchedulerNode schedule,
                MechanismContext context) {
            Evaluation child = evaluate(schedule.named("thenState", wrapped), context);
            if (child.complete()) {
                return evaluate(schedule.named("thenNext", next), context);
            }
            return child;
        }

        private static Action groupOutput(List<Action> actions) {
            if (actions.isEmpty()) {
                return null;
            }
            if (actions.size() == 1) {
                return actions.get(0);
            }
            return new Actions.Parallel(actions);
        }

        private enum GroupMode {
            PARALLEL,
            RACE,
            DEADLINE
        }

        private static final class Evaluation {
            private Action output;
            private boolean complete;
            private MechanismContext context;

            private Evaluation set(Action output, boolean complete, MechanismContext context) {
                this.output = output;
                this.complete = complete;
                this.context = context;
                return this;
            }

            private Action output() {
                return output;
            }

            private boolean complete() {
                return complete;
            }

            private MechanismContext context() {
                return context;
            }
        }

        private static final class Result {
            private Action action;
            private MechanismContext context;

            private Result set(Action action, MechanismContext context) {
                this.action = action;
                this.context = context;
                return this;
            }

            private Action action() {
                return action;
            }

            private MechanismContext context() {
                return context;
            }
        }

        private static final class Node {
            private final double startSeconds;
            private boolean entered;
            private int index;
            private PathAction pathState;
            private PathRuntime pathRuntime;
            private final List<Action> groupOutputs = new ArrayList<>();

            private Node(double startSeconds) {
                this.startSeconds = startSeconds;
            }
        }

        private static final class SchedulerNode {
            private final Action action;
            private final List<SchedulerNode> indexedChildren = new ArrayList<>();
            private final Map<String, SchedulerNode> namedChildren = new HashMap<>();
            private final Evaluation evaluation = new Evaluation();
            private Node runtime;

            private SchedulerNode(Action action) {
                this.action = Objects.requireNonNull(action, "action");
                lowerKnownChildren();
            }

            private static SchedulerNode lower(Action action) {
                return new SchedulerNode(action);
            }

            private Action action() {
                return action;
            }

            private Node runtime(MechanismContext context) {
                if (runtime == null) {
                    runtime = new Node(context.nowSeconds());
                }
                return runtime;
            }

            private SchedulerNode indexed(int index, Action childState) {
                while (indexedChildren.size() <= index) {
                    indexedChildren.add(null);
                }
                SchedulerNode child = indexedChildren.get(index);
                if (child == null || child.action() != childState) {
                    child = lower(childState);
                    indexedChildren.set(index, child);
                }
                return child;
            }

            private SchedulerNode named(String name, Action childState) {
                SchedulerNode child = namedChildren.get(name);
                if (child == null || child.action() != childState) {
                    child = lower(childState);
                    namedChildren.put(name, child);
                }
                return child;
            }

            private void reset() {
                runtime = null;
                for (SchedulerNode child : indexedChildren) {
                    if (child != null) {
                        child.reset();
                    }
                }
                namedChildren.values().forEach(SchedulerNode::reset);
            }

            private Evaluation result(Action output, boolean complete, MechanismContext context) {
                return evaluation.set(output, complete, context);
            }

            private void lowerKnownChildren() {
                if (action instanceof Actions.Sequence sequence) {
                    int index = 0;
                    for (Actions.SequenceStep step : sequence.steps()) {
                        indexed(index++, step.action());
                    }
                    if (sequence.next() != null) {
                        named("next", sequence.next());
                    }
                } else if (action instanceof Actions.Cycle cycle) {
                    int index = 0;
                    for (Actions.CycleStep step : cycle.steps()) {
                        indexed(index++, step.action());
                    }
                } else if (action instanceof Actions.Parallel parallel) {
                    lowerIndexed(parallel.Actions());
                } else if (action instanceof Actions.Race race) {
                    lowerIndexed(race.Actions());
                } else if (action instanceof Actions.Deadline deadline) {
                    lowerIndexed(deadline.Actions());
                } else if (action instanceof Actions.Timeout timeout) {
                    named("timeout", timeout.action());
                } else if (action instanceof Actions.Conditional conditional) {
                    lowerConditional(conditional.action(), conditional.next());
                } else if (action instanceof Action.Conditional conditional) {
                    lowerConditional(conditional.action(), conditional.next());
                } else if (action instanceof Actions.Then then) {
                    lowerThen(then.action(), then.next());
                } else if (action instanceof Action.Then then) {
                    lowerThen(then.action(), then.next());
                } else if (action instanceof Actions.Clamped clamped) {
                    named("clamped", clamped.action());
                } else if (action instanceof Action.Clamped clamped) {
                    named("clamped", clamped.action());
                }
            }

            private void lowerIndexed(List<Action> actions) {
                for (int i = 0; i < actions.size(); i++) {
                    indexed(i, actions.get(i));
                }
            }

            private void lowerConditional(Action wrapped, Action next) {
                named("conditional", wrapped);
                if (next != null) {
                    named("next", next);
                }
            }

            private void lowerThen(Action wrapped, Action next) {
                named("thenState", wrapped);
                named("thenNext", next);
            }
        }
    }
}
