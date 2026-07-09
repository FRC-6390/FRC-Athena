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
public final class MechanismRuntime {
    private final Mechanism mechanism;
    private final MechanismNode node;
    private final ActionContext actionContext;
    private final OutputResolver resolver;
    private final OutputApplier applier;
    private final HookRuntime hookRuntime = new HookRuntime();
    private final List<HookBinding> hookBindings;
    private final StateScheduler scheduler;
    private final Map<PathState, PathRuntime> pathRuntimes;
    private Runnable simulationStep = () -> {
    };
    private State state;
    private double stateStartSeconds = Double.NaN;

    private MechanismRuntime(
            MechanismNode node,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathState, PathRuntime> pathRuntimes) {
        this.node = Objects.requireNonNull(node, "node");
        this.mechanism = Objects.requireNonNull(node.mechanism(), "mechanism");
        this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
        this.resolver = Objects.requireNonNull(resolver, "resolver");
        this.applier = OutputApplier.using(actionContext);
        this.hookBindings = List.copyOf(node.hooks().values());
        this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
        this.scheduler = new StateScheduler(actionContext, pathRuntimes);
        if (node.initialState() == null) {
            throw new IllegalStateException(
                    "Mechanism " + mechanism.getClass().getName() + " does not declare any State fields.");
        }
        this.state = node.initialState();
    }

    public static MechanismRuntime of(Mechanism mechanism, ActionContext actionContext) {
        return new MechanismRuntime(
                MechanismIntrospector.inspect(mechanism),
                actionContext,
                OutputResolver.empty(),
                new HashMap<>());
    }

    public static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver) {
        return new MechanismRuntime(MechanismIntrospector.inspect(mechanism), actionContext, resolver, new HashMap<>());
    }

    public static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathState, PathRuntime> pathRuntimes) {
        return new MechanismRuntime(MechanismIntrospector.inspect(mechanism), actionContext, resolver, pathRuntimes);
    }

    static MechanismRuntime of(
            MechanismNode node,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathState, PathRuntime> pathRuntimes) {
        return new MechanismRuntime(node, actionContext, resolver, pathRuntimes);
    }

    public MechanismRuntime path(PathState ref, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(runtime, "runtime"));
        return this;
    }

    public MechanismRuntime simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        return this;
    }

    public State state() {
        return state;
    }

    public void set(State state) {
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
        State updated = mechanism.update(state, timedContext);
        if (updated != state) {
            state = Objects.requireNonNull(updated, "updated");
            stateStartSeconds = safeMechanismContext.nowSeconds();
            scheduler.reset();
            timedContext = withTimeInState(safeMechanismContext, 0.0);
        }
        StateScheduler.Result active = scheduler.evaluate(state, timedContext);
        List<ResolvedOutput> outputs = resolver.resolve(mechanism, active.state(), active.context());
        applier.applyAll(outputs);
        hookRuntime.run(safeEventContext, actionContext, hookBindings);
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
        private final Map<PathState, PathRuntime> pathRuntimes;
        private SchedulerNode root;

        private StateScheduler(ActionContext actionContext, Map<PathState, PathRuntime> pathRuntimes) {
            this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
            this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
        }

        void reset() {
            if (root != null) {
                root.reset();
            }
        }

        Result evaluate(State state, MechanismContext context) {
            if (root == null || root.state() != state) {
                root = SchedulerNode.lower(state);
            }
            Evaluation evaluation = evaluate(root, context);
            return new Result(evaluation.output() == null ? States.neutral() : evaluation.output(), evaluation.context());
        }

        private Evaluation evaluate(SchedulerNode schedule, MechanismContext context) {
            Objects.requireNonNull(schedule, "schedule");
            State state = schedule.state();
            Node node = schedule.runtime(context);
            MechanismContext local = withTimeInState(context, context.nowSeconds() - node.startSeconds);

            if (state instanceof States.ChildSet childSet) {
                States.ChildSet output = States.set();
                boolean complete = !childSet.targets().isEmpty();
                int index = 0;
                for (States.ChildTarget target : childSet.targets()) {
                    Evaluation child = evaluate(schedule.indexed(index, target.state()), context);
                    if (child.output() != null) {
                        output.set(target.mechanism(), child.output());
                    }
                    complete &= child.complete();
                    index++;
                }
                return new Evaluation(output.targets().isEmpty() ? null : output, complete, local);
            }
            if (state instanceof States.Sequence sequence) {
                return evaluateSequence(sequence, schedule, context, local, node);
            }
            if (state instanceof States.Cycle cycle) {
                return evaluateCycle(cycle, schedule, context, node);
            }
            if (state instanceof States.Parallel parallel) {
                return evaluateGroup(parallel.states(), schedule, context, GroupMode.PARALLEL, -1);
            }
            if (state instanceof States.Race race) {
                return evaluateGroup(race.states(), schedule, context, GroupMode.RACE, -1);
            }
            if (state instanceof States.Deadline deadline) {
                return evaluateGroup(deadline.states(), schedule, context, GroupMode.DEADLINE, 0);
            }
            if (state instanceof States.Choice choice) {
                return evaluate(schedule.named("choice", choice.choose(local)), context);
            }
            if (state instanceof States.WhenBranch branch) {
                return evaluate(schedule.named("when", branch.choose(local)), context);
            }
            if (state instanceof States.Timeout timeout) {
                if (timeout.expired(local)) {
                    return new Evaluation(null, true, local);
                }
                Evaluation child = evaluate(schedule.named("timeout", timeout.state()), context);
                return new Evaluation(child.output(), child.complete(), child.context());
            }
            if (state instanceof States.Conditional conditional) {
                return evaluateConditional(
                        conditional.state(),
                        conditional.condition(),
                        conditional.next(),
                        schedule,
                        context,
                        local);
            }
            if (state instanceof State.Conditional conditional) {
                return evaluateConditional(
                        conditional.state(),
                        conditional.condition(),
                        conditional.next(),
                        schedule,
                        context,
                        local);
            }
            if (state instanceof States.Then then) {
                return evaluateThen(then.state(), then.next(), schedule, context);
            }
            if (state instanceof State.Then then) {
                return evaluateThen(then.state(), then.next(), schedule, context);
            }
            if (state instanceof States.Clamped clamped) {
                Evaluation child = evaluate(schedule.named("clamped", clamped.state()), context);
                return new Evaluation(child.output() == null ? null : States.clamp(child.output(), clamped.range()),
                        child.complete(), child.context());
            }
            if (state instanceof State.Clamped clamped) {
                Evaluation child = evaluate(schedule.named("clamped", clamped.state()), context);
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
            if (state instanceof PathState pathState) {
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
                    return new Evaluation(null, true, local);
                }
                return new Evaluation(null, false, local);
            }
            return new Evaluation(state, false, local);
        }

        private Evaluation evaluateSequence(
                States.Sequence sequence,
                SchedulerNode schedule,
                MechanismContext context,
                MechanismContext local,
                Node node) {
            if (local.timeInStateSeconds() >= sequence.timeoutSeconds()) {
                return sequence.next() == null
                        ? new Evaluation(null, true, local)
                        : evaluate(schedule.named("next", sequence.next()), context);
            }
            List<States.SequenceStep> steps = sequence.steps();
            while (node.index < steps.size()) {
                States.SequenceStep step = steps.get(node.index);
                SchedulerNode stepNode = schedule.indexed(node.index, step.state());
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
            return new Evaluation(null, true, local);
        }

        private Evaluation evaluateCycle(
                States.Cycle cycle,
                SchedulerNode schedule,
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
                SchedulerNode stepNode = schedule.indexed(node.index, step.state());
                Evaluation child = evaluate(stepNode, context);
                if (child.complete() || step.advance().test(child.context())) {
                    stepNode.reset();
                    node.index = (node.index + 1) % steps.size();
                    continue;
                }
                return child;
            }
            return new Evaluation(null, false, context);
        }

        private Evaluation evaluateGroup(
                List<State> states,
                SchedulerNode schedule,
                MechanismContext context,
                GroupMode mode,
                int deadlineIndex) {
            List<State> outputs = new ArrayList<>();
            boolean anyComplete = false;
            boolean allComplete = !states.isEmpty();
            for (int i = 0; i < states.size(); i++) {
                Evaluation child = evaluate(schedule.indexed(i, states.get(i)), context);
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
                State wrapped,
                StateCondition condition,
                State next,
                SchedulerNode schedule,
                MechanismContext context,
                MechanismContext local) {
            if (condition.test(local)) {
                return next == null ? new Evaluation(null, true, local) : evaluate(schedule.named("next", next), context);
            }
            Evaluation child = evaluate(schedule.named("conditional", wrapped), context);
            if (child.complete()) {
                return next == null ? child : evaluate(schedule.named("next", next), context);
            }
            return child;
        }

        private Evaluation evaluateThen(
                State wrapped,
                State next,
                SchedulerNode schedule,
                MechanismContext context) {
            Evaluation child = evaluate(schedule.named("thenState", wrapped), context);
            if (child.complete()) {
                return evaluate(schedule.named("thenNext", next), context);
            }
            return child;
        }

        private static State groupOutput(List<State> states) {
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

        private record Evaluation(State output, boolean complete, MechanismContext context) {
        }

        private record Result(State state, MechanismContext context) {
        }

        private static final class Node {
            private final double startSeconds;
            private boolean entered;
            private int index;
            private PathState pathState;
            private PathRuntime pathRuntime;

            private Node(double startSeconds) {
                this.startSeconds = startSeconds;
            }
        }

        private static final class SchedulerNode {
            private final State state;
            private final List<SchedulerNode> indexedChildren = new ArrayList<>();
            private final Map<String, SchedulerNode> namedChildren = new HashMap<>();
            private Node runtime;

            private SchedulerNode(State state) {
                this.state = Objects.requireNonNull(state, "state");
                lowerKnownChildren();
            }

            private static SchedulerNode lower(State state) {
                return new SchedulerNode(state);
            }

            private State state() {
                return state;
            }

            private Node runtime(MechanismContext context) {
                if (runtime == null) {
                    runtime = new Node(context.nowSeconds());
                }
                return runtime;
            }

            private SchedulerNode indexed(int index, State childState) {
                while (indexedChildren.size() <= index) {
                    indexedChildren.add(null);
                }
                SchedulerNode child = indexedChildren.get(index);
                if (child == null || child.state() != childState) {
                    child = lower(childState);
                    indexedChildren.set(index, child);
                }
                return child;
            }

            private SchedulerNode named(String name, State childState) {
                SchedulerNode child = namedChildren.get(name);
                if (child == null || child.state() != childState) {
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

            private void lowerKnownChildren() {
                if (state instanceof States.ChildSet childSet) {
                    int index = 0;
                    for (States.ChildTarget target : childSet.targets()) {
                        indexed(index++, target.state());
                    }
                } else if (state instanceof States.Sequence sequence) {
                    int index = 0;
                    for (States.SequenceStep step : sequence.steps()) {
                        indexed(index++, step.state());
                    }
                    if (sequence.next() != null) {
                        named("next", sequence.next());
                    }
                } else if (state instanceof States.Cycle cycle) {
                    int index = 0;
                    for (States.CycleStep step : cycle.steps()) {
                        indexed(index++, step.state());
                    }
                } else if (state instanceof States.Parallel parallel) {
                    lowerIndexed(parallel.states());
                } else if (state instanceof States.Race race) {
                    lowerIndexed(race.states());
                } else if (state instanceof States.Deadline deadline) {
                    lowerIndexed(deadline.states());
                } else if (state instanceof States.Timeout timeout) {
                    named("timeout", timeout.state());
                } else if (state instanceof States.Conditional conditional) {
                    lowerConditional(conditional.state(), conditional.next());
                } else if (state instanceof State.Conditional conditional) {
                    lowerConditional(conditional.state(), conditional.next());
                } else if (state instanceof States.Then then) {
                    lowerThen(then.state(), then.next());
                } else if (state instanceof State.Then then) {
                    lowerThen(then.state(), then.next());
                } else if (state instanceof States.Clamped clamped) {
                    named("clamped", clamped.state());
                } else if (state instanceof State.Clamped clamped) {
                    named("clamped", clamped.state());
                }
            }

            private void lowerIndexed(List<State> states) {
                for (int i = 0; i < states.size(); i++) {
                    indexed(i, states.get(i));
                }
            }

            private void lowerConditional(State wrapped, State next) {
                named("conditional", wrapped);
                if (next != null) {
                    named("next", next);
                }
            }

            private void lowerThen(State wrapped, State next) {
                named("thenState", wrapped);
                named("thenNext", next);
            }
        }
    }
}
