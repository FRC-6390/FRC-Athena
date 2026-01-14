package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import ca.frc6390.athena.core.RobotAuto.AutoKey;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Declarative builder for composing autonomous plans with sequencing, branching, and interrupts.
 */
public final class AutoPlan {
    private final Node root;

    private AutoPlan(Node root) {
        this.root = root;
    }

    public static AutoPlan empty() {
        return new AutoPlan(new EmptyNode());
    }

    public static AutoPlan step(AutoKey key) {
        Objects.requireNonNull(key, "key");
        return new AutoPlan(new StepNode(StepRef.of(key), null, null));
    }

    public static AutoPlan step(String id) {
        Objects.requireNonNull(id, "id");
        return new AutoPlan(new StepNode(StepRef.parse(id), null, null));
    }

    public static AutoPlan step(AutoKey key, boolean resetOdometry) {
        Objects.requireNonNull(key, "key");
        return new AutoPlan(new StepNode(StepRef.of(key), resetOdometry, null));
    }

    public static AutoPlan step(String id, boolean resetOdometry) {
        Objects.requireNonNull(id, "id");
        return new AutoPlan(new StepNode(StepRef.parse(id), resetOdometry, null));
    }

    public static AutoPlan step(AutoKey key, int splitIndex) {
        Objects.requireNonNull(key, "key");
        return new AutoPlan(new StepNode(StepRef.of(key, splitIndex), null, null));
    }

    public static AutoPlan step(String id, int splitIndex) {
        Objects.requireNonNull(id, "id");
        return new AutoPlan(new StepNode(StepRef.of(id, splitIndex), null, null));
    }

    public static AutoPlan step(AutoKey key, int splitIndex, boolean resetOdometry) {
        Objects.requireNonNull(key, "key");
        return new AutoPlan(new StepNode(StepRef.of(key, splitIndex), resetOdometry, null));
    }

    public static AutoPlan step(String id, int splitIndex, boolean resetOdometry) {
        Objects.requireNonNull(id, "id");
        return new AutoPlan(new StepNode(StepRef.of(id, splitIndex), resetOdometry, null));
    }

    public static AutoPlan step(AutoKey key, Pose2d resetPose) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(resetPose, "resetPose");
        return new AutoPlan(new StepNode(StepRef.of(key), true, resetPose));
    }

    public static AutoPlan step(String id, Pose2d resetPose) {
        Objects.requireNonNull(id, "id");
        return new AutoPlan(new StepNode(StepRef.parse(id), true, resetPose));
    }

    public static AutoPlan step(AutoKey key, int splitIndex, Pose2d resetPose) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(resetPose, "resetPose");
        return new AutoPlan(new StepNode(StepRef.of(key, splitIndex), true, resetPose));
    }

    public static AutoPlan step(String id, int splitIndex, Pose2d resetPose) {
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(resetPose, "resetPose");
        return new AutoPlan(new StepNode(StepRef.of(id, splitIndex), true, resetPose));
    }

    public static AutoPlan command(Command command) {
        Objects.requireNonNull(command, "command");
        return new AutoPlan(new CommandNode(command));
    }

    public static AutoPlan parallel(AutoPlan... steps) {
        return new AutoPlan(new ParallelNode(steps));
    }

    public static AutoPlan race(AutoPlan... steps) {
        return new AutoPlan(new RaceNode(steps));
    }

    public static AutoPlan deadline(AutoPlan deadline, AutoPlan... steps) {
        Objects.requireNonNull(deadline, "deadline");
        return new AutoPlan(new DeadlineNode(deadline, steps));
    }

    public static AutoPlan withTimeout(AutoPlan plan, double seconds) {
        Objects.requireNonNull(plan, "plan");
        return new AutoPlan(new TimeoutNode(plan, seconds));
    }

    public static AutoPlan until(AutoPlan plan, BooleanSupplier condition) {
        Objects.requireNonNull(plan, "plan");
        Objects.requireNonNull(condition, "condition");
        return new AutoPlan(new UntilNode(plan, condition));
    }

    public static AutoPlan sequence(AutoPlan... steps) {
        return new AutoPlan(new SequenceNode(steps));
    }

    public static AutoPlan branch(BooleanSupplier condition, AutoPlan ifTrue, AutoPlan ifFalse) {
        Objects.requireNonNull(condition, "condition");
        Objects.requireNonNull(ifTrue, "ifTrue");
        Objects.requireNonNull(ifFalse, "ifFalse");
        return new AutoPlan(new BranchNode(condition, ifTrue, ifFalse));
    }

    public static AutoPlan interrupt(AutoPlan base, BooleanSupplier condition, AutoPlan onInterrupt) {
        Objects.requireNonNull(base, "base");
        Objects.requireNonNull(condition, "condition");
        Objects.requireNonNull(onInterrupt, "onInterrupt");
        return new AutoPlan(new InterruptNode(base, condition, onInterrupt));
    }

    public AutoPlan then(AutoPlan next) {
        Objects.requireNonNull(next, "next");
        return sequence(this, next);
    }

    public AutoPlan withTimeout(double seconds) {
        return withTimeout(this, seconds);
    }

    public AutoPlan until(BooleanSupplier condition) {
        Objects.requireNonNull(condition, "condition");
        return until(this, condition);
    }

    public AutoPlan parallelWith(AutoPlan... others) {
        AutoPlan[] plans = new AutoPlan[(others == null ? 0 : others.length) + 1];
        plans[0] = this;
        if (others != null && others.length > 0) {
            System.arraycopy(others, 0, plans, 1, others.length);
        }
        return parallel(plans);
    }

    public AutoPlan raceWith(AutoPlan... others) {
        AutoPlan[] plans = new AutoPlan[(others == null ? 0 : others.length) + 1];
        plans[0] = this;
        if (others != null && others.length > 0) {
            System.arraycopy(others, 0, plans, 1, others.length);
        }
        return race(plans);
    }

    public AutoPlan deadlineWith(AutoPlan... others) {
        return deadline(this, others);
    }

    public Command build(Function<AutoKey, Command> resolver) {
        Objects.requireNonNull(resolver, "resolver");
        return build(new SimpleContext(resolver));
    }

    public Command build(Context context) {
        Objects.requireNonNull(context, "context");
        return root.build(context);
    }

    private interface Node {
        Command build(Context context);
        void collect(List<StepRef> steps);
    }

    public interface Context {
        Command resolve(StepRef ref);
        Optional<Pose2d> startingPose(StepRef ref);
        Command resetPose(Pose2d pose);
    }

    List<StepRef> stepRefs() {
        List<StepRef> refs = new ArrayList<>();
        root.collect(refs);
        return refs;
    }

    private static final class SequenceNode implements Node {
        private final List<AutoPlan> steps;

        private SequenceNode(AutoPlan... steps) {
            this.steps = new ArrayList<>();
            if (steps != null) {
                this.steps.addAll(Arrays.asList(steps));
            }
        }

        @Override
        public Command build(Context context) {
            List<Command> commands = new ArrayList<>();
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                commands.add(step.build(context));
            }
            if (commands.isEmpty()) {
                return Commands.none();
            }
            return Commands.sequence(commands.toArray(Command[]::new));
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                stepsOut.addAll(step.stepRefs());
            }
        }
    }

    private static final class EmptyNode implements Node {
        @Override
        public Command build(Context context) {
            return Commands.none();
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
        }
    }

    private static final class StepNode implements Node {
        private final StepRef ref;
        private final Boolean resetOdometry;
        private final Pose2d resetPose;

        private StepNode(StepRef ref, Boolean resetOdometry, Pose2d resetPose) {
            this.ref = ref;
            this.resetOdometry = resetOdometry;
            this.resetPose = resetPose;
        }

        @Override
        public Command build(Context context) {
            Command base = context.resolve(ref);
            if (resetPose != null) {
                return Commands.sequence(context.resetPose(resetPose), base);
            }
            if (resetOdometry == null || !resetOdometry.booleanValue()) {
                return base;
            }
            return context.startingPose(ref)
                    .map(pose -> Commands.sequence(context.resetPose(pose), base))
                    .orElse(base);
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            stepsOut.add(ref);
        }
    }

    private static final class CommandNode implements Node {
        private final Command command;

        private CommandNode(Command command) {
            this.command = command;
        }

        @Override
        public Command build(Context context) {
            return command;
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
        }
    }

    private static final class ParallelNode implements Node {
        private final List<AutoPlan> steps;

        private ParallelNode(AutoPlan... steps) {
            this.steps = new ArrayList<>();
            if (steps != null) {
                this.steps.addAll(Arrays.asList(steps));
            }
        }

        @Override
        public Command build(Context context) {
            List<Command> commands = new ArrayList<>();
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                commands.add(step.build(context));
            }
            if (commands.isEmpty()) {
                return Commands.none();
            }
            return Commands.parallel(commands.toArray(Command[]::new));
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                stepsOut.addAll(step.stepRefs());
            }
        }
    }

    private static final class RaceNode implements Node {
        private final List<AutoPlan> steps;

        private RaceNode(AutoPlan... steps) {
            this.steps = new ArrayList<>();
            if (steps != null) {
                this.steps.addAll(Arrays.asList(steps));
            }
        }

        @Override
        public Command build(Context context) {
            List<Command> commands = new ArrayList<>();
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                commands.add(step.build(context));
            }
            if (commands.isEmpty()) {
                return Commands.none();
            }
            return Commands.race(commands.toArray(Command[]::new));
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                stepsOut.addAll(step.stepRefs());
            }
        }
    }

    private static final class DeadlineNode implements Node {
        private final AutoPlan deadline;
        private final List<AutoPlan> steps;

        private DeadlineNode(AutoPlan deadline, AutoPlan... steps) {
            this.deadline = deadline;
            this.steps = new ArrayList<>();
            if (steps != null) {
                this.steps.addAll(Arrays.asList(steps));
            }
        }

        @Override
        public Command build(Context context) {
            Command deadlineCommand = deadline.build(context);
            List<Command> commands = new ArrayList<>();
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                commands.add(step.build(context));
            }
            return Commands.deadline(deadlineCommand, commands.toArray(Command[]::new));
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            stepsOut.addAll(deadline.stepRefs());
            for (AutoPlan step : steps) {
                if (step == null) {
                    continue;
                }
                stepsOut.addAll(step.stepRefs());
            }
        }
    }

    private static final class TimeoutNode implements Node {
        private final AutoPlan plan;
        private final double seconds;

        private TimeoutNode(AutoPlan plan, double seconds) {
            this.plan = plan;
            this.seconds = seconds;
        }

        @Override
        public Command build(Context context) {
            return plan.build(context).withTimeout(seconds);
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            stepsOut.addAll(plan.stepRefs());
        }
    }

    private static final class UntilNode implements Node {
        private final AutoPlan plan;
        private final BooleanSupplier condition;

        private UntilNode(AutoPlan plan, BooleanSupplier condition) {
            this.plan = plan;
            this.condition = condition;
        }

        @Override
        public Command build(Context context) {
            return plan.build(context).until(condition);
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            stepsOut.addAll(plan.stepRefs());
        }
    }

    private static final class BranchNode implements Node {
        private final BooleanSupplier condition;
        private final AutoPlan ifTrue;
        private final AutoPlan ifFalse;

        private BranchNode(BooleanSupplier condition, AutoPlan ifTrue, AutoPlan ifFalse) {
            this.condition = condition;
            this.ifTrue = ifTrue;
            this.ifFalse = ifFalse;
        }

        @Override
        public Command build(Context context) {
            return Commands.either(ifTrue.build(context), ifFalse.build(context), condition);
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            stepsOut.addAll(ifTrue.stepRefs());
            stepsOut.addAll(ifFalse.stepRefs());
        }
    }

    private static final class InterruptNode implements Node {
        private final AutoPlan base;
        private final BooleanSupplier condition;
        private final AutoPlan onInterrupt;

        private InterruptNode(AutoPlan base, BooleanSupplier condition, AutoPlan onInterrupt) {
            this.base = base;
            this.condition = condition;
            this.onInterrupt = onInterrupt;
        }

        @Override
        public Command build(Context context) {
            Command baseCmd = base.build(context);
            Command interruptCmd = onInterrupt.build(context);
            Command race = Commands.race(baseCmd, Commands.waitUntil(condition));
            return Commands.sequence(race, Commands.either(interruptCmd, Commands.none(), condition));
        }

        @Override
        public void collect(List<StepRef> stepsOut) {
            stepsOut.addAll(base.stepRefs());
            stepsOut.addAll(onInterrupt.stepRefs());
        }
    }

    private static final class SimpleContext implements Context {
        private final Function<AutoKey, Command> resolver;

        private SimpleContext(Function<AutoKey, Command> resolver) {
            this.resolver = resolver;
        }

        @Override
        public Command resolve(StepRef ref) {
            return resolver.apply(ref.key());
        }

        @Override
        public Optional<Pose2d> startingPose(StepRef ref) {
            return Optional.empty();
        }

        @Override
        public Command resetPose(Pose2d pose) {
            return Commands.none();
        }
    }

    public record StepRef(AutoKey key, OptionalInt splitIndex) {
        private static StepRef of(AutoKey key) {
            return new StepRef(key, OptionalInt.empty());
        }

        private static StepRef of(AutoKey key, int splitIndex) {
            return new StepRef(key, OptionalInt.of(splitIndex));
        }

        private static StepRef of(String id, int splitIndex) {
            return new StepRef(AutoKey.of(id), OptionalInt.of(splitIndex));
        }

        private static StepRef parse(String id) {
            int dot = id.lastIndexOf('.');
            if (dot <= 0 || dot == id.length() - 1) {
                return new StepRef(AutoKey.of(id), OptionalInt.empty());
            }
            String maybeIndex = id.substring(dot + 1);
            try {
                int index = Integer.parseInt(maybeIndex);
                return new StepRef(AutoKey.of(id.substring(0, dot)), OptionalInt.of(index));
            } catch (NumberFormatException ex) {
                return new StepRef(AutoKey.of(id), OptionalInt.empty());
            }
        }
    }
}
