package ca.frc6390.athena.mechanisms.superstructure;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import ca.frc6390.athena.util.SupplierUtil;

/**
 * Represents a single step inside a macro state. Each step queues a set of mechanism setpoints and
 * waits for a completion condition before advancing.
 */
public final class SuperstructureStateStep {

    private final List<StepCommand> commands;
    private final BooleanSupplier completion;
    private final List<Consumer<SuperstructureRuntime>> onEnter;
    private final List<Consumer<SuperstructureRuntime>> onExit;

    private SuperstructureStateStep(List<StepCommand> commands,
                                    BooleanSupplier completion,
                                    List<Consumer<SuperstructureRuntime>> onEnter,
                                    List<Consumer<SuperstructureRuntime>> onExit) {
        this.commands = List.copyOf(commands);
        this.completion = SupplierUtil.wrapBoolean(completion, true);
        this.onEnter = List.copyOf(onEnter);
        this.onExit = List.copyOf(onExit);
    }

    void enter(SuperstructureRuntime runtime) {
        onEnter.forEach(action -> action.accept(runtime));
        commands.forEach(command -> runtime.command(command.key, command.state, command.condition));
    }

    void exit(SuperstructureRuntime runtime) {
        onExit.forEach(action -> action.accept(runtime));
    }

    boolean isComplete() {
        return completion.getAsBoolean();
    }

    public static Builder create() {
        return new Builder();
    }

    private record StepCommand(String key, Enum<?> state, BooleanSupplier condition) {}

    public static final class Builder {
        private final List<StepCommand> commands = new ArrayList<>();
        private final List<Consumer<SuperstructureRuntime>> onEnter = new ArrayList<>();
        private final List<Consumer<SuperstructureRuntime>> onExit = new ArrayList<>();
        private BooleanSupplier completion = () -> true;

        public Builder setpoint(String key, Enum<?> state) {
            return setpoint(key, state, () -> true);
        }

        public Builder setpoint(String key, Enum<?> state, BooleanSupplier condition) {
            commands.add(new StepCommand(key, Objects.requireNonNull(state), SupplierUtil.wrapBoolean(condition, true)));
            return this;
        }

        public Builder onEnter(Consumer<SuperstructureRuntime> action) {
            onEnter.add(Objects.requireNonNull(action));
            return this;
        }

        public Builder onExit(Consumer<SuperstructureRuntime> action) {
            onExit.add(Objects.requireNonNull(action));
            return this;
        }

        public Builder await(BooleanSupplier completionCondition) {
            completion = SupplierUtil.wrapBoolean(completionCondition, true);
            return this;
        }

        public SuperstructureStateStep build() {
            return new SuperstructureStateStep(commands, completion, onEnter, onExit);
        }
    }
}
