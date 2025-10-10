package ca.frc6390.athena.mechanisms.superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import ca.frc6390.athena.util.SupplierUtil;

/**
 * Compiled representation of a macro state's behavior. Contains the ordered steps that will execute,
 * completion conditions, and optional side effects.
 */
public final class SuperstructureStateDef {

    private final List<SuperstructureStateStep> steps;
    private final List<Consumer<SuperstructureRuntime>> onEnter;
    private final List<Consumer<SuperstructureRuntime>> onExit;
    private final BooleanSupplier completionCondition;
    private final Enum<?> followupState;

    private SuperstructureStateDef(List<SuperstructureStateStep> steps,
                                   List<Consumer<SuperstructureRuntime>> onEnter,
                                   List<Consumer<SuperstructureRuntime>> onExit,
                                   BooleanSupplier completionCondition,
                                   Enum<?> followupState) {
        this.steps = List.copyOf(steps);
        this.onEnter = List.copyOf(onEnter);
        this.onExit = List.copyOf(onExit);
        this.completionCondition = SupplierUtil.wrapBoolean(completionCondition, true);
        this.followupState = followupState;
    }

    public Session createSession() {
        return new Session();
    }

    Optional<Enum<?>> followupState() {
        return Optional.ofNullable(followupState);
    }

    public static final class Builder {

        private final List<SuperstructureStateStep> steps = new ArrayList<>();
        private final List<Consumer<SuperstructureRuntime>> onEnter = new ArrayList<>();
        private final List<Consumer<SuperstructureRuntime>> onExit = new ArrayList<>();
        private final List<DefaultCommand> defaultCommands = new ArrayList<>();
        private BooleanSupplier completionCondition = () -> true;
        private Enum<?> followup;

        public Builder withSetpoint(String key, Enum<?> state) {
            return withSetpoint(key, state, () -> true);
        }

        public Builder withSetpoint(String key, Enum<?> state, BooleanSupplier condition) {
            defaultCommands.add(new DefaultCommand(key, Objects.requireNonNull(state),
                    SupplierUtil.wrapBoolean(condition, true)));
            return this;
        }

        public Builder composedOf(SuperstructureStateStep... steps) {
            return composedOf(Arrays.asList(steps));
        }

        public Builder composedOf(List<SuperstructureStateStep> steps) {
            this.steps.addAll(Objects.requireNonNull(steps));
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

        public Builder advanceWhen(BooleanSupplier condition) {
            completionCondition = SupplierUtil.wrapBoolean(condition, true);
            return this;
        }

        public Builder followedBy(Enum<?> state) {
            followup = Objects.requireNonNull(state);
            return this;
        }

        public SuperstructureStateDef build() {
            List<SuperstructureStateStep> compiledSteps = new ArrayList<>(steps);

            if (!defaultCommands.isEmpty()) {
                SuperstructureStateStep.Builder defaultStep = SuperstructureStateStep.create();
                for (DefaultCommand command : defaultCommands) {
                    defaultStep.setpoint(command.key, command.state, command.condition);
                }
                defaultStep.await(completionCondition);
                compiledSteps.add(defaultStep.build());
                completionCondition = () -> true;
            }

            if (compiledSteps.isEmpty()) {
                // No commands were provided; the state is effectively a wait until completion condition.
                SuperstructureStateStep emptyStep = SuperstructureStateStep.create()
                        .await(completionCondition)
                        .build();
                compiledSteps.add(emptyStep);
                completionCondition = () -> true;
            }

            return new SuperstructureStateDef(compiledSteps, onEnter, onExit, completionCondition, followup);
        }

        private record DefaultCommand(String key, Enum<?> state, BooleanSupplier condition) {}
    }

    public final class Session {
        private int currentStep = -1;
        private boolean stepsFinished = false;
        private boolean followupQueued = false;
        private boolean exited = false;

        private void startNextStep(SuperstructureRuntime runtime) {
            currentStep++;
            if (currentStep >= steps.size()) {
                stepsFinished = true;
                return;
            }
            steps.get(currentStep).enter(runtime);
        }

        public void enter(SuperstructureRuntime runtime) {
            onEnter.forEach(action -> action.accept(runtime));
            startNextStep(runtime);
        }

        public void update(SuperstructureRuntime runtime) {
            if (stepsFinished) {
                return;
            }
            SuperstructureStateStep step = steps.get(currentStep);
            if (step.isComplete()) {
                step.exit(runtime);
                startNextStep(runtime);
            }
        }

        public boolean isSatisfied() {
            return stepsFinished && completionCondition.getAsBoolean();
        }

        public boolean shouldQueueFollowup() {
            return !followupQueued && isSatisfied() && followupState != null;
        }

        public Enum<?> followupState() {
            followupQueued = true;
            return followupState;
        }

        public void exit(SuperstructureRuntime runtime) {
            if (!exited) {
                onExit.forEach(action -> action.accept(runtime));
                exited = true;
            }
        }
    }
}
