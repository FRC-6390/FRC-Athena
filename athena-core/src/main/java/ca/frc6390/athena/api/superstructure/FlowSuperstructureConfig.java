package ca.frc6390.athena.api.superstructure;

import java.util.List;

import ca.frc6390.athena.api.superstructure.behavior.BehaviorConfigurer;
import ca.frc6390.athena.api.superstructure.behavior.SuperstructureBehavior;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDefinition;
import ca.frc6390.athena.api.superstructure.input.InputsConfigurer;
import ca.frc6390.athena.api.superstructure.input.SuperstructureInputs;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class FlowSuperstructureConfig<S, SP> implements SuperstructureConfig<S, SP> {
    private String name;
    private final S initialState;
    private double stateMachineDelaySeconds;
    private final SuperstructureMechanisms<SP> mechanisms;
    private final SuperstructureInputs inputs;
    private final SuperstructureBehavior<S, SP> behavior;

    public FlowSuperstructureConfig(String name, S initialState) {
        this.name = name != null ? name : "";
        this.initialState = initialState;
        this.mechanisms = SuperstructureMechanisms.create();
        this.inputs = SuperstructureInputs.create();
        this.behavior = SuperstructureBehavior.create();
    }

    public static <S, SP> FlowSuperstructureConfig<S, SP> fromDefinition(
            SuperstructureDefinition<SP> definition,
            S initialState) {
        FlowSuperstructureConfig<S, SP> config = new FlowSuperstructureConfig<>(definition.name(), initialState);
        config.stateMachineDelaySeconds = definition.stateMachineDelaySeconds();
        config.mechanisms.merge(SuperstructureMechanisms.from(definition.children()));
        config.inputs.merge(SuperstructureInputs.from(definition.inputs()));
        config.behavior.merge(SuperstructureBehavior.from(
            definition.constraints(),
            definition.hooks(),
            definition.transitionHooks()));
        return config;
    }

    public FlowSuperstructureConfig<S, SP> named(String name) {
        this.name = name != null ? name : "";
        return this;
    }

    public FlowSuperstructureConfig<S, SP> stateMachineDelaySeconds(double delaySeconds) {
        this.stateMachineDelaySeconds = delaySeconds;
        return this;
    }

    public FlowSuperstructureConfig<S, SP> mechanisms(MechanismsConfigurer<SP> configurer) {
        if (configurer != null) {
            configurer.apply(mechanisms);
        }
        return this;
    }

    public FlowSuperstructureConfig<S, SP> inputs(InputsConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(inputs);
        }
        return this;
    }

    public FlowSuperstructureConfig<S, SP> behavior(BehaviorConfigurer<S, SP> configurer) {
        if (configurer != null) {
            configurer.apply(behavior);
        }
        return this;
    }

    @Override
    public S initialState() {
        return initialState;
    }

    @Override
    public double stateMachineDelaySeconds() {
        return stateMachineDelaySeconds;
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public void mechanisms(SuperstructureMechanisms<SP> mechanisms) {
        this.mechanisms.merge(mechanisms);
    }

    @Override
    public void inputs(SuperstructureInputs inputs) {
        this.inputs.merge(inputs);
    }

    @Override
    public void behavior(SuperstructureBehavior<S, SP> behavior) {
        this.behavior.merge(behavior);
    }

    public SuperstructureMechanisms<SP> mechanismsSection() {
        return mechanisms;
    }

    public SuperstructureInputs inputsSection() {
        return inputs;
    }

    public SuperstructureBehavior<S, SP> behaviorSection() {
        return behavior;
    }

    @Override
    public SuperstructureDefinition<SP> definition() {
        return new SuperstructureDefinition<>(
            name(),
            initialState().getClass(),
            StateNames.name(initialState()),
            java.util.Optional.of(initialState()),
            stateMachineDelaySeconds(),
            List.copyOf(mechanisms.definitions()),
            inputs.definitions(),
            behavior.constraintDefinitions(),
            behavior.hookDefinitions(),
            behavior.transitionDefinitions());
    }
}
