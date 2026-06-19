package ca.frc6390.athena.api.mechanism;

import java.util.Optional;

import ca.frc6390.athena.api.mechanism.behavior.BehaviorConfigurer;
import ca.frc6390.athena.api.mechanism.behavior.MechanismBehavior;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.encoder.EncodersConfigurer;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoders;
import ca.frc6390.athena.api.mechanism.identity.IdentityConfig;
import ca.frc6390.athena.api.mechanism.identity.IdentityConfigurer;
import ca.frc6390.athena.api.mechanism.input.InputsConfigurer;
import ca.frc6390.athena.api.mechanism.input.MechanismInputs;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotors;
import ca.frc6390.athena.api.mechanism.motor.MotorsConfigurer;

public class FlowMechanismConfig implements MechanismConfig {
    private String name;
    private boolean disabled;
    private Optional<Class<?>> stateType = Optional.empty();
    private Optional<String> initialStateName = Optional.empty();
    private Optional<Object> initialState = Optional.empty();
    private double stateMachineDelaySeconds;
    private final IdentityConfig identity;
    private final MechanismMotors motors;
    private final MechanismEncoders encoders;
    private final MechanismInputs inputs;
    private final MechanismBehavior behavior;

    public FlowMechanismConfig(String name) {
        this.name = name != null ? name : "";
        this.identity = IdentityConfig.create();
        this.motors = MechanismMotors.create();
        this.encoders = MechanismEncoders.create();
        this.inputs = MechanismInputs.create();
        this.behavior = MechanismBehavior.create();
    }

    public static FlowMechanismConfig fromDefinition(MechanismDefinition definition) {
        FlowMechanismConfig config = new FlowMechanismConfig(definition.name());
        config.disabled = definition.disabled();
        config.stateType = definition.stateType();
        config.initialStateName = definition.initialStateName();
        config.initialState = definition.initialState();
        config.stateMachineDelaySeconds = definition.stateMachineDelaySeconds();
        config.identity.merge(IdentityConfig.from(definition.identity()));
        config.motors.merge(MechanismMotors.from(definition.motors()));
        config.encoders.merge(MechanismEncoders.from(definition.encoders()));
        config.inputs.merge(MechanismInputs.from(definition.inputs()));
        config.behavior.merge(MechanismBehavior.from(definition.loops(), definition.automation()));
        return config;
    }

    public FlowMechanismConfig named(String name) {
        this.name = name != null ? name : "";
        return this;
    }

    public FlowMechanismConfig disabled(boolean disabled) {
        this.disabled = disabled;
        this.identity.disabled(disabled);
        return this;
    }

    public FlowMechanismConfig stateMachineDelaySeconds(double delaySeconds) {
        this.stateMachineDelaySeconds = delaySeconds;
        return this;
    }

    public FlowMechanismConfig identity(IdentityConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(identity);
        }
        return this;
    }

    public FlowMechanismConfig motors(MotorsConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(motors);
        }
        return this;
    }

    public FlowMechanismConfig encoders(EncodersConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(encoders);
        }
        return this;
    }

    public FlowMechanismConfig inputs(InputsConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(inputs);
        }
        return this;
    }

    public FlowMechanismConfig behavior(BehaviorConfigurer configurer) {
        if (configurer != null) {
            configurer.apply(behavior);
        }
        return this;
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public boolean disabled() {
        return disabled || identity.definition().disabled();
    }

    @Override
    public void identity(IdentityConfig identity) {
        this.identity.merge(identity);
    }

    @Override
    public void motors(MechanismMotors motors) {
        this.motors.merge(motors);
    }

    @Override
    public void encoders(MechanismEncoders encoders) {
        this.encoders.merge(encoders);
    }

    @Override
    public void inputs(MechanismInputs inputs) {
        this.inputs.merge(inputs);
    }

    @Override
    public void behavior(MechanismBehavior behavior) {
        this.behavior.merge(behavior);
    }

    public IdentityConfig identitySection() {
        return identity;
    }

    public MechanismMotors motorsSection() {
        return motors;
    }

    public MechanismEncoders encodersSection() {
        return encoders;
    }

    public MechanismInputs inputsSection() {
        return inputs;
    }

    public MechanismBehavior behaviorSection() {
        return behavior;
    }

    protected void state(Optional<Class<?>> stateType, Optional<String> initialStateName) {
        this.stateType = stateType;
        this.initialStateName = initialStateName;
    }

    protected void initialState(Optional<Object> initialState) {
        this.initialState = initialState != null ? initialState : Optional.empty();
    }

    protected void stateMachineDelaySecondsInternal(double delaySeconds) {
        this.stateMachineDelaySeconds = delaySeconds;
    }

    @Override
    public MechanismDefinition definition() {
        return new MechanismDefinition(
            name(),
            disabled(),
            stateType,
            initialStateName,
            initialState,
            stateMachineDelaySeconds,
            identity.definition(),
            motors.definitions(),
            encoders.definitions(),
            inputs.definitions(),
            behavior.loopDefinitions(),
            behavior.automationDefinitions());
    }
}
