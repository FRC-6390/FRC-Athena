package ca.frc6390.athena.api.mechanism;

import java.util.Optional;

import ca.frc6390.athena.api.mechanism.behavior.MechanismBehavior;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoders;
import ca.frc6390.athena.api.mechanism.identity.IdentityConfig;
import ca.frc6390.athena.api.mechanism.input.MechanismInputs;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotors;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class MechanismSpec {
    private final FlowMechanismConfig config;

    public MechanismSpec() {
        this(new FlowMechanismConfig(""));
    }

    MechanismSpec(FlowMechanismConfig config) {
        this.config = config;
    }

    public MechanismSpec name(String name) {
        config.named(name);
        return this;
    }

    public MechanismSpec disabled(boolean disabled) {
        config.disabled(disabled);
        return this;
    }

    public MechanismSpec initialState(Object initialState) {
        config.state(
            Optional.ofNullable(initialState).map(Object::getClass),
            Optional.ofNullable(StateNames.name(initialState)));
        config.initialState(Optional.ofNullable(initialState));
        return this;
    }

    public MechanismSpec stateMachineDelaySeconds(double delaySeconds) {
        config.stateMachineDelaySecondsInternal(delaySeconds);
        return this;
    }

    public IdentityConfig identity() {
        return config.identitySection();
    }

    public MechanismMotors motors() {
        return config.motorsSection();
    }

    public MechanismEncoders encoders() {
        return config.encodersSection();
    }

    public MechanismInputs inputs() {
        return config.inputsSection();
    }

    public MechanismBehavior behavior() {
        return config.behaviorSection();
    }

    public MechanismDefinition definition() {
        return config.definition();
    }
}
