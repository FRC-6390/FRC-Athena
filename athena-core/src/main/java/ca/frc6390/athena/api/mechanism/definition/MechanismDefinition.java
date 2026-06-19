package ca.frc6390.athena.api.mechanism.definition;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

import ca.frc6390.athena.api.mechanism.identity.MechanismIdentity;

public record MechanismDefinition(
    String name,
    boolean disabled,
    Optional<Class<?>> stateType,
    Optional<String> initialStateName,
    Optional<Object> initialState,
    double stateMachineDelaySeconds,
    MechanismIdentity identity,
    List<MechanismMotorDefinition> motors,
    List<MechanismEncoderDefinition> encoders,
    List<MechanismInputDefinition> inputs,
    List<MechanismLoopDefinition> loops,
    List<MechanismAutomationDefinition> automation
) {
    public MechanismDefinition {
        name = Objects.requireNonNull(name, "name");
        stateType = Objects.requireNonNull(stateType, "stateType");
        initialStateName = Objects.requireNonNull(initialStateName, "initialStateName");
        initialState = Objects.requireNonNull(initialState, "initialState");
        identity = Objects.requireNonNull(identity, "identity");
        motors = List.copyOf(Objects.requireNonNull(motors, "motors"));
        encoders = List.copyOf(Objects.requireNonNull(encoders, "encoders"));
        inputs = List.copyOf(Objects.requireNonNull(inputs, "inputs"));
        loops = List.copyOf(Objects.requireNonNull(loops, "loops"));
        automation = List.copyOf(Objects.requireNonNull(automation, "automation"));
    }

    public MechanismDefinition(
            String name,
            boolean disabled,
            Optional<Class<?>> stateType,
            Optional<String> initialStateName,
            double stateMachineDelaySeconds,
            MechanismIdentity identity,
            List<MechanismMotorDefinition> motors,
            List<MechanismEncoderDefinition> encoders,
            List<MechanismInputDefinition> inputs,
            List<MechanismLoopDefinition> loops,
            List<MechanismAutomationDefinition> automation) {
        this(
            name,
            disabled,
            stateType,
            initialStateName,
            Optional.empty(),
            stateMachineDelaySeconds,
            identity,
            motors,
            encoders,
            inputs,
            loops,
            automation);
    }
}
