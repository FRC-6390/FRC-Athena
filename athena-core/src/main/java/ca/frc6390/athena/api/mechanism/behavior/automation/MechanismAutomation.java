package ca.frc6390.athena.api.mechanism.behavior.automation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationPhase;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class MechanismAutomation {
    private final List<MechanismAutomationDefinition> definitions = new ArrayList<>();

    private MechanismAutomation() {
    }

    public static MechanismAutomation create() {
        return new MechanismAutomation();
    }

    public static MechanismAutomation from(List<MechanismAutomationDefinition> automation) {
        MechanismAutomation result = create();
        if (automation != null) {
            result.definitions.addAll(automation);
        }
        return result;
    }

    public <S> MechanismAutomation onStateEnter(
            MechanismStateHookCallback callback,
            S... states) {
        return add(MechanismAutomationPhase.STATE_ENTER, callback, states);
    }

    public <S> MechanismAutomation onStatePeriodic(
            MechanismStateHookCallback callback,
            S... states) {
        return add(MechanismAutomationPhase.STATE_PERIODIC, callback, states);
    }

    public <S> MechanismAutomation onStateExit(
            MechanismStateHookCallback callback,
            S... states) {
        return add(MechanismAutomationPhase.STATE_EXIT, callback, states);
    }

    public MechanismAutomation add(MechanismAutomationDefinition definition) {
        definitions.add(Objects.requireNonNull(definition, "definition"));
        return this;
    }

    public MechanismAutomation merge(MechanismAutomation other) {
        if (other != null) {
            definitions.addAll(other.definitions);
        }
        return this;
    }

    public List<MechanismAutomationDefinition> definitions() {
        return List.copyOf(definitions);
    }

    @SafeVarargs
    private final <S> MechanismAutomation add(
            MechanismAutomationPhase phase,
            MechanismStateHookCallback callback,
            S... states) {
        Objects.requireNonNull(phase, "phase");
        Objects.requireNonNull(callback, "callback");
        Objects.requireNonNull(states, "states");
        if (states.length == 0) {
            throw new IllegalArgumentException("states must contain at least one state");
        }
        List<String> stateNames = Arrays.stream(states)
                .filter(Objects::nonNull)
                .map(StateNames::name)
                .toList();
        if (stateNames.isEmpty()) {
            throw new IllegalArgumentException("states must contain at least one non-null state");
        }
        definitions.add(new MechanismAutomationDefinition(phase, stateNames, callback));
        return this;
    }
}
