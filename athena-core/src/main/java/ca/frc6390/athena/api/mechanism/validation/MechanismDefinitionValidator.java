package ca.frc6390.athena.api.mechanism.validation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.definition.MechanismAutomationDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismFeedforwardControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismPidControllerDefinition;
import ca.frc6390.athena.mechanisms.statespec.StateId;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class MechanismDefinitionValidator {
    private MechanismDefinitionValidator() {
    }

    public static List<MechanismValidationIssue> validate(MechanismDefinition definition) {
        List<MechanismValidationIssue> issues = new ArrayList<>();
        Set<String> knownStates = knownStates(definition, issues);

        if (definition.stateType().isPresent() != definition.initialStateName().isPresent()) {
            issues.add(new MechanismValidationIssue(
                "state.metadata.incomplete",
                ValidationSeverity.ERROR,
                "Stateful mechanisms must declare both stateType and initialStateName together."));
        }

        definition.initialStateName().ifPresent(initialState -> {
            if (!knownStates.isEmpty() && !knownStates.contains(initialState)) {
                issues.add(new MechanismValidationIssue(
                    "state.initialState.unknown",
                    ValidationSeverity.ERROR,
                    "Initial state is not declared by the state enum: " + initialState));
            }
        });

        if (definition.identity().continuousRotation() && definition.identity().travelRange().isPresent()) {
            issues.add(new MechanismValidationIssue(
                "identity.travelRange.continuousRotationConflict",
                ValidationSeverity.ERROR,
                "Continuous rotation cannot be combined with a bounded travel range."));
        }

        long defaultPositionSources = definition.encoders().stream()
            .filter(ca.frc6390.athena.api.mechanism.definition.MechanismEncoderDefinition::defaultPositionSource)
            .count();
        if (defaultPositionSources > 1) {
            issues.add(new MechanismValidationIssue(
                "encoder.defaultPosition.duplicate",
                ValidationSeverity.ERROR,
                "Only one encoder can be the default position source."));
        }

        Set<String> loopNames = new HashSet<>();
        definition.loops().forEach(loop -> {
            if (!loopNames.add(loop.name())) {
                issues.add(new MechanismValidationIssue(
                    "loop.name.duplicate",
                    ValidationSeverity.ERROR,
                    "Loop names must be unique: " + loop.name()));
            }

            validateStateNames(issues, knownStates, loop.activation().states(), "loop.state.unknown", loop.name());

            if (loop.activation().mode() == LoopMode.MANUAL && !loop.activation().states().isEmpty()) {
                issues.add(new MechanismValidationIssue(
                    "loop.schedule.manualStateConflict",
                    ValidationSeverity.ERROR,
                    "Manual loops cannot declare active states: " + loop.name()));
            }

            if (loop.controller() instanceof MechanismPidControllerDefinition pid
                    && pid.tolerance().isPresent()
                    && pid.tolerance().getAsDouble() < 0.0) {
                issues.add(new MechanismValidationIssue(
                    "loop.pid.tolerance.invalid",
                    ValidationSeverity.ERROR,
                    "PID tolerance cannot be negative: " + loop.name()));
            }

            if (loop.controller() instanceof MechanismFeedforwardControllerDefinition feedforward) {
                feedforward.tolerance().ifPresent(tolerance -> {
                    if (tolerance < 0.0) {
                        issues.add(new MechanismValidationIssue(
                            "loop.feedforward.tolerance.invalid",
                            ValidationSeverity.ERROR,
                            "Feedforward tolerance cannot be negative: " + loop.name()));
                    }
                });
            }
        });

        if (!definition.automation().isEmpty() && definition.stateType().isEmpty()) {
            issues.add(new MechanismValidationIssue(
                "automation.stateType.required",
                ValidationSeverity.ERROR,
                "State automation requires a stateful mechanism definition."));
        }

        definition.automation().forEach(automation -> validateAutomation(issues, knownStates, automation));

        return List.copyOf(issues);
    }

    private static void validateAutomation(
            List<MechanismValidationIssue> issues,
            Set<String> knownStates,
            MechanismAutomationDefinition automation) {
        if (automation.states().isEmpty()) {
            issues.add(new MechanismValidationIssue(
                "automation.states.empty",
                ValidationSeverity.ERROR,
                "Automation hooks must declare at least one state."));
            return;
        }
        validateStateNames(issues, knownStates, automation.states(), "automation.state.unknown", automation.phase().name());
    }

    private static void validateStateNames(
            List<MechanismValidationIssue> issues,
            Set<String> knownStates,
            List<String> states,
            String code,
            String owner) {
        if (states.isEmpty() || knownStates.isEmpty()) {
            return;
        }
        for (String state : states) {
            if (state == null || state.isBlank() || knownStates.contains(state)) {
                continue;
            }
            issues.add(new MechanismValidationIssue(
                code,
                ValidationSeverity.ERROR,
                "Unknown state '" + state + "' referenced by " + owner));
        }
    }

    private static Set<String> knownStates(MechanismDefinition definition, List<MechanismValidationIssue> issues) {
        if (definition.initialState().orElse(null) instanceof StateId stateId) {
            Set<String> states = new HashSet<>();
            stateId.owner().states().forEach(state -> states.add(state.name()));
            return states;
        }
        if (definition.stateType().isEmpty()) {
            return Set.of();
        }
        Class<?> rawType = definition.stateType().orElseThrow();
        if (!rawType.isEnum()) {
            return Set.of();
        }
        Object[] constants = rawType.getEnumConstants();
        if (constants == null || constants.length == 0) {
            return Set.of();
        }
        Set<String> states = new HashSet<>();
        Arrays.stream(constants)
            .filter(Enum.class::isInstance)
            .map(StateNames::name)
            .forEach(states::add);
        return states;
    }
}
