package ca.frc6390.athena.superstructure.spec;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.mechanism.spec.MechanismStateSpec;
import ca.frc6390.athena.runtime.validation.ValidationReport;

/**
 * Immutable superstructure spec.
 *
 * @param name superstructure name
 * @param parts mechanism parts
 * @param states superstructure states
 */
public record SuperstructureSpec(
        String name,
        List<SuperstructurePartSpec> parts,
        List<SuperstructureStateSpec> states) {
    public SuperstructureSpec {
        name = name == null || name.isBlank() ? "superstructure" : name;
        parts = List.copyOf(parts);
        states = List.copyOf(states);
    }

    /**
     * Validates using global context.
     *
     * @return validation report
     */
    public ValidationReport validate() {
        return validate(AthenaValidationContext.global());
    }

    /**
     * Validates this superstructure and child mechanisms.
     *
     * @param context validation context
     * @return validation report
     */
    public ValidationReport validate(AthenaValidationContext context) {
        ValidationReport.Builder report = ValidationReport.builder();
        Set<String> partNames = parts.stream().map(SuperstructurePartSpec::name).collect(Collectors.toSet());
        if (parts.isEmpty()) {
            report.error("superstructure.no-parts", name, "Superstructure must declare at least one part.");
        }
        if (partNames.size() != parts.size()) {
            report.error("superstructure.duplicate-part", name + ".parts", "Superstructure part names must be unique.");
        }
        Set<String> stateNames = states.stream().map(SuperstructureStateSpec::name).collect(Collectors.toSet());
        if (stateNames.size() != states.size()) {
            report.error("superstructure.duplicate-state", name + ".states", "Superstructure state names must be unique.");
        }
        for (SuperstructurePartSpec part : parts) {
            validatePart(context, report, part);
        }
        for (SuperstructureStateSpec state : states) {
            for (Map.Entry<String, String> target : state.partTargets().entrySet()) {
                String partName = target.getKey();
                if (!partNames.contains(partName)) {
                    report.error(
                            "superstructure.unknown-part",
                            name + "." + state.name() + "." + partName,
                            "State references unknown part " + partName + ".");
                } else {
                    validatePartTarget(report, state.name(), partName, target.getValue());
                }
            }
        }
        return report.build();
    }

    private void validatePart(AthenaValidationContext context, ValidationReport.Builder report, SuperstructurePartSpec part) {
        if (part.kind() == SuperstructurePartSpec.Kind.MECHANISM) {
            report.addAll(part.mechanism().validate(context));
        } else {
            report.addAll(part.superstructure().validate(context));
        }
    }

    private void validatePartTarget(
            ValidationReport.Builder report,
            String stateName,
            String partName,
            String targetStateName) {
        SuperstructurePartSpec part = parts.stream()
                .filter(candidate -> candidate.name().equals(partName))
                .findFirst()
                .orElseThrow();
        Set<String> childStateNames = part.kind() == SuperstructurePartSpec.Kind.MECHANISM
                ? part.mechanism().states().stream().map(MechanismStateSpec::name).collect(Collectors.toSet())
                : part.superstructure().states().stream().map(SuperstructureStateSpec::name).collect(Collectors.toSet());
        if (!childStateNames.isEmpty() && !childStateNames.contains(targetStateName)) {
            report.error(
                    "superstructure.unknown-target",
                    name + "." + stateName + "." + partName,
                    "State references unknown target " + targetStateName + " for part " + partName + ".");
        }
    }
}
