package ca.frc6390.athena.drivetrain.spec;

import java.util.List;

import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.runtime.validation.ValidationReport;

/**
 * Immutable differential drivetrain declaration.
 *
 * @param name drivetrain name
 * @param leftMotors left side motors
 * @param rightMotors right side motors
 * @param trackWidth track width
 */
public record DifferentialDrivetrainSpec(
        String name,
        List<MotorSpec> leftMotors,
        List<MotorSpec> rightMotors,
        TrackWidth trackWidth) {
    public DifferentialDrivetrainSpec {
        name = name == null || name.isBlank() ? "drivetrain" : name;
        leftMotors = List.copyOf(leftMotors);
        rightMotors = List.copyOf(rightMotors);
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
     * Validates backend availability and percent-output capability.
     *
     * @param context validation context
     * @return validation report
     */
    public ValidationReport validate(AthenaValidationContext context) {
        ValidationReport.Builder report = ValidationReport.builder();
        if (leftMotors.isEmpty()) {
            report.error("drivetrain.no-left-motors", name + ".left", "Differential drivetrain needs left motors.");
        }
        if (rightMotors.isEmpty()) {
            report.error("drivetrain.no-right-motors", name + ".right", "Differential drivetrain needs right motors.");
        }
        leftMotors.forEach(motor -> validateMotor(context, report, motor));
        rightMotors.forEach(motor -> validateMotor(context, report, motor));
        return report.build();
    }

    private void validateMotor(AthenaValidationContext context, ValidationReport.Builder report, MotorSpec motor) {
        MotorBackend backend = context.backendRegistry().motorBackendFor(motor.kind()).orElse(null);
        if (backend == null) {
            report.error("hardware.missing-backend", motor.path(), "No motor backend installed for " + motor.kind().key() + ".");
            return;
        }
        if (!backend.capabilities(motor.kind()).contains(MotorCapability.PERCENT_OUTPUT)) {
            report.error(
                    "hardware.missing-capability",
                    motor.path(),
                    motor.kind().key() + " does not provide required drivetrain percent output.");
        }
    }
}
