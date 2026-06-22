package ca.frc6390.athena.drivetrain.spec;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.runtime.validation.ValidationReport;

/**
 * Immutable swerve drivetrain declaration.
 *
 * @param name drivetrain name
 * @param modules swerve modules
 * @param trackWidth track width
 * @param wheelBase wheelbase
 */
public record SwerveDrivetrainSpec(
        String name,
        List<SwerveModuleSpec> modules,
        TrackWidth trackWidth,
        WheelBase wheelBase) {
    public SwerveDrivetrainSpec {
        name = name == null || name.isBlank() ? "swerve" : name;
        modules = List.copyOf(modules);
        trackWidth = trackWidth == null ? TrackWidth.meters(0.6) : trackWidth;
        wheelBase = wheelBase == null ? WheelBase.meters(0.6) : wheelBase;
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
     * Validates module completeness, gains, geometry, and motor backends.
     *
     * @param context validation context
     * @return validation report
     */
    public ValidationReport validate(AthenaValidationContext context) {
        ValidationReport.Builder report = ValidationReport.builder();
        if (modules.isEmpty()) {
            report.error("swerve.no-modules", name, "Swerve drivetrain needs at least one module.");
        }
        Set<String> names = new HashSet<>();
        for (SwerveModuleSpec module : modules) {
            validateModule(context, report, names, module);
        }
        return report.build();
    }

    private void validateModule(
            AthenaValidationContext context,
            ValidationReport.Builder report,
            Set<String> names,
            SwerveModuleSpec module) {
        String path = module.path(name);
        if (!names.add(module.name())) {
            report.error("swerve.duplicate-module", path, "Swerve module names must be unique.");
        }
        if (!Double.isFinite(module.xMeters()) || !Double.isFinite(module.yMeters())) {
            report.error("swerve.invalid-location", path, "Swerve module location must be finite.");
        }
        if (!module.control().isFinite()) {
            report.error("swerve.invalid-gains", path, "Swerve module gains must be finite.");
        }
        validateMotor(context, report, module.driveMotor(), path + ".drive", MotorCapability.VELOCITY_CLOSED_LOOP);
        validateMotor(context, report, module.steerMotor(), path + ".steer", MotorCapability.POSITION_CLOSED_LOOP);
        if (module.steerEncoder() == null) {
            report.error("swerve.missing-encoder", path + ".encoder", "Swerve module needs a steer encoder.");
        }
    }

    private void validateMotor(
            AthenaValidationContext context,
            ValidationReport.Builder report,
            MotorSpec motor,
            String path,
            MotorCapability capability) {
        if (motor == null) {
            report.error("swerve.missing-motor", path, "Swerve module motor is required.");
            return;
        }
        MotorBackend backend = context.backendRegistry().motorBackendFor(motor.kind()).orElse(null);
        if (backend == null) {
            report.error("hardware.missing-backend", motor.path(), "No motor backend installed for " + motor.kind().key() + ".");
            return;
        }
        if (!backend.capabilities(motor.kind()).contains(capability)) {
            report.error("hardware.missing-capability", motor.path(), motor.kind().key() + " does not provide " + capability + ".");
        }
    }
}
