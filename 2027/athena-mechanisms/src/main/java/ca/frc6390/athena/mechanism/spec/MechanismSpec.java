package ca.frc6390.athena.mechanism.spec;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;
import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.input.InputSpec;
import ca.frc6390.athena.hardware.spec.AthenaValidationContext;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.runtime.validation.ValidationReport;

/**
 * Immutable normalized mechanism declaration.
 *
 * @param name mechanism name
 * @param motors lowered motor specs
 * @param encoders lowered encoder specs
 * @param inputs lowered input specs
 * @param positionSource named encoder used for position, or {@code null}
 * @param velocitySource named encoder used for velocity, or {@code null}
 * @param control requested control declaration
 * @param states named mechanism states
 */
public record MechanismSpec(
        String name,
        List<MotorSpec> motors,
        List<EncoderSpec> encoders,
        List<InputSpec> inputs,
        String positionSource,
        String velocitySource,
        ControlSpec control,
        List<MechanismStateSpec> states) {
    public MechanismSpec {
        name = name == null || name.isBlank() ? "mechanism" : name;
        motors = List.copyOf(motors);
        encoders = List.copyOf(encoders);
        inputs = List.copyOf(inputs);
        control = control == null ? ControlSpec.none() : control;
        states = List.copyOf(states);
    }

    /**
     * Returns requested control mode.
     *
     * @return control mode
     */
    public ControlMode controlMode() {
        return control.mode();
    }

    /**
     * Validates this spec using the global validation context.
     *
     * @return validation report
     */
    public ValidationReport validate() {
        return validate(AthenaValidationContext.global());
    }

    /**
     * Validates this spec using an explicit context.
     *
     * @param context validation context
     * @return validation report
     */
    public ValidationReport validate(AthenaValidationContext context) {
        ValidationReport.Builder report = ValidationReport.builder();
        if (motors.isEmpty()) {
            report.error("mechanism.no-motors", name, "Mechanism must declare at least one motor.");
        }
        for (MotorSpec motor : motors) {
            validateMotor(context, report, motor);
        }
        validateEncoderSources(report);
        for (EncoderSpec encoder : encoders) {
            validateEncoder(report, encoder);
        }
        for (InputSpec input : inputs) {
            validateInput(report, input);
        }
        validateControl(report);
        validateStates(report);
        return report.build();
    }

    private void validateControl(ValidationReport.Builder report) {
        control.pidSpec().ifPresent(pid -> {
            if (!pid.isFinite()) {
                report.error("control.invalid-pid", name + ".control.pid", "PID gains must be finite.");
            }
        });
        control.feedforwardSpec().ifPresent(feedforward -> {
            if (!feedforward.isFinite()) {
                report.error(
                        "control.invalid-feedforward",
                        name + ".control.feedforward",
                        "Feedforward gains must be finite.");
            }
        });
    }

    private void validateStates(ValidationReport.Builder report) {
        Set<String> stateNames = states.stream().map(MechanismStateSpec::name).collect(Collectors.toSet());
        if (stateNames.size() != states.size()) {
            report.error("mechanism.duplicate-state", name + ".states", "Mechanism state names must be unique.");
        }
        for (MechanismStateSpec state : states) {
            state.targetValue().ifPresent(target -> {
                if (!Double.isFinite(target)) {
                    report.error("mechanism.invalid-state-target", name + "." + state.name(), "State target must be finite.");
                }
            });
        }
    }

    private void validateEncoderSources(ValidationReport.Builder report) {
        Set<String> encoderNames = encoders.stream().map(EncoderSpec::name).collect(Collectors.toSet());
        if (positionSource != null && !encoderNames.contains(positionSource)) {
            report.error(
                    "mechanism.unknown-position-source",
                    name + ".positionSource",
                    "Position source references unknown encoder " + positionSource + ".");
        }
        if (velocitySource != null && !encoderNames.contains(velocitySource)) {
            report.error(
                    "mechanism.unknown-velocity-source",
                    name + ".velocitySource",
                    "Velocity source references unknown encoder " + velocitySource + ".");
        }
    }

    private void validateEncoder(ValidationReport.Builder report, EncoderSpec encoder) {
        if (encoder.id() < 0) {
            report.error("encoder.invalid-id", encoder.path(), "Encoder id must be non-negative.");
        }
        if (!Double.isFinite(encoder.gearRatio()) || encoder.gearRatio() <= 0.0) {
            report.error("encoder.invalid-gear-ratio", encoder.path(), "Encoder gear ratio must be positive.");
        }
        if (!Double.isFinite(encoder.offset())) {
            report.error("encoder.invalid-offset", encoder.path(), "Encoder offset must be finite.");
        }
    }

    private void validateInput(ValidationReport.Builder report, InputSpec input) {
        if ((input.sourceKind() == InputSourceKind.DIGITAL_CHANNEL || input.sourceKind() == InputSourceKind.ANALOG_CHANNEL)
                && input.channel() < 0) {
            report.error("input.invalid-channel", input.path(), "Hardware input channel must be non-negative.");
        }
        if (input.sourceKind() == InputSourceKind.RUNTIME_SUPPLIER && input.label().isBlank()) {
            report.error("input.missing-runtime-label", input.path(), "Runtime input must include a source label.");
        }
    }

    private void validateMotor(AthenaValidationContext context, ValidationReport.Builder report, MotorSpec motor) {
        MotorBackend backend = context.backendRegistry()
                .motorBackendFor(motor.kind())
                .orElse(null);
        if (backend == null) {
            report.error(
                    "hardware.missing-backend",
                    motor.path(),
                    "No motor backend installed for " + motor.kind().key() + ".");
            return;
        }
        MotorCapability required = requiredCapability();
        if (required != null && !backend.capabilities(motor.kind()).contains(required)) {
            report.error(
                    "hardware.missing-capability",
                    motor.path(),
                    motor.kind().key() + " does not provide required capability " + required + ".");
        }
        if (motor.integratedEncoder()
                && !backend.capabilities(motor.kind()).contains(MotorCapability.INTEGRATED_ENCODER)) {
            report.error(
                    "hardware.missing-integrated-encoder",
                    motor.path(),
                    motor.kind().key() + " does not report an integrated encoder.");
        }
    }

    private MotorCapability requiredCapability() {
        return switch (control.mode()) {
            case PERCENT_OUTPUT -> MotorCapability.PERCENT_OUTPUT;
            case POSITION -> MotorCapability.POSITION_CLOSED_LOOP;
            case VELOCITY -> MotorCapability.VELOCITY_CLOSED_LOOP;
            case NONE -> null;
        };
    }
}
