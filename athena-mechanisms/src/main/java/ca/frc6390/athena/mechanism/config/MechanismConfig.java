package ca.frc6390.athena.mechanism.config;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

import ca.frc6390.athena.hardware.config.MotorConfig;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;
import ca.frc6390.athena.hardware.input.InputConfig;
import ca.frc6390.athena.hardware.input.InputSpec;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.mechanism.spec.ControlSpec;
import ca.frc6390.athena.mechanism.spec.MechanismStateSpec;
import ca.frc6390.athena.mechanism.spec.MechanismSpec;

/**
 * Student-facing mechanism declaration.
 */
public final class MechanismConfig {
    private final String name;
    private final List<NamedMotor> motors = new ArrayList<>();
    private final List<NamedEncoder> encoders = new ArrayList<>();
    private final List<NamedInput> inputs = new ArrayList<>();
    private final List<MechanismStateConfig> states = new ArrayList<>();
    private final ControlConfig control = new ControlConfig();
    private String positionSource;
    private String velocitySource;

    MechanismConfig(String name) {
        this.name = name == null || name.isBlank() ? "mechanism" : name;
    }

    /**
     * Adds a motor to this mechanism.
     *
     * @param name motor name
     * @param configure motor configuration callback
     * @return this config
     */
    public MechanismConfig motor(String name, Consumer<MotorConfig> configure) {
        MotorConfig motor = MotorConfig.create();
        if (configure != null) {
            configure.accept(motor);
        }
        motors.add(new NamedMotor(name, motor));
        return this;
    }

    /**
     * Adds a referenced motor using its stable default name.
     *
     * @param motor motor reference
     * @return this config
     */
    public MechanismConfig motor(MotorRef motor) {
        Objects.requireNonNull(motor, "motor");
        return motor(motor.defaultName(), motor);
    }

    /**
     * Adds a referenced motor.
     *
     * @param name motor name
     * @param motor motor reference
     * @return this config
     */
    public MechanismConfig motor(String name, MotorRef motor) {
        Objects.requireNonNull(motor, "motor");
        return motor(name, config -> config.hardware(motor));
    }

    /**
     * Adds an input to this mechanism.
     *
     * @param name input name
     * @param configure input configuration callback
     * @return this config
     */
    public MechanismConfig input(String name, Consumer<InputConfig> configure) {
        InputConfig input = InputConfig.create();
        if (configure != null) {
            configure.accept(input);
        }
        inputs.add(new NamedInput(name, input));
        return this;
    }

    /**
     * Adds an encoder to this mechanism.
     *
     * @param name encoder name
     * @param configure encoder configuration callback
     * @return this config
     */
    public MechanismConfig encoder(String name, Consumer<EncoderConfig> configure) {
        EncoderConfig encoder = EncoderConfig.create();
        if (configure != null) {
            configure.accept(encoder);
        }
        encoders.add(new NamedEncoder(name, encoder));
        return this;
    }

    /**
     * Adds a referenced encoder using its stable default name.
     *
     * @param encoder encoder reference
     * @return this config
     */
    public MechanismConfig encoder(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        return encoder(encoder.defaultName(), encoder);
    }

    /**
     * Adds a referenced encoder.
     *
     * @param name encoder name
     * @param encoder encoder reference
     * @return this config
     */
    public MechanismConfig encoder(String name, EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        return encoder(name, config -> config.hardware(encoder));
    }

    /**
     * Selects a named encoder as the position source.
     *
     * @param encoderName encoder name
     * @return this config
     */
    public MechanismConfig positionSource(String encoderName) {
        positionSource = encoderName;
        return this;
    }

    /**
     * Selects a referenced encoder as the position source.
     *
     * @param encoder encoder reference
     * @return this config
     */
    public MechanismConfig positionSource(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        positionSource = encoder.defaultName();
        return this;
    }

    /**
     * Selects a named encoder as the velocity source.
     *
     * @param encoderName encoder name
     * @return this config
     */
    public MechanismConfig velocitySource(String encoderName) {
        velocitySource = encoderName;
        return this;
    }

    /**
     * Selects a referenced encoder as the velocity source.
     *
     * @param encoder encoder reference
     * @return this config
     */
    public MechanismConfig velocitySource(EncoderRef encoder) {
        Objects.requireNonNull(encoder, "encoder");
        velocitySource = encoder.defaultName();
        return this;
    }

    /**
     * Configures the control mode.
     *
     * @param configure control configuration callback
     * @return this config
     */
    public MechanismConfig control(Consumer<ControlConfig> configure) {
        if (configure != null) {
            configure.accept(control);
        }
        return this;
    }

    /**
     * Adds a named mechanism state.
     *
     * @param name state name
     * @param configure state configuration callback
     * @return this config
     */
    public MechanismConfig state(String name, Consumer<MechanismStateConfig> configure) {
        MechanismStateConfig state = new MechanismStateConfig(name);
        if (configure != null) {
            configure.accept(state);
        }
        states.add(state);
        return this;
    }

    /**
     * Lowers this public declaration into an immutable mechanism spec.
     *
     * @return mechanism spec
     */
    public MechanismSpec toSpec() {
        List<MotorSpec> motorSpecs = motors.stream()
                .map(motor -> motor.config().toSpec(name, motor.name()))
                .toList();
        List<EncoderSpec> encoderSpecs = encoders.stream()
                .map(encoder -> encoder.config().toSpec(name, encoder.name()))
                .toList();
        List<InputSpec> inputSpecs = inputs.stream()
                .map(input -> input.config().toSpec(name, input.name()))
                .toList();
        ControlSpec controlSpec = control.toSpec();
        List<MechanismStateSpec> stateSpecs = states.stream().map(MechanismStateConfig::toSpec).toList();
        return new MechanismSpec(
                name,
                motorSpecs,
                encoderSpecs,
                inputSpecs,
                positionSource,
                velocitySource,
                controlSpec,
                stateSpecs);
    }

    private record NamedMotor(String name, MotorConfig config) {
        private NamedMotor {
            name = name == null || name.isBlank() ? "motor" : name;
            Objects.requireNonNull(config, "config");
        }
    }

    private record NamedEncoder(String name, EncoderConfig config) {
        private NamedEncoder {
            name = name == null || name.isBlank() ? "encoder" : name;
            Objects.requireNonNull(config, "config");
        }
    }

    private record NamedInput(String name, InputConfig config) {
        private NamedInput {
            name = name == null || name.isBlank() ? "input" : name;
            Objects.requireNonNull(config, "config");
        }
    }
}
