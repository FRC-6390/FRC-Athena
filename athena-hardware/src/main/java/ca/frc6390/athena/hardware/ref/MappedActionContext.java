package ca.frc6390.athena.hardware.ref;

import ca.frc6390.athena.hardware.backend.MotorDevice;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Map-backed action context for runtimes that resolve refs from registered handles.
 */
public final class MappedActionContext implements ActionContext {
    private final Map<EncoderRef, RuntimeEncoder> encoders = new LinkedHashMap<>();
    private final Map<MotorRef, RuntimeMotor> motors = new LinkedHashMap<>();
    private final Map<BooleanRef, RuntimeBoolean> booleans = new LinkedHashMap<>();
    private final Map<NumberRef, RuntimeNumber> numbers = new LinkedHashMap<>();

    /**
     * Registers an encoder runtime handle.
     *
     * @param ref encoder declaration
     * @param encoder runtime encoder
     * @return this context
     */
    public MappedActionContext encoder(EncoderRef ref, RuntimeEncoder encoder) {
        encoders.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(encoder, "encoder"));
        return this;
    }

    public boolean hasEncoder(EncoderRef ref) {
        return encoders.containsKey(ref);
    }

    /**
     * Registers a motor runtime handle.
     *
     * @param ref motor declaration
     * @param motor runtime motor
     * @return this context
     */
    public MappedActionContext motor(MotorRef ref, RuntimeMotor motor) {
        motors.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(motor, "motor"));
        return this;
    }

    public boolean hasMotor(MotorRef ref) {
        return motors.containsKey(ref);
    }

    /**
     * Registers a backend motor device.
     *
     * @param ref motor declaration
     * @param motor backend motor device
     * @return this context
     */
    public MappedActionContext motor(MotorRef ref, MotorDevice motor) {
        return motor(ref, RuntimeMotor.from(motor));
    }

    /**
     * Registers a boolean runtime handle.
     *
     * @param ref boolean declaration
     * @param value runtime boolean
     * @return this context
     */
    public MappedActionContext bool(BooleanRef ref, RuntimeBoolean value) {
        booleans.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(value, "value"));
        BooleanRef.bindRuntime(ref, value::get);
        return this;
    }

    public boolean hasBoolean(BooleanRef ref) {
        return booleans.containsKey(ref);
    }

    /**
     * Registers a numeric runtime handle.
     *
     * @param ref number declaration
     * @param value runtime number
     * @return this context
     */
    public MappedActionContext number(NumberRef ref, RuntimeNumber value) {
        numbers.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(value, "value"));
        return this;
    }

    public boolean hasNumber(NumberRef ref) {
        return numbers.containsKey(ref);
    }

    @Override
    public RuntimeEncoder encoder(EncoderRef ref) {
        RuntimeEncoder encoder = encoders.get(ref);
        if (encoder == null) {
            throw new IllegalStateException("No runtime encoder registered for " + ref.defaultName());
        }
        return encoder;
    }

    @Override
    public RuntimeMotor motor(MotorRef ref) {
        RuntimeMotor motor = motors.get(ref);
        if (motor == null) {
            throw new IllegalStateException("No runtime motor registered for " + ref.defaultName());
        }
        return motor;
    }

    @Override
    public RuntimeBoolean bool(BooleanRef ref) {
        RuntimeBoolean value = booleans.get(ref);
        if (value == null) {
            throw new IllegalStateException("No runtime boolean registered for " + ref.defaultName());
        }
        return value;
    }

    @Override
    public RuntimeNumber number(NumberRef ref) {
        RuntimeNumber value = numbers.get(ref);
        if (value == null) {
            throw new IllegalStateException("No runtime number registered for " + ref.defaultName());
        }
        return value;
    }
}
