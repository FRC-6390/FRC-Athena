package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Map-backed action context for runtimes that resolve devices from registered handles.
 */
public final class MappedActionContext implements ActionContext {
    private final Map<EncoderDevice, EncoderHandle> encoders = new LinkedHashMap<>();
    private final Map<MotorDevice, MotorHandle> motors = new LinkedHashMap<>();
    private final Map<ImuDevice, ImuHandle> imus = new LinkedHashMap<>();

    /**
     * Registers an encoder runtime handle.
     *
     * @param ref encoder declaration
     * @param encoder runtime encoder
     * @return this context
     */
    public MappedActionContext encoder(EncoderDevice ref, EncoderHandle encoder) {
        encoders.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(encoder, "encoder"));
        return this;
    }

    public boolean hasEncoder(EncoderDevice ref) {
        return encoders.containsKey(ref);
    }

    /**
     * Registers a motor runtime handle.
     *
     * @param ref motor declaration
     * @param motor runtime motor
     * @return this context
     */
    public MappedActionContext motor(MotorDevice ref, MotorHandle motor) {
        motors.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(motor, "motor"));
        return this;
    }

    public boolean hasMotor(MotorDevice ref) {
        return motors.containsKey(ref);
    }

    public MappedActionContext imu(ImuDevice ref, ImuHandle imu) {
        imus.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(imu, "imu"));
        return this;
    }

    public boolean hasImu(ImuDevice ref) {
        return imus.containsKey(ref);
    }

    @Override
    public EncoderHandle encoder(EncoderDevice ref) {
        EncoderHandle encoder = encoders.get(ref);
        if (encoder == null) {
            throw new IllegalStateException("No runtime encoder registered for " + ref.defaultName());
        }
        return encoder;
    }

    @Override
    public MotorHandle motor(MotorDevice ref) {
        MotorHandle motor = motors.get(ref);
        if (motor == null) {
            throw new IllegalStateException("No runtime motor registered for " + ref.defaultName());
        }
        return motor;
    }

    @Override
    public ImuHandle imu(ImuDevice ref) {
        ImuHandle imu = imus.get(ref);
        if (imu == null) {
            throw new IllegalStateException("No runtime IMU registered for " + ref.defaultName());
        }
        return imu;
    }
}
