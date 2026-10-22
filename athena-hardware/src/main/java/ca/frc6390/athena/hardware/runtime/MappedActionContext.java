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
    private long bindingRevision;

    @Override
    public long bindingRevision() {
        return bindingRevision;
    }

    /**
     * Registers an encoder runtime handle.
     *
     * @param ref encoder declaration
     * @param encoder runtime encoder
     * @return this context
     */
    public MappedActionContext encoder(EncoderDevice ref, EncoderHandle encoder) {
        Objects.requireNonNull(ref, "ref");
        Objects.requireNonNull(encoder, "encoder");
        if (encoders.put(ref, encoder) != encoder) bindingRevision++;
        return this;
    }

    public boolean hasEncoder(EncoderDevice ref) {
        return !encoders.isEmpty() && encoders.containsKey(ref);
    }

    /**
     * Registers a motor runtime handle.
     *
     * @param ref motor declaration
     * @param motor runtime motor
     * @return this context
     */
    public MappedActionContext motor(MotorDevice ref, MotorHandle motor) {
        Objects.requireNonNull(ref, "ref");
        Objects.requireNonNull(motor, "motor");
        if (motors.put(ref, motor) != motor) bindingRevision++;
        return this;
    }

    // Overlay contexts commonly leave these maps empty and delegate to the hardware graph.
    // Avoid hashing an entire follower declaration merely to test an empty registry.
    public boolean hasMotor(MotorDevice ref) {
        return !motors.isEmpty() && motors.containsKey(ref);
    }

    public MappedActionContext imu(ImuDevice ref, ImuHandle imu) {
        Objects.requireNonNull(ref, "ref");
        Objects.requireNonNull(imu, "imu");
        if (imus.put(ref, imu) != imu) bindingRevision++;
        return this;
    }

    public boolean hasImu(ImuDevice ref) {
        return !imus.isEmpty() && imus.containsKey(ref);
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
