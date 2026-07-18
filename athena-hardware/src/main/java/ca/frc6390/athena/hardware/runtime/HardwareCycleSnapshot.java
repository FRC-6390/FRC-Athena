package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Map;
import java.util.Optional;

/** Immutable hardware inputs and health captured after one graph refresh. */
public record HardwareCycleSnapshot(
        long sequence,
        long timestampNanos,
        Map<HardwareIdentity, MotorInput> motors,
        Map<HardwareIdentity, EncoderInput> encoders,
        Map<HardwareIdentity, ImuInput> imus) {
    public HardwareCycleSnapshot {
        motors = motors == null ? Map.of() : Map.copyOf(motors);
        encoders = encoders == null ? Map.of() : Map.copyOf(encoders);
        imus = imus == null ? Map.of() : Map.copyOf(imus);
    }

    public Optional<MotorInput> motor(MotorDevice device) {
        return Optional.ofNullable(motors.get(HardwareIdentity.motor(device)));
    }

    public Optional<EncoderInput> encoder(EncoderDevice device) {
        return Optional.ofNullable(encoders.get(HardwareIdentity.encoder(device)));
    }

    public Optional<ImuInput> imu(ImuDevice device) {
        return Optional.ofNullable(imus.get(HardwareIdentity.imu(device)));
    }

    /** Returns true once this snapshot was produced by a graph refresh. */
    public boolean captured() {
        return sequence > 0;
    }

    public record MotorInput(
            String name,
            double positionRotations,
            double velocityRotationsPerSecond,
            double appliedVoltage,
            double supplyCurrentAmps,
            double statorCurrentAmps,
            boolean connected,
            String failure) {
        public MotorInput { failure = failure == null ? "" : failure; }
    }

    public record EncoderInput(
            String name,
            double positionRotations,
            double absolutePositionRotations,
            double velocityRotationsPerSecond,
            boolean connected,
            String failure) {
        public EncoderInput { failure = failure == null ? "" : failure; }
    }

    public record ImuInput(
            String name,
            double yawDegrees,
            double pitchDegrees,
            double rollDegrees,
            double angleDegrees,
            double yawRateDegreesPerSecond,
            double linearAccelerationXG,
            double linearAccelerationYG,
            double linearAccelerationZG,
            boolean connected,
            String failure) {
        public ImuInput { failure = failure == null ? "" : failure; }
    }

    public static HardwareCycleSnapshot empty() {
        return new HardwareCycleSnapshot(0, 0, Map.of(), Map.of(), Map.of());
    }
}
