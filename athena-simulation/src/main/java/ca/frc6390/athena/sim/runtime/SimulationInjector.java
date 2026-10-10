package ca.frc6390.athena.sim.runtime;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.sim.hardware.SimEncoderHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import java.util.Objects;

/**
 * Writes hardware inputs by device name into a simulation session, instead of letting simulated physics
 * produce them. Replay uses this to drive a robot from a recorded run. Names match
 * {@code defaultName()} of the declared device; the first device with a matching name is used.
 *
 * <p>Each call returns false when no declared device has that name.</p>
 */
public final class SimulationInjector {
    private final SimulationCoordinator coordinator;

    SimulationInjector(SimulationCoordinator coordinator) {
        this.coordinator = Objects.requireNonNull(coordinator, "coordinator");
    }

    /**
     * Injects motor feedback and electrical readings.
     *
     * @param name motor name
     * @param positionRotations integrated position
     * @param velocityRotationsPerSecond velocity
     * @param appliedVoltage applied voltage
     * @param supplyCurrentAmps supply current
     * @param statorCurrentAmps stator current
     * @return true when the motor exists
     */
    public boolean motor(
            String name,
            double positionRotations,
            double velocityRotationsPerSecond,
            double appliedVoltage,
            double supplyCurrentAmps,
            double statorCurrentAmps) {
        for (SimMotorHandle handle : coordinator.motorHandles().values()) {
            MotorDevice device = handle.device();
            if (device.defaultName().equals(name)) {
                handle.state(positionRotations, velocityRotationsPerSecond)
                        .injectElectrical(appliedVoltage, supplyCurrentAmps, statorCurrentAmps);
                return true;
            }
        }
        return false;
    }

    /**
     * Injects encoder readings.
     *
     * @param name encoder name
     * @param positionRotations position
     * @param absolutePositionRotations absolute position
     * @param velocityRotationsPerSecond velocity
     * @return true when the encoder exists
     */
    public boolean encoder(
            String name,
            double positionRotations,
            double absolutePositionRotations,
            double velocityRotationsPerSecond) {
        for (SimEncoderHandle handle : coordinator.encoderHandles().values()) {
            EncoderDevice device = handle.device();
            if (device.defaultName().equals(name)) {
                coordinator.encoderState(device, positionRotations, absolutePositionRotations, velocityRotationsPerSecond);
                return true;
            }
        }
        return false;
    }

    /**
     * Injects a digital input's active state.
     *
     * @param name digital input name
     * @param active active state, after inversion
     * @return true when the input exists
     */
    public boolean digitalInput(String name, boolean active) {
        for (DigitalInputDevice device : coordinator.digitalInputHandles().keySet()) {
            if (device.defaultName().equals(name)) {
                coordinator.digitalInput(device, active);
                return true;
            }
        }
        return false;
    }
}
