package ca.frc6390.athena.sim.runtime;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.sim.hardware.SimEncoderHandle;
import ca.frc6390.athena.sim.hardware.SimImuHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
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
     * @param position integrated position
     * @param velocity velocity
     * @param appliedVoltage applied voltage
     * @param supplyCurrent supply current
     * @param statorCurrent stator current
     * @return true when the motor exists
     */
    public boolean motor(
            String name,
            Angle position,
            AngularVelocity velocity,
            Voltage appliedVoltage,
            Current supplyCurrent,
            Current statorCurrent) {
        for (SimMotorHandle handle : coordinator.motorHandles().values()) {
            MotorDevice device = handle.device();
            if (device.defaultName().equals(name)) {
                handle.state(position.in(Units.Rotations), velocity.in(Units.RotationsPerSecond))
                        .injectElectrical(appliedVoltage.in(Units.Volts), supplyCurrent.in(Units.Amps),
                                statorCurrent.in(Units.Amps));
                return true;
            }
        }
        return false;
    }

    /**
     * Injects encoder readings.
     *
     * @param name encoder name
     * @param position raw sensor position
     * @param absolutePosition raw absolute sensor position
     * @param velocity raw sensor velocity
     * @return true when the encoder exists
     */
    public boolean encoder(
            String name,
            Angle position,
            Angle absolutePosition,
            AngularVelocity velocity) {
        for (SimEncoderHandle handle : coordinator.encoderHandles().values()) {
            EncoderDevice device = handle.device();
            if (device.defaultName().equals(name)) {
                coordinator.encoderState(device, position, absolutePosition, velocity);
                return true;
            }
        }
        return false;
    }

    /**
     * Injects IMU readings.
     *
     * @param name IMU name
     * @param yaw yaw
     * @param pitch pitch
     * @param roll roll
     * @param yawRate yaw rate
     * @param pitchRate pitch rate
     * @param rollRate roll rate
     * @param accelerationX x acceleration
     * @param accelerationY y acceleration
     * @param accelerationZ z acceleration
     * @return true when the IMU exists
     */
    public boolean imu(
            String name,
            Angle yaw,
            Angle pitch,
            Angle roll,
            AngularVelocity yawRate,
            AngularVelocity pitchRate,
            AngularVelocity rollRate,
            LinearAcceleration accelerationX,
            LinearAcceleration accelerationY,
            LinearAcceleration accelerationZ) {
        for (SimImuHandle handle : coordinator.imuHandles().values()) {
            ImuDevice device = handle.device();
            if (device.defaultName().equals(name)) {
                handle.yawDegrees(yaw.in(Units.Degrees))
                        .pitchDegrees(pitch.in(Units.Degrees))
                        .rollDegrees(roll.in(Units.Degrees))
                        .angularRateDegreesPerSecond(rollRate.in(Units.DegreesPerSecond),
                                pitchRate.in(Units.DegreesPerSecond), yawRate.in(Units.DegreesPerSecond))
                        .linearAccelerationG(accelerationX.in(Units.Gs), accelerationY.in(Units.Gs),
                                accelerationZ.in(Units.Gs));
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
