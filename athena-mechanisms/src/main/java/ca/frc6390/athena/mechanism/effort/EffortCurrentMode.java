package ca.frc6390.athena.mechanism.effort;

import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Hook for a vendor hardware torque or current mode (B6). It is an extension point only.
 *
 * <p>Athena enforces effort limits with a voltage clamp on the software output path, which works for any
 * controller. A motor handle that also exposes a hardware current or torque mode can implement this
 * interface later. The output path would then pass the per-motor current target to the controller instead of
 * a voltage. No built-in handle implements it yet, so the clamp is always used.</p>
 */
public interface EffortCurrentMode {
    /**
     * Returns whether the motor can be driven by a current target on its controller.
     *
     * @param motor declared motor
     * @return true when {@link #setCurrentAmps(MotorDevice, double)} is supported
     */
    boolean supportsCurrentMode(MotorDevice motor);

    /**
     * Commands a per-motor current on the controller.
     *
     * @param motor declared motor
     * @param amps current target in amps
     */
    void setCurrentAmps(MotorDevice motor, double amps);
}
