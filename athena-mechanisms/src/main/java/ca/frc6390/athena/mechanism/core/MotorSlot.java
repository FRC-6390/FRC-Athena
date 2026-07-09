package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Fillable motor slot.
 *
 * @param <O> owner type
 */
public final class MotorSlot<O> extends Slot<O, MotorDevice> {
    MotorSlot(O owner, String name, Runnable onFill) {
        super(owner, name, onFill);
    }
}
