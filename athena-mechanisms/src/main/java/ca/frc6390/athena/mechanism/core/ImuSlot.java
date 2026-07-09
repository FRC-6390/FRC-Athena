package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.ImuDevice;

/**
 * Fillable IMU slot.
 *
 * @param <O> owner type
 */
public final class ImuSlot<O> extends Slot<O, ImuDevice> {
    ImuSlot(O owner, String name, Runnable onFill) {
        super(owner, name, onFill);
    }
}
