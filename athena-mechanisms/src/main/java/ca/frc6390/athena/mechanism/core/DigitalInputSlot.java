package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;

/**
 * Fillable digital input slot.
 *
 * @param <O> owner type
 */
public final class DigitalInputSlot<O> extends Slot<O, DigitalInputDevice> {
    DigitalInputSlot(O owner, String name, Runnable onFill) {
        super(owner, name, onFill);
    }
}
