package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.EncoderDevice;

/**
 * Fillable encoder slot.
 *
 * @param <O> owner type
 */
public final class EncoderSlot<O> extends Slot<O, EncoderDevice> {
    EncoderSlot(O owner, String name, Runnable onFill) {
        super(owner, name, onFill);
    }
}
