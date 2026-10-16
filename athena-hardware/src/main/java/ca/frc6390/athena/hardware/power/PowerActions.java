package ca.frc6390.athena.hardware.power;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.runtime.DeviceActionFactories;

/** Creates one-shot power actions through the device action factory. */
final class PowerActions {
    private PowerActions() {
    }

    @SuppressWarnings("unchecked")
    static <T extends DeviceAction> T doOnce(ActionBinding binding) {
        return (T) DeviceActionFactories.get().doOnce(binding);
    }
}
