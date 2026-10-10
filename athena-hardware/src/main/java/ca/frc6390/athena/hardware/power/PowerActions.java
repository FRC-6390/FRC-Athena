package ca.frc6390.athena.hardware.power;

import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import java.lang.reflect.Method;

/**
 * Creates one-shot Actions without a compile-time dependency on the mechanism module, the same way
 * {@link ca.frc6390.athena.hardware.device.MotorDevice} creates its action factories.
 */
final class PowerActions {
    private PowerActions() {
    }

    @SuppressWarnings("unchecked")
    static <T extends DeviceAction> T doOnce(ActionBinding binding) {
        try {
            Class<?> actions = Class.forName("ca.frc6390.athena.mechanism.core.Actions");
            Method factory = actions.getMethod("doOnce", ActionBinding.class);
            return (T) factory.invoke(null, binding);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException("Power profile actions require athena-mechanisms on the classpath.", exception);
        }
    }
}
