package ca.frc6390.athena.hardware.runtime;

/**
 * Marker for an Athena action created by a hardware declaration.
 */
public interface DeviceAction extends ActionBinding, Runnable {
    /**
     * Requests this action from the active robot runtime.
     */
    void request();

    /** Requests this action when used through Java's runnable APIs. */
    @Override
    default void run() {
        request();
    }

    /**
     * Runs another device action after this one completes.
     *
     * @param next next action
     * @param <T> resulting Athena action type
     * @return composed action
     */
    @SuppressWarnings("unchecked")
    default <T extends DeviceAction> T then(DeviceAction next) {
        try {
            Class<?> actions = Class.forName("ca.frc6390.athena.mechanism.core.Actions");
            var factory = actions.getDeclaredMethod("then", DeviceAction.class, DeviceAction.class);
            if (!factory.canAccess(null)) {
                factory.setAccessible(true);
            }
            return (T) factory.invoke(null, this, next);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException("Device action composition requires athena-mechanisms on the classpath.", exception);
        }
    }
}
