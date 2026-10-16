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
        return (T) DeviceActionFactories.get().then(this, next);
    }

    /**
     * Completes this action after the supplied duration.
     *
     * @param seconds maximum action duration in seconds
     * @param <T> resulting Athena action type
     * @return timeout-wrapped action
     */
    @SuppressWarnings("unchecked")
    default <T extends DeviceAction> T timeout(Number seconds) {
        return (T) DeviceActionFactories.get().timeout(this, seconds.doubleValue());
    }

    /**
     * Runs this action for the supplied duration, then completes it.
     *
     * @param seconds action duration in seconds
     * @param <T> resulting Athena action type
     * @return duration-wrapped action
     */
    @SuppressWarnings("unchecked")
    default <T extends DeviceAction> T forTime(Number seconds) {
        return (T) timeout(seconds);
    }
}
