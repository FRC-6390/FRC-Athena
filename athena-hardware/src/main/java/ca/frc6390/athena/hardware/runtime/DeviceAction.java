package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.signal.MotorStallSignal;

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
     * Runs this action until the stall detector reports a commanded motor that is not moving,
     * then completes. Debounce timing restarts every time the composed action starts.
     *
     * @param signal stall detector, usually {@code motor.stall()} with tuned thresholds
     * @param <T> resulting Athena action type
     * @return stall-completing action
     */
    @SuppressWarnings("unchecked")
    default <T extends DeviceAction> T untilStalled(MotorStallSignal signal) {
        return (T) DeviceActionFactories.get().untilStalled(this, signal);
    }
}
