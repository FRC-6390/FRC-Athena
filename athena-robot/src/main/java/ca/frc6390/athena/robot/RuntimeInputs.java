package ca.frc6390.athena.robot;

import java.util.Objects;
import java.util.Optional;

/**
 * Seam between recorded driver station input and controller reads.
 *
 * <p>Controllers normally read WPILib's driver station directly. While a replay runs, {@link #inject} makes the
 * recorded joystick and driver station state of the current cycle visible to them instead, so recorded bindings
 * fire exactly as they did on the robot. The injected state is process-wide and must be released with
 * {@link #release()} when the replay ends.</p>
 */
public final class RuntimeInputs {
    private static volatile RuntimeInputSnapshot injected;

    private RuntimeInputs() {
    }

    /**
     * Makes {@code snapshot} the input that controllers read until {@link #release()} is called.
     *
     * @param snapshot recorded input for the cycle about to run
     */
    public static void inject(RuntimeInputSnapshot snapshot) {
        injected = Objects.requireNonNull(snapshot, "snapshot");
    }

    /** Stops injecting; controllers read the live driver station again. */
    public static void release() {
        injected = null;
    }

    /** Returns the injected snapshot, or empty when controllers read the live driver station. */
    public static Optional<RuntimeInputSnapshot> injected() {
        return Optional.ofNullable(injected);
    }
}
