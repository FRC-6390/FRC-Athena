package ca.frc6390.athena.mechanism.persist;

/**
 * Built-in tuning lock groups.
 *
 * <p>A lock group is any class. A writable telemetry value declares its group with
 * {@code @Telemetry(lock = ...)} or {@code TelemetryValue.lock(...)}; the robot defines the group's
 * condition once with {@code AthenaRobot.tuningLock(group, condition)}. A group without a defined
 * condition behaves as {@link InMatch}.</p>
 */
public final class Locks {
    private Locks() {
    }

    /**
     * Locked while the driver station reports an FMS connection. This is the default group and the
     * group that undefined groups fall back to.
     */
    public static final class InMatch {
        private InMatch() {
        }
    }

    /** Never locked. Writes are always accepted. */
    public static final class Never {
        private Never() {
        }
    }

    /** Always locked. Writes are always rejected. */
    public static final class Always {
        private Always() {
        }
    }
}
