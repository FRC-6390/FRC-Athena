package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.List;

/**
 * Runtime access used by ref actions.
 */
public interface ActionContext {
    /**
     * Resolves an encoder declaration to its runtime handle.
     *
     * @param ref encoder declaration
     * @return runtime encoder
     */
    default EncoderHandle encoder(EncoderDevice ref) {
        throw new UnsupportedOperationException("Runtime encoder access is not available.");
    }

    /**
     * Resolves a motor declaration to its runtime handle.
     *
     * @param ref motor declaration
     * @return runtime motor
     */
    default MotorHandle motor(MotorDevice ref) {
        throw new UnsupportedOperationException("Runtime motor access is not available.");
    }

    /** Returns followers whose commands must be mirrored by Athena instead of vendor hardware. */
    default List<SoftwareMotorFollower> softwareFollowers(MotorDevice leader) {
        return List.of();
    }

    /**
     * All follower constraints in the leader's command frame, including native followers and descendants.
     * Native entries are read-only: only entries whose {@link MotorFollower#softwareCommand()} is true
     * receive mirrored commands. Graph contexts cache this view until the topology changes.
     */
    default List<? extends MotorFollower> followerConstraints(MotorDevice leader) {
        List<SoftwareMotorFollower> direct = softwareFollowers(leader);
        if (direct.isEmpty()) return direct;
        java.util.ArrayList<MotorFollower> followers = new java.util.ArrayList<>();
        java.util.Set<MotorDevice> visited = new java.util.HashSet<>();
        visited.add(leader);
        collectSoftwareConstraints(this, leader, null, followers, visited);
        return List.copyOf(followers);
    }

    private static void collectSoftwareConstraints(ActionContext context, MotorDevice leader, MotorHandle parentFrame,
            java.util.List<MotorFollower> result, java.util.Set<MotorDevice> visited) {
        for (SoftwareMotorFollower local : context.softwareFollowers(leader)) {
            if (!visited.add(local.device())) continue;
            SoftwareMotorFollower follower = parentFrame == null ? local : new SoftwareMotorFollower(local.device(),
                    new ChainedFollowerMotorHandle(local.handle(), parentFrame));
            result.add(follower);
            collectSoftwareConstraints(context, follower.device(), follower.handle(), result, visited);
        }
    }

    interface MotorFollower {
        MotorDevice device();
        MotorHandle handle();
        default boolean softwareCommand() { return false; }
    }

    /** One runtime follower target driven from its leader's resolved command. */
    record SoftwareMotorFollower(MotorDevice device, MotorHandle handle) implements MotorFollower {
        public SoftwareMotorFollower {
            if (device == null || handle == null) {
                throw new NullPointerException("Software follower device and handle are required.");
            }
        }
        @Override public boolean softwareCommand() { return true; }
    }

    default ImuHandle imu(ImuDevice ref) {
        throw new UnsupportedOperationException("Runtime IMU access is not available.");
    }

    /** Records a recoverable runtime hardware operation failure. */
    default void hardwareFailure(Object declaration, RuntimeException exception) {
        // Contexts without an owning hardware graph have nowhere to publish failures.
    }

    /**
     * Empty context for tests that only execute pure runnable actions.
     *
     * @return empty action context
     */
    static ActionContext empty() {
        return new ActionContext() {
        };
    }
}
