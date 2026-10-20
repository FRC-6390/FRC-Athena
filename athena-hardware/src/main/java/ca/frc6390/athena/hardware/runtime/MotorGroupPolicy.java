package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.List;
import java.util.Objects;

/**
 * How one leader-and-followers motor group responds when a member dies. The group is the leader and every motor
 * declared with {@code .follow(leader)}. Robot code declares this through the mechanism-level
 * {@code MotorFailover}; backends and runtimes read it here.
 *
 * @param leader declared group leader
 * @param promotionOrder followers to promote first, in order; others follow by ascending CAN id
 * @param promoteLeader true to promote a surviving follower when the leader dies
 * @param dropFollowers true to keep running without a dead follower
 * @param minimumAlive fewest alive motors the group runs with (at least 1)
 * @param outputScale percent and voltage scale while any member is dead, in (0, 1]
 * @param currentLimitScale current-limit scale for survivors while any member is dead, in (0, 1]
 * @param rejoin true to let a recovered motor rejoin the group as a follower
 * @param recoverySamples consecutive healthy refresh cycles before a dead motor rejoins (at least 1)
 */
public record MotorGroupPolicy(
        MotorDevice leader,
        List<MotorDevice> promotionOrder,
        boolean promoteLeader,
        boolean dropFollowers,
        int minimumAlive,
        double outputScale,
        double currentLimitScale,
        boolean rejoin,
        int recoverySamples) {
    public MotorGroupPolicy {
        Objects.requireNonNull(leader, "leader");
        promotionOrder = List.copyOf(promotionOrder == null ? List.of() : promotionOrder);
        if (minimumAlive < 1) throw new IllegalArgumentException("A motor group needs at least one alive motor.");
        if (!(outputScale > 0.0 && outputScale <= 1.0)) {
            throw new IllegalArgumentException("Output derate must be in (0, 1].");
        }
        if (!(currentLimitScale > 0.0 && currentLimitScale <= 1.0)) {
            throw new IllegalArgumentException("Current-limit derate must be in (0, 1].");
        }
        if (recoverySamples < 1) throw new IllegalArgumentException("Recovery needs at least one healthy sample.");
    }

    /** Returns the strict policy: any member death fails the group, as Athena did before failover existed. */
    public static MotorGroupPolicy strict(MotorDevice leader) {
        return new MotorGroupPolicy(leader, List.of(), false, false, 1, 1.0, 1.0, true, 5);
    }
}
