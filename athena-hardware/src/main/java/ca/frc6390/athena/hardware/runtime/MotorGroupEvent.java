package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Objects;

/**
 * One change in a motor group's health.
 *
 * @param kind what happened
 * @param motor the motor that died or recovered
 * @param leader the motor leading the group after the change
 * @param aliveCount alive motors after the change
 * @param reason why the motor was judged dead, or empty
 */
public record MotorGroupEvent(Kind kind, MotorDevice motor, MotorDevice leader, int aliveCount, String reason) {
    /** Kinds of motor group change. */
    public enum Kind {
        /** The leader died and a surviving follower now leads the group. */
        LEADER_PROMOTED,
        /** A follower died; the group keeps running without it. */
        FOLLOWER_DROPPED,
        /** The group cannot keep running: the mechanism's failure policy now applies. */
        GROUP_FAILED,
        /** A dead motor is healthy again and rejoined the group as a follower. */
        MOTOR_RECOVERED
    }

    public MotorGroupEvent {
        Objects.requireNonNull(kind, "kind");
        Objects.requireNonNull(motor, "motor");
        Objects.requireNonNull(leader, "leader");
        reason = reason == null ? "" : reason;
    }

    /** Returns whether this event reports a failure, not a recovery. */
    public boolean failure() {
        return kind != Kind.MOTOR_RECOVERED;
    }
}
