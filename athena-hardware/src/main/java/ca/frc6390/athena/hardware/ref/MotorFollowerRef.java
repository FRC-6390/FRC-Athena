package ca.frc6390.athena.hardware.ref;

import java.util.Objects;

/**
 * Declares that one motor should copy another motor's commanded input.
 */
public record MotorFollowerRef(MotorRef leader) {
    public MotorFollowerRef {
        Objects.requireNonNull(leader, "leader");
    }
}
