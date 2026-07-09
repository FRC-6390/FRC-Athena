package ca.frc6390.athena.hardware.device;

import java.util.Objects;

/**
 * Declares that one motor should copy another motor's commanded input.
 */
public record MotorFollowerBinding(MotorDevice leader) {
    public MotorFollowerBinding {
        Objects.requireNonNull(leader, "leader");
    }
}
