package ca.frc6390.athena.hardware.capability;

import java.util.EnumSet;
import java.util.Set;

/**
 * Immutable set of motor capabilities.
 *
 * @param motorCapabilities supported motor capabilities
 */
public record CapabilitySet(Set<MotorCapability> motorCapabilities) {
    /**
     * Creates a capability set.
     *
     * @param capabilities motor capabilities
     * @return capability set
     */
    public static CapabilitySet of(MotorCapability... capabilities) {
        EnumSet<MotorCapability> set = EnumSet.noneOf(MotorCapability.class);
        if (capabilities != null) {
            for (MotorCapability capability : capabilities) {
                if (capability != null) {
                    set.add(capability);
                }
            }
        }
        return new CapabilitySet(set);
    }

    public CapabilitySet {
        motorCapabilities = Set.copyOf(motorCapabilities);
    }

    /**
     * Returns whether the capability is present.
     *
     * @param capability capability to check
     * @return true if present
     */
    public boolean contains(MotorCapability capability) {
        return motorCapabilities.contains(capability);
    }
}
