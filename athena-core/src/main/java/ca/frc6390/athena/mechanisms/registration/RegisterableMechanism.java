package ca.frc6390.athena.mechanisms;

import java.util.List;

/**
 * Common contract for anything that can be registered with RobotCore.
 */
public interface RegisterableMechanism {
    /**
     * Mechanisms to register (flattened). For a plain mechanism this is just itself;
     * for a superstructure this includes all child mechanisms.
     */
    List<Mechanism> flattenForRegistration();
}
