package ca.frc6390.athena.mechanism.sysid;

/** Test phases understood by WPILib's SysId analyzer. */
public enum SysIdState {
    QUASISTATIC_FORWARD,
    QUASISTATIC_REVERSE,
    DYNAMIC_FORWARD,
    DYNAMIC_REVERSE,
    NONE
}
