package ca.frc6390.athena.mechanism.sysid;

/** One synchronized characterization sample in rotations or meters. */
public record SysIdSample(
        SysIdState state,
        boolean angular,
        double appliedVoltage,
        double position,
        double velocity,
        double currentAmps) {
}
