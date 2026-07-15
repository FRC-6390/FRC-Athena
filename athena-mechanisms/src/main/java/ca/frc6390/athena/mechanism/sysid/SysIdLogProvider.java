package ca.frc6390.athena.mechanism.sysid;

/** Service-provider interface used to connect Athena characterization to a logger. */
public interface SysIdLogProvider {
    /** Opens one complete four-direction characterization log. */
    SysIdLog open(String routineName, String motorName);
}
