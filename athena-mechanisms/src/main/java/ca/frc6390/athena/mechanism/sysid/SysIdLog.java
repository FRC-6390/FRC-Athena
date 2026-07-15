package ca.frc6390.athena.mechanism.sysid;

/** Runtime destination for SysId-compatible characterization samples. */
public interface SysIdLog {
    /** Records one active-test sample. */
    void record(SysIdSample sample);

    /** Records the end of the current test. */
    void end();
}
