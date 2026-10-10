package ca.frc6390.athena.arcp;

/** Robot-side Action decision, run on the robot thread like {@link WriteHandler}. */
@FunctionalInterface
public interface ActionHandler {

    /** Handler that accepts everything. Only for tests and simulation. */
    ActionHandler ACCEPT_ALL = (signalId, run) -> WriteResult.accept(Boolean.valueOf(run));

    /**
     * @param signalId Action signal from the dictionary
     * @param run true to run the Action, false to cancel it
     * @return accept, or reject with a reason
     */
    WriteResult action(long signalId, boolean run);
}
