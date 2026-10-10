package ca.frc6390.athena.arcp.v2;

/**
 * Robot-side decision hook. The server never pre-validates writes or actions (E5): every request
 * is forwarded here and the returned {@link WriteResult} is sent back to the client.
 *
 * <p>Called on control connection threads. Implementations should hand the request to the robot
 * loop and wait for the decision, or decide immediately if the check is cheap.</p>
 */
public interface WriteHandler {

    /** Handler that accepts everything. Only for tests and simulation. */
    WriteHandler ACCEPT_ALL = new WriteHandler() {
        @Override
        public WriteResult write(long signalId, Object value) {
            return WriteResult.accept(value);
        }

        @Override
        public WriteResult action(long signalId, boolean run) {
            return WriteResult.accept(Boolean.valueOf(run));
        }
    };

    /**
     * @param signalId target signal from the dictionary
     * @param value typed value (Boolean, Long, Double, String, arrays of those, or {@link ArcpStruct})
     * @return accept or reject with a reason
     */
    WriteResult write(long signalId, Object value);

    /**
     * @param signalId Action signal from the dictionary
     * @param run true to run the Action, false to cancel it
     * @return accept or reject with a reason
     */
    WriteResult action(long signalId, boolean run);
}
