package ca.frc6390.athena.arcp;

/**
 * Robot-side write decision. The server never pre-validates writes (E5): every write request is
 * handed to the robot thread through {@link NativeArcpServer#serviceRequests()}, and the returned
 * {@link WriteResult} is sent back to the client.
 */
@FunctionalInterface
public interface WriteHandler {

    /** Handler that accepts everything. Only for tests and simulation. */
    WriteHandler ACCEPT_ALL = (signalId, value) -> WriteResult.accept(value);

    /**
     * @param signalId target signal from the dictionary
     * @param value typed value (Boolean, Long, Double, String, arrays of those, or {@link ArcpStruct})
     * @return accept, or reject with a reason and the value the robot still holds
     */
    WriteResult write(long signalId, Object value);
}
