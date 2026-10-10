package ca.frc6390.athena.arcp;

import java.util.List;

/**
 * Robot-side ARCP publisher. The runtime calls it from the robot thread once per cycle.
 * {@link NativeArcpServer} implements it on top of the Rust server; {@link #NONE} does nothing
 * when ARCP is not enabled.
 */
public interface ArcpSink {

    /** Sink that drops everything. */
    ArcpSink NONE = new ArcpSink() {
        @Override
        public void publishDictionary(List<ArcpDictEntry> entries, List<ArcpStructSchema> schemas) {
        }

        @Override
        public void publishCycle(long cycleTimestampMicros, ArcpCycle cycle) {
        }

        @Override
        public void marker(long timestampMicros, int kind, String message, long signalId) {
        }

        @Override
        public void serviceRequests() {
        }

        @Override
        public ArcpStatus status() {
            return ArcpStatus.STOPPED;
        }
    };

    /**
     * Replaces the dictionary and schemas and pushes them to connected clients.
     *
     * @param entries signal announcements
     * @param schemas struct schemas referenced by the entries
     */
    void publishDictionary(List<ArcpDictEntry> entries, List<ArcpStructSchema> schemas);

    /**
     * Publishes one robot cycle. The native server sends only values that changed.
     *
     * @param cycleTimestampMicros FPGA cycle timestamp in microseconds
     * @param cycle current values for this cycle
     */
    void publishCycle(long cycleTimestampMicros, ArcpCycle cycle);

    /**
     * Sends a marker (boot, match start, fault, ...) to every subscriber.
     *
     * @param timestampMicros marker time
     * @param kind marker kind, see {@link ArcpMarker}
     * @param message human-readable text
     * @param signalId related signal, or 0
     */
    void marker(long timestampMicros, int kind, String message, long signalId);

    /**
     * Answers client writes and actions queued since the last call. Call once per robot cycle from
     * the robot thread. Requests not answered within the server's reply timeout are rejected.
     */
    void serviceRequests();

    /** Current server state for dashboards and logs. */
    ArcpStatus status();
}
