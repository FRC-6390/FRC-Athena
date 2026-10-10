package ca.frc6390.athena.arcp.v2;

import java.util.List;

/**
 * Extension point the robot runtime uses to publish to ARCP. {@link ArcpServer} implements it;
 * {@link #NONE} discards everything when ARCP is not enabled.
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
    };

    /**
     * Replaces the dictionary and schemas and pushes them to connected clients.
     *
     * @param entries signal announcements
     * @param schemas struct schemas referenced by the entries
     */
    void publishDictionary(List<ArcpDictEntry> entries, List<ArcpStructSchema> schemas);

    /**
     * Publishes one robot cycle. Unchanged values are ignored by the server.
     *
     * @param cycleTimestampMicros FPGA cycle timestamp in microseconds
     * @param cycle current values for this cycle
     */
    void publishCycle(long cycleTimestampMicros, ArcpCycle cycle);

    /**
     * Sends a marker (boot, match start, fault, ...) to every subscriber.
     *
     * @param timestampMicros marker time
     * @param kind marker kind, see {@link ArcpCodec#MARKER_FAULT} and friends
     * @param message human-readable text
     * @param signalId related signal, or 0
     */
    void marker(long timestampMicros, int kind, String message, long signalId);
}
