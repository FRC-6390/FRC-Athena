package ca.frc6390.athena.arcp;

import java.util.List;

/**
 * Robot-side ARCP publisher. The runtime publishes telemetry from the robot thread; the replay encoder
 * publishes replay records/backfill attachment from its ordered worker. Implementations must support both.
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

    /**
     * Sends raw replay record envelopes (dictionary, frames, markers) to sessions that opted in to replay.
     * The bytes are complete records produced by the replay encoder; they are forwarded unchanged.
     *
     * @param records buffer holding complete record envelopes
     * @param length number of valid bytes in {@code records}
     */
    default void publishRecords(byte[] records, int length) {
    }

    /**
     * Installs the source for replay backfill requests. Called once when the replay stream is attached.
     *
     * @param source backfill source
     */
    default void backfill(ArcpBackfill source) {
    }
}
