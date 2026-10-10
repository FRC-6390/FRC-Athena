package ca.frc6390.athena.wpilib.arcp;

import ca.frc6390.athena.arcp.ArcpBackfill;
import ca.frc6390.athena.arcp.ArcpSink;
import ca.frc6390.athena.robot.replay.ReplayStreamSink;
import java.util.List;
import java.util.Objects;

/**
 * Streams replay records to ARCP clients. The records are the replay encoder's bytes (dictionary, frame and
 * marker records in the shared ARCP v2 record format), forwarded unchanged to sessions that opted in with
 * {@code ReplaySubscribe}. Backfill for sequence gaps is answered from the replay recorder's buffer.
 *
 * <p>Runs on the robot thread and does not block.</p>
 */
final class ArcpReplaySink implements ReplayStreamSink {
    private final ArcpSink sink;

    ArcpReplaySink(ArcpSink sink) {
        this.sink = Objects.requireNonNull(sink, "sink");
    }

    @Override
    public void publishDictionary(byte[] records) {
        sink.publishRecords(records, records.length);
    }

    @Override
    public void publishFrame(long sequence, byte[] records) {
        sink.publishRecords(records, records.length);
    }

    @Override
    public void publishMarker(long sequence, byte[] record) {
        sink.publishRecords(record, record.length);
    }

    @Override
    public void attach(ReplayBackfill backfill) {
        Objects.requireNonNull(backfill, "backfill");
        sink.backfill(new ArcpBackfill() {
            @Override
            public List<byte[]> records(long fromSequence, long toSequence) {
                return backfill.records(fromSequence, toSequence);
            }
        });
    }
}
