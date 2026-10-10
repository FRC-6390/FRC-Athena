package ca.frc6390.athena.arcp.v2;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.arcp.v2.ArcpMessages.ClientRequest;
import ca.frc6390.athena.arcp.v2.ArcpMessages.FileInfo;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ReplyData;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ServerMessage;
import ca.frc6390.athena.runtime.wire.WireReader;
import ca.frc6390.athena.runtime.wire.WireWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Cross-checks the Java ARCP codec against the reference codec and the golden vectors. */
final class ArcpCodecTest {

    private static final long CYCLE = 1_234_567_890L;

    private static String vector(String name) throws IOException {
        return Files.readString(GoldenVectorsTest.vectorDir().resolve(name), StandardCharsets.US_ASCII).trim();
    }

    private static List<ArcpDictEntry> dictionaryEntries() {
        return List.of(
                new ArcpDictEntry(1, "robot/turret/angle", 3, "deg", 1, 3, GoldenVectorsTest.LOCK_GROUP, 0),
                new ArcpDictEntry(2, "robot/turret/enabled", 1, "", 2, 1, "", 0),
                new ArcpDictEntry(3, "robot/drive/pose", 9, "m", 4, 0, "", 1),
                new ArcpDictEntry(4, "robot/mode", 4, "", 1, 0, "", 0),
                new ArcpDictEntry(5, "athena/locks/" + GoldenVectorsTest.LOCK_GROUP, 1, "", 7, 0, "", 0));
    }

    @Test
    void dictionaryRecordMatchesGoldenVector() throws IOException {
        assertEquals(vector("dictionary.hex"), GoldenVectorsTest.hex(ArcpCodec.dictionaryRecord(dictionaryEntries())));
    }

    @Test
    void structSchemaRecordMatchesGoldenVector() throws IOException {
        byte[] record = ArcpCodec.structSchemaRecord(
                new ArcpStructSchema(1, "Pose2d", "Translation2d translation;Rotation2d rotation"));
        assertEquals(vector("struct_schema.hex"), GoldenVectorsTest.hex(record));
    }

    @Test
    void markerRecordsMatchGoldenVector() throws IOException {
        byte[] markers = GoldenVectorsTest.concat(
                ArcpCodec.markerRecord(-5000L, ArcpCodec.MARKER_FAULT, "motor fault", 0),
                ArcpCodec.markerRecord(CYCLE + 60_000, ArcpCodec.MARKER_USER_BASE, "user", 111));
        assertEquals(vector("marker.hex"), GoldenVectorsTest.hex(markers));
    }

    @Test
    void keyframeWithEveryTypeTagMatchesGoldenVector() throws IOException {
        byte[] structBytes = new WireWriterHelper().f64(1.0).f64(2.0).f64(0.5).bytes();
        List<byte[]> entries = List.of(
                ArcpCodec.encodeEntry(37, true),
                ArcpCodec.encodeEntry(74, -1234567890123L),
                ArcpCodec.encodeEntry(111, 3.25),
                ArcpCodec.encodeEntry(148, "hello ✓"),
                ArcpCodec.encodeEntry(185, new boolean[] {true, false, true}),
                ArcpCodec.encodeEntry(222, new long[] {0L, -1L, Long.MAX_VALUE}),
                ArcpCodec.encodeEntry(259, new double[] {1.5, -2.25}),
                ArcpCodec.encodeEntry(296, new String[] {"a", "", "bc"}),
                ArcpCodec.encodeEntry(333, new ArcpStruct(1, structBytes)));
        byte[] frame = ArcpCodec.frameRecord(true, false, CYCLE, 42, 0, 1, entries);
        assertEquals(vector("frame_keyframe_all_types.hex"), GoldenVectorsTest.hex(frame));
    }

    @Test
    void deltaAndFragmentFramesMatchGoldenVectors() throws IOException {
        byte[] delta = ArcpCodec.frameRecord(false, false, CYCLE + 20_000, 43, 0, 1,
                List.of(ArcpCodec.encodeEntry(74, -1234567889999L), ArcpCodec.encodeEntry(37, false)));
        assertEquals(vector("frame_delta.hex"), GoldenVectorsTest.hex(delta));
        byte[] fragment = ArcpCodec.frameRecord(false, true, CYCLE + 40_000, 44, 1, 2,
                List.of(ArcpCodec.encodeEntry(148, "fragmented")));
        assertEquals(vector("frame_fragment.hex"), GoldenVectorsTest.hex(fragment));
    }

    @Test
    void valueRoundTripsThroughReferenceReader() {
        Object[] values = {
            Boolean.TRUE, 42L, 2.5, "x", new boolean[] {false, true}, new long[] {-7L},
            new double[] {0.25}, new String[] {"p", "q"}, new ArcpStruct(9, new byte[] {1, 2, 3})
        };
        for (Object value : values) {
            WireWriter w = new WireWriter();
            ArcpCodec.writeValue(w, value);
            Object back = ArcpCodec.readValue(new WireReader(w.toByteArray()));
            assertTrue(ArcpCodec.sameValue(value, back), String.valueOf(value));
        }
    }

    @Test
    void requestsRoundTrip() {
        List<ClientRequest> requests = List.of(
                ClientRequest.hello(1, "dash", 2, 5810),
                ClientRequest.subscribe(2, "robot/turret/**", ArcpRate.hz(50.0)),
                ClientRequest.subscribe(3, "robot/**", ArcpRate.ON_CHANGE),
                ClientRequest.unsubscribe(4, 7),
                ClientRequest.write(5, 300, 1.25),
                ClientRequest.actionRun(6, 9),
                ClientRequest.actionCancel(7, 9),
                ClientRequest.timePing(8, 123_456),
                ClientRequest.schemaRequest(9),
                ClientRequest.fileList(10),
                ClientRequest.fileDownload(11, "a.athrp", 10, 4096),
                ClientRequest.fileDelete(12, "b.json"));
        for (ClientRequest r : requests) {
            ClientRequest back = ArcpCodec.decodeRequest(ArcpCodec.encodeRequest(r));
            assertEquals(r.kind(), back.kind());
            assertEquals(r.requestId(), back.requestId());
            assertEquals(r.text(), back.text());
            assertEquals(r.id(), back.id());
            assertEquals(r.timestamp(), back.timestamp());
            assertEquals(r.offset(), back.offset());
            assertEquals(r.maxLength(), back.maxLength());
            assertEquals(r.rate(), back.rate());
            assertEquals(r.port(), back.port());
        }
    }

    @Test
    void serverMessagesRoundTrip() {
        List<ServerMessage> messages = List.of(
                new ServerMessage.Reply(3, ArcpMessages.STATUS_REJECTED, "rejected: locked (InMatch)", new ReplyData.None()),
                new ServerMessage.Reply(4, ArcpMessages.STATUS_OK, "", new ReplyData.Subscription(4)),
                new ServerMessage.Reply(5, ArcpMessages.STATUS_OK, "", new ReplyData.Session(2)),
                new ServerMessage.Reply(6, ArcpMessages.STATUS_OK, "",
                        new ReplyData.Files(List.of(new FileInfo("x.athrp", 99, 5)))),
                new ServerMessage.Reply(7, ArcpMessages.STATUS_OK, "", new ReplyData.Chunk(true, new byte[] {1, 2, 3})),
                new ServerMessage.Schema(0, dictionaryEntries(),
                        List.of(new ArcpStructSchema(1, "Pose2d", "Translation2d translation;Rotation2d rotation"))),
                new ServerMessage.TimePong(8, 1, 2),
                new ServerMessage.WriteReply(9, false, "rejected: locked (InMatch)", 1.5));
        for (ServerMessage m : messages) {
            ServerMessage back = ArcpCodec.decodeServer(ArcpCodec.encodeServer(m));
            if (m instanceof ServerMessage.Reply reply && reply.data() instanceof ReplyData.Chunk chunk) {
                ReplyData.Chunk other = (ReplyData.Chunk) ((ServerMessage.Reply) back).data();
                assertArrayEquals(chunk.data(), other.data());
            } else {
                assertEquals(m.toString(), back.toString());
            }
        }
    }

    @Test
    void framingWaitsForCompleteBodies() {
        byte[] body = ArcpCodec.encodeRequest(ClientRequest.fileList(1));
        byte[] framed = ArcpCodec.frame(body);
        assertEquals(-1, ArcpCodec.framedLength(framed, framed.length - 1));
        assertEquals(body.length, ArcpCodec.framedLength(framed, framed.length));
    }

    @Test
    void patternRules() {
        assertTrue(ArcpPattern.matches("robot/turret/**", "robot/turret"));
        assertTrue(ArcpPattern.matches("robot/turret/**", "robot/turret/pid/kp"));
        assertTrue(!ArcpPattern.matches("robot/turret/**", "robot/turretx/angle"));
        assertTrue(ArcpPattern.matches("robot/*/angle", "robot/turret/angle"));
        assertTrue(!ArcpPattern.matches("robot/*/angle", "robot/turret/pid/angle"));
        assertTrue(ArcpPattern.matches("robot/tur*/an?le", "robot/turret/angle"));
        assertEquals("pattern has an empty segment", ArcpPattern.validationError("robot//x"));
        assertEquals(null, ArcpPattern.validationError("robot/**"));
    }

    /** Small helper to build struct bytes with the reference writer. */
    private static final class WireWriterHelper {
        private final WireWriter w = new WireWriter();

        WireWriterHelper f64(double v) {
            w.f64(v);
            return this;
        }

        byte[] bytes() {
            return w.toByteArray();
        }
    }
}
