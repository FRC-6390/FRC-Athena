package ca.frc6390.athena.arcp.v2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.arcp.v2.ArcpMessages.ClientRequest;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ReplyData;
import ca.frc6390.athena.arcp.v2.ArcpMessages.ServerMessage;
import ca.frc6390.athena.runtime.wire.Wire;
import ca.frc6390.athena.runtime.wire.WireReader;
import java.io.IOException;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.net.DatagramPacket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** End-to-end tests over real TCP and UDP sockets on loopback. */
final class ArcpServerTest {

    private static final long ANGLE = 1;
    private static final long POSE = 2;
    private static final long ENABLED = 3;
    private static final long OTHER = 4;

    @TempDir
    Path tempDir;

    private final List<String> writeLog = new CopyOnWriteArrayList<>();
    private ArcpServer server;
    private DatagramSocket udp;
    private int udpPort;

    private static List<ArcpDictEntry> dictionary() {
        return List.of(
                new ArcpDictEntry(ANGLE, "robot/turret/angle", 3, "deg", ArcpDictEntry.ROLE_STATE,
                        ArcpDictEntry.FLAG_WRITABLE, "InMatch", 0),
                new ArcpDictEntry(POSE, "robot/turret/pose", 9, "m", ArcpDictEntry.ROLE_VALUE, 0, "", 1),
                new ArcpDictEntry(ENABLED, "robot/turret/enabled", 1, "", ArcpDictEntry.ROLE_VALUE, 0, "", 0),
                new ArcpDictEntry(OTHER, "robot/intake/speed", 3, "", ArcpDictEntry.ROLE_VALUE, 0, "", 0));
    }

    private static List<ArcpStructSchema> schemas() {
        return List.of(new ArcpStructSchema(1, "Pose2d", "Translation2d translation;Rotation2d rotation"));
    }

    private void start(ArcpServer.Config config) throws IOException {
        server = new ArcpServer(config, new WriteHandler() {
            @Override
            public WriteResult write(long signalId, Object value) {
                writeLog.add("write:" + signalId);
                return signalId == ANGLE
                        ? WriteResult.reject("rejected: locked (InMatch)", 0.0)
                        : WriteResult.accept(value);
            }

            @Override
            public WriteResult action(long signalId, boolean run) {
                writeLog.add("action:" + signalId + ":" + run);
                return WriteResult.accept(Boolean.valueOf(run));
            }
        });
        server.publishDictionary(dictionary(), schemas());
        if (udp != null) {
            udp.close();
        }
        udp = new DatagramSocket(0, InetAddress.getLoopbackAddress());
        udp.setSoTimeout(1500);
        udpPort = udp.getLocalPort();
    }

    @BeforeEach
    void setUp() throws IOException {
        start(ArcpServer.Config.defaults());
    }

    @AfterEach
    void tearDown() throws IOException {
        if (server != null) {
            server.close();
        }
        if (udp != null) {
            udp.close();
        }
    }

    private ArcpTestClient connect() throws IOException {
        return new ArcpTestClient(server.controlPort());
    }

    private static ArcpCycle cycle(Object... pairs) {
        ArcpCycle c = new ArcpCycle();
        for (int i = 0; i < pairs.length; i += 2) {
            c.set((Long) pairs[i], pairs[i + 1]);
        }
        return c;
    }

    @Test
    void dictionaryIsPushedOnChangeAndSentOnSchemaRequest() throws IOException {
        try (ArcpTestClient client = connect()) {
            List<ArcpDictEntry> extended = new ArrayList<>(dictionary());
            extended.add(new ArcpDictEntry(50, "robot/new/signal", 2, "ticks", 2, 0, "", 0));
            server.publishDictionary(extended, schemas());
            ServerMessage pushed = client.readMessage();
            assertTrue(pushed instanceof ServerMessage.Schema s && s.dictionary().equals(extended));

            ServerMessage reply = client.request(ClientRequest::schemaRequest);
            assertTrue(reply instanceof ServerMessage.Schema s && s.dictionary().get(0).writable());
        }
    }

    @Test
    void subscribeSendsSnapshotThenOnlyMatchingChanges() throws IOException {
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            server.publishCycle(1_000_000, cycle(ANGLE, 10.0, OTHER, 5.0));
            assertTrue(ArcpTestClient.drainFrames(udp).isEmpty(), "nothing subscribed yet");

            client.subscribe("robot/turret/**", ArcpRate.ON_CHANGE);
            List<ArcpTestClient.Frame> snapshot = ArcpTestClient.drainFrames(udp);
            assertEquals(1, snapshot.size());
            assertEquals(1, snapshot.get(0).entries().size(), "only the matching signal is snapshotted");
            assertEquals(ANGLE, snapshot.get(0).entries().get(0).signalId());

            server.publishCycle(1_020_000, cycle(ANGLE, 10.0, OTHER, 6.0));
            assertTrue(ArcpTestClient.drainFrames(udp).isEmpty(), "unchanged matching value is not resent");

            server.publishCycle(1_040_000, cycle(ANGLE, 11.5, OTHER, 7.0));
            List<ArcpTestClient.Frame> batch = ArcpTestClient.drainFrames(udp);
            assertEquals(1, batch.size());
            assertEquals(1, batch.get(0).entries().size());
            assertEquals(11.5, (Double) batch.get(0).entries().get(0).value());
            assertEquals(1_040_000L, batch.get(0).timestamp());
            assertFalse(batch.get(0).keyframe());
        }
    }

    @Test
    void hzRatePacesUpdates() throws IOException {
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            client.subscribe("robot/turret/angle", ArcpRate.hz(10.0));
            ArcpTestClient.drainFrames(udp);
            for (long i = 0; i <= 500; i++) {
                server.publishCycle(i * 1000, cycle(ANGLE, (double) i));
            }
            int frames = ArcpTestClient.drainFrames(udp).size();
            assertTrue(frames >= 4 && frames <= 8, "expected about 5 frames at 10 Hz, got " + frames);
        }
    }

    @Test
    void oneCycleIsOneBatchedDatagram() throws IOException {
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            client.subscribe("robot/**", ArcpRate.ON_CHANGE);
            ArcpTestClient.drainFrames(udp);
            server.publishCycle(5_000_000, cycle(ANGLE, 1.0, ENABLED, true));
            int datagrams = 0;
            int entries = 0;
            byte[] buf = new byte[65536];
            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            while (true) {
                try {
                    udp.receive(packet);
                } catch (SocketTimeoutException timeout) {
                    break;
                }
                datagrams++;
                for (ArcpTestClient.Frame f : ArcpTestClient.parseDatagram(packet.getData(), packet.getLength())) {
                    entries += f.entries().size();
                }
            }
            assertEquals(1, datagrams, "both changed signals fit in one datagram");
            assertEquals(2, entries);
        }
    }

    @Test
    void largeCyclesAreFragmentedAndRecoverable() throws IOException {
        server.close();
        start(new ArcpServer.Config(0, null, 0L, 256, 1_000_000L));
        List<ArcpDictEntry> dict = new ArrayList<>(dictionary());
        for (int i = 0; i < 40; i++) {
            dict.add(new ArcpDictEntry(100 + i, "robot/bulk/" + i, 3, "", ArcpDictEntry.ROLE_VALUE, 0, "", 0));
        }
        server.publishDictionary(dict, schemas());
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            client.subscribe("robot/**", ArcpRate.ON_CHANGE);
            ArcpTestClient.drainFrames(udp);
            ArcpCycle c = new ArcpCycle();
            for (int i = 0; i < 40; i++) {
                c.set(100 + i, (double) i);
            }
            server.publishCycle(9_000_000, c);
            List<ArcpTestClient.Frame> frames = ArcpTestClient.drainFrames(udp);
            assertTrue(frames.size() > 1, "expected fragments");
            assertTrue(frames.stream().allMatch(f -> f.fragmented() && f.count() == frames.size()));
            List<Long> ids = frames.stream().flatMap(f -> f.entries().stream()).map(ArcpCycle.Entry::signalId)
                    .sorted(Comparator.naturalOrder()).toList();
            assertEquals(40, ids.size());
            assertEquals(100L, ids.get(0));
            assertEquals(139L, ids.get(39));
        }
    }

    @Test
    void markersReachSubscribersUnfiltered() throws IOException {
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            server.marker(42, ArcpCodec.MARKER_FAULT, "boom", 0);
            byte[] buf = new byte[1024];
            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            udp.receive(packet);
            WireReader r = new WireReader(Arrays.copyOf(packet.getData(), packet.getLength()));
            WireReader.Record record = r.record();
            assertEquals(Wire.RECORD_MARKER, record.type());
            assertEquals(42L, record.payload().zigzag());
        }
    }

    @Test
    void writesAreForwardedAndRobotReplyIsReturned() throws IOException {
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            ServerMessage.WriteReply rejected = client.write(ANGLE, 3.0);
            assertFalse(rejected.accepted());
            assertEquals("rejected: locked (InMatch)", rejected.reason());
            assertEquals(0.0, (Double) rejected.actualValue());

            ServerMessage.WriteReply accepted = client.write(OTHER, 3.0);
            assertTrue(accepted.accepted());
            assertEquals(3.0, (Double) accepted.actualValue());
            assertEquals(List.of("write:" + ANGLE, "write:" + OTHER), writeLog.subList(0, 2));

            assertEquals(ArcpMessages.STATUS_OK, client.reply(id -> ClientRequest.actionRun(id, 9)).status());
            assertEquals(ArcpMessages.STATUS_OK, client.reply(id -> ClientRequest.actionCancel(id, 9)).status());
            assertTrue(writeLog.contains("action:9:true") && writeLog.contains("action:9:false"));
        }
    }

    @Test
    void timeSyncWorksBeforeHelloAndOtherRequestsNeedHello() throws IOException {
        try (ArcpTestClient client = connect()) {
            ServerMessage.Reply refused = client.reply(ClientRequest::fileList);
            assertEquals(ArcpMessages.STATUS_ERROR, refused.status());
            assertEquals("hello required", refused.reason());
            ServerMessage pong = client.request(id -> ClientRequest.timePing(id, 77));
            assertTrue(pong instanceof ServerMessage.TimePong p && p.clientMicros() == 77 && p.serverMicros() > 0);
        }
    }

    @Test
    void hellosWithWrongVersionAndBadSubscriptionsAreRejected() throws IOException {
        try (ArcpTestClient client = connect()) {
            ServerMessage.Reply r = client.reply(id -> ClientRequest.hello(id, "old", 1, 1));
            assertEquals("unsupported protocol version", r.reason());
        }
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            ServerMessage.Reply bad = client.reply(id -> ClientRequest.subscribe(id, "robot//x", ArcpRate.ON_CHANGE));
            assertEquals(ArcpMessages.STATUS_ERROR, bad.status());
            ServerMessage.Reply badRate = client.reply(id -> ClientRequest.subscribe(id, "robot/**", ArcpRate.hz(-1)));
            assertEquals(ArcpMessages.STATUS_ERROR, badRate.status());
        }
    }

    @Test
    void keyframeContainsEverySubscribedSignal() throws IOException {
        server.close();
        start(new ArcpServer.Config(0, null, 0L, 1200, 100_000L));
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            client.subscribe("robot/turret/**", ArcpRate.ON_CHANGE);
            ArcpTestClient.drainFrames(udp);
            server.publishCycle(0, cycle(ANGLE, 1.0, ENABLED, true));
            ArcpTestClient.drainFrames(udp);
            server.publishCycle(200_000, cycle(ANGLE, 1.0, ENABLED, true));
            List<ArcpTestClient.Frame> frames = ArcpTestClient.drainFrames(udp);
            assertTrue(frames.stream().anyMatch(f -> f.keyframe() && f.entries().size() == 2));
        }
    }

    @Test
    void fileServiceListsDownloadsAndDeletesWithinRoot() throws IOException {
        server.close();
        Files.writeString(tempDir.resolve("match-1.athrp"), "0123456789");
        Files.writeString(tempDir.resolve("tuning.json"), "{}");
        start(new ArcpServer.Config(0, tempDir, 0L, 1200, 1_000_000L));
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            ServerMessage.Reply list = client.reply(ClientRequest::fileList);
            List<ArcpMessages.FileInfo> files = ((ReplyData.Files) list.data()).files();
            assertEquals(List.of("match-1.athrp", "tuning.json"), files.stream().map(ArcpMessages.FileInfo::name).toList());
            assertEquals(10, files.get(0).size());

            ReplyData.Chunk middle = (ReplyData.Chunk) client.reply(id -> ClientRequest.fileDownload(id, "match-1.athrp", 4, 3)).data();
            assertFalse(middle.eof());
            assertEquals("456", new String(middle.data(), StandardCharsets.US_ASCII));
            ReplyData.Chunk tail = (ReplyData.Chunk) client.reply(id -> ClientRequest.fileDownload(id, "match-1.athrp", 8, 100)).data();
            assertTrue(tail.eof());

            for (String bad : List.of("../secret", "a/b", "..", "", "C:evil")) {
                assertEquals(ArcpMessages.STATUS_ERROR,
                        client.reply(id -> ClientRequest.fileDownload(id, bad, 0, 10)).status(), bad);
                assertEquals(ArcpMessages.STATUS_ERROR,
                        client.reply(id -> ClientRequest.fileDelete(id, bad)).status(), bad);
            }
            assertEquals(ArcpMessages.STATUS_OK,
                    client.reply(id -> ClientRequest.fileDelete(id, "tuning.json")).status());
            assertFalse(Files.exists(tempDir.resolve("tuning.json")));
        }
    }

    @Test
    void fileServiceIsOffWithoutRoot() throws IOException {
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            ServerMessage.Reply r = client.reply(ClientRequest::fileList);
            assertEquals("file service not configured", r.reason());
        }
    }

    @Test
    void bandwidthBudgetThrottlesNewestSubscriptionFirst() throws IOException {
        server.close();
        start(new ArcpServer.Config(0, null, 200L, 1200, 1_000_000L));
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            long first = client.subscribe("robot/turret/angle", ArcpRate.hz(500.0));
            long second = client.subscribe("robot/intake/**", ArcpRate.hz(500.0));
            ArcpTestClient.drainFrames(udp);
            for (long i = 0; i < 3000; i++) {
                server.publishCycle(i * 2000, cycle(ANGLE, (double) i, OTHER, (double) i));
            }
            ArcpServer.Status status = server.status();
            int firstDivisor = divisor(status, first);
            int secondDivisor = divisor(status, second);
            assertTrue(secondDivisor > 1, "the newest subscription must be throttled");
            assertTrue(secondDivisor >= firstDivisor, "lowest priority is throttled first");
            assertEquals(2, status.subscriptions());
        }
    }

    private static int divisor(ArcpServer.Status status, long subscriptionId) {
        return status.throttled().stream()
                .filter(t -> t.subscriptionId() == subscriptionId)
                .mapToInt(ArcpServer.Throttled::divisor)
                .findFirst()
                .orElse(1);
    }

    @Test
    void writeHandlerReceivesTypedValues() throws IOException {
        // Round trip a struct write through the handler.
        List<Object> seen = new ArrayList<>();
        server.close();
        server = new ArcpServer(ArcpServer.Config.defaults(), new WriteHandler() {
            @Override
            public WriteResult write(long signalId, Object value) {
                seen.add(value);
                return WriteResult.accept(value);
            }

            @Override
            public WriteResult action(long signalId, boolean run) {
                return WriteResult.accept(Boolean.valueOf(run));
            }
        });
        try (ArcpTestClient client = connect()) {
            client.hello(udpPort);
            assertTrue(client.write(POSE, new ArcpStruct(1, new byte[24])).accepted());
            assertEquals(1, seen.size());
            assertTrue(seen.get(0) instanceof ArcpStruct s && s.bytes().length == 24);
        }
    }

    @Test
    void closeStopsAcceptingConnections() throws IOException {
        int port = server.controlPort();
        server.close();
        server = null;
        assertThrows(IOException.class, () -> new Socket(InetAddress.getLoopbackAddress(), port).close());
    }
}
