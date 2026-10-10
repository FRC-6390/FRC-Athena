package ca.frc6390.athena.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * End-to-end test through JNI: the Rust server runs in this process, a test client talks TCP and
 * UDP to it, and the robot side is driven from the test thread the way the robot loop would.
 */
class NativeArcpServerTest {

    static final long ANGLE = 1;
    static final long RUNNING = 2;
    static final long RUN = 3;

    private NativeArcpServer server;
    private ExecutorService pool;

    @BeforeEach
    void start() {
        pool = Executors.newCachedThreadPool();
    }

    @AfterEach
    void stop() {
        if (server != null) {
            server.close();
        }
        pool.shutdownNow();
    }

    private static List<ArcpDictEntry> dictionary() {
        return List.of(
                new ArcpDictEntry(ANGLE, "robot/turret/angle", 3, "deg", ArcpDictEntry.ROLE_STATE,
                        ArcpDictEntry.FLAG_WRITABLE, "", 0),
                new ArcpDictEntry(RUNNING, "robot/intake/running", 1, "", ArcpDictEntry.ROLE_STATE, 0, "", 0),
                new ArcpDictEntry(RUN, "robot/intake/run", 1, "", ArcpDictEntry.ROLE_ACTION,
                        ArcpDictEntry.FLAG_WRITABLE, "", 0));
    }

    private static ArcpCycle cycle(double angle, boolean running) {
        return new ArcpCycle().set(ANGLE, angle).set(RUNNING, running);
    }

    @Test
    void frameWriteRejectionAndActionReplyReachTheClient() throws Exception {
        WriteHandler writes = (signalId, value) -> WriteResult.reject("rejected: locked (InMatch)", 1.5);
        ActionHandler actions = (signalId, run) -> WriteResult.reject("not ready", Boolean.FALSE);
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), writes, actions);
        server.publishDictionary(dictionary(), List.of());

        try (ArcpWireClient client = new ArcpWireClient(server.controlPort());
                DatagramSocket udp = new DatagramSocket(0, InetAddress.getLoopbackAddress())) {
            ArcpWireClient.Reply hello = client.hello(udp.getLocalPort());
            assertEquals(ArcpWireClient.STATUS_OK, hello.status());

            ArcpWireClient.Reply sub = client.subscribe("robot/**");
            assertEquals(ArcpWireClient.STATUS_OK, sub.status());
            assertTrue(sub.subscriptionId() >= 1);

            server.publishCycle(1_000_000L, cycle(1.5, true));
            List<ArcpWireClient.Frame> frames = ArcpWireClient.receiveFrames(udp, 2_000);
            assertFalse(frames.isEmpty(), "expected a frame after publishCycle");
            ArcpWireClient.Frame frame = frames.get(0);
            assertEquals(1_000_000L, frame.timestampMicros());
            assertEquals(1.5, (Double) frame.values().get(ANGLE));
            assertEquals(Boolean.TRUE, frame.values().get(RUNNING));

            // Write 2.0; the robot rejects it and the client gets the value the robot holds.
            Future<ArcpWireClient.WriteAnswer> answer = pool.submit(
                    () -> client.awaitWriteReply(client.sendWrite(ANGLE, 2.0)));
            ArcpWireClient.WriteAnswer reply = serviceUntilDone(answer);
            assertFalse(reply.accepted());
            assertEquals("rejected: locked (InMatch)", reply.reason());
            assertEquals(1.5, reply.actualValue());

            // Action run is rejected by the robot with a reason: status Rejected.
            Future<ArcpWireClient.Reply> actionReply = pool.submit(
                    () -> client.awaitReply(client.sendAction(RUN)));
            long deadline = System.currentTimeMillis() + 3_000;
            while (!actionReply.isDone()) {
                server.serviceRequests();
                assertTrue(System.currentTimeMillis() < deadline, "action was never answered");
                Thread.sleep(5);
            }
            ArcpWireClient.Reply rejected = actionReply.get();
            assertEquals(ArcpWireClient.STATUS_REJECTED, rejected.status());

            ArcpStatus status = server.status();
            assertEquals(1, status.sessions());
            assertEquals(1, status.subscriptions());
        }
    }

    @Test
    void acceptedWriteIsAnsweredWithTheRobotValue() throws Exception {
        WriteHandler writes = (signalId, value) -> WriteResult.accept(value);
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), writes, ActionHandler.ACCEPT_ALL);
        server.publishDictionary(dictionary(), List.of());
        try (ArcpWireClient client = new ArcpWireClient(server.controlPort())) {
            client.hello(0);
            Future<ArcpWireClient.WriteAnswer> answer = pool.submit(
                    () -> client.awaitWriteReply(client.sendWrite(ANGLE, 42.0)));
            ArcpWireClient.WriteAnswer reply = serviceUntilDone(answer);
            assertTrue(reply.accepted());
            assertEquals("", reply.reason());
            assertEquals(42.0, reply.actualValue());
        }
    }

    @Test
    void writeWithoutRobotAnswerIsRejectedAfterTheTimeout() throws Exception {
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), WriteHandler.ACCEPT_ALL,
                ActionHandler.ACCEPT_ALL);
        server.publishDictionary(dictionary(), List.of());
        server.publishCycle(1L, cycle(7.0, false));
        try (ArcpWireClient client = new ArcpWireClient(server.controlPort())) {
            client.hello(0);
            ArcpWireClient.WriteAnswer reply = client.awaitWriteReply(client.sendWrite(ANGLE, 9.0));
            assertFalse(reply.accepted());
            assertTrue(reply.reason().contains("did not answer"), reply.reason());
            assertEquals(7.0, reply.actualValue());
        }
    }

    /** Services the robot side until the client's answer arrives, the way a robot cycle would. */
    private ArcpWireClient.WriteAnswer serviceUntilDone(Future<ArcpWireClient.WriteAnswer> answer) throws Exception {
        long deadline = System.currentTimeMillis() + 3_000;
        while (!answer.isDone()) {
            server.serviceRequests();
            assertTrue(System.currentTimeMillis() < deadline, "write was never answered");
            Thread.sleep(5);
        }
        return answer.get(1, TimeUnit.SECONDS);
    }
}
