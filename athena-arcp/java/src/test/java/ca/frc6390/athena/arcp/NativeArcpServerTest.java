package ca.frc6390.athena.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;

import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.io.InputStream;
import java.io.ByteArrayOutputStream;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicInteger;
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
        AtomicInteger executed = new AtomicInteger();
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), (id, value) -> {
            executed.incrementAndGet(); return WriteResult.accept(value);
        }, ActionHandler.ACCEPT_ALL);
        server.publishDictionary(dictionary(), List.of());
        server.publishCycle(1L, cycle(7.0, false));
        try (ArcpWireClient client = new ArcpWireClient(server.controlPort())) {
            client.hello(0);
            ArcpWireClient.WriteAnswer reply = client.awaitWriteReply(client.sendWrite(ANGLE, 9.0));
            assertFalse(reply.accepted());
            assertTrue(reply.reason().contains("did not answer"), reply.reason());
            assertEquals(7.0, reply.actualValue());
            server.serviceRequests();
            assertEquals(0, executed.get(), "timed-out writes must not execute when robot servicing resumes");
        }
    }

    @Test
    void stalledSchemaReaderCannotDelayRobotPublicationOrHealthyClients() throws Exception {
        List<ArcpDictEntry> large = new ArrayList<>();
        for (int i = 1; i <= 1200; i++) {
            large.add(new ArcpDictEntry(i,
                    "robot/drivetrain/frontLeft/encoder/telemetry/diagnostic/channel/position/" + i,
                    3, "rot", ArcpDictEntry.ROLE_STATE, 0, "", 0));
        }
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), WriteHandler.ACCEPT_ALL,
                ActionHandler.ACCEPT_ALL);
        server.publishDictionary(large, List.of());
        try (ArcpWireClient healthy = new ArcpWireClient(server.controlPort()); Socket slow = new Socket()) {
            healthy.hello(0);
            slow.setReceiveBufferSize(65536);
            slow.connect(new java.net.InetSocketAddress(InetAddress.getLoopbackAddress(), server.controlPort()));
            slow.setSoTimeout(3000);
            InputStream input = slow.getInputStream();
            byte[] header = input.readNBytes(4);
            int length = (header[0] & 255) | ((header[1] & 255) << 8)
                    | ((header[2] & 255) << 16) | ((header[3] & 255) << 24);
            assertEquals(length, input.readNBytes(length).length);
            slow.setReceiveBufferSize(1024);
            ByteArrayOutputStream requests = new ByteArrayOutputStream();
            for (int i = 1; i <= 64; i++) {
                requests.write(new byte[] {5, 0, 0, 0, 0x40, (byte) i, 0, 0, 0});
            }
            slow.getOutputStream().write(requests.toByteArray());
            slow.getOutputStream().flush();
            Thread.sleep(100);
            assertTimeoutPreemptively(Duration.ofMillis(250), () -> {
                server.publishDictionary(large, List.of());
                server.publishCycle(1_000_000L, cycle(1.5, true));
                server.status();
            });
            // A slow outbox must not poison the other client's ordered schema and control replies.
            healthy.hello(0);
        }
    }

    @Test
    void blockedBackfillRunsOffRobotThreadWhileWritesStillComplete() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicReference<String> worker = new AtomicReference<>();
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), WriteHandler.ACCEPT_ALL,
                ActionHandler.ACCEPT_ALL);
        server.backfill((from, to) -> {
            worker.set(Thread.currentThread().getName());
            started.countDown();
            try {
                if (!release.await(2, TimeUnit.SECONDS)) throw new IllegalStateException("worker not released");
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException(interrupted);
            }
            return List.of(new byte[] {1, 2, 3});
        });
        try (ArcpWireClient history = new ArcpWireClient(server.controlPort());
                ArcpWireClient controls = new ArcpWireClient(server.controlPort())) {
            history.hello(0);
            controls.hello(0);
            int request = history.sendBackfill(1, 3);
            Future<ArcpWireClient.BackfillAnswer> answer = pool.submit(() -> history.awaitBackfill(request));
            pumpUntil(() -> started.getCount() == 0);
            assertEquals("athena-arcp-backfill", worker.get());
            int write = controls.sendWrite(ANGLE, 42.0);
            ArcpWireClient.WriteAnswer written = serviceUntilDone(pool.submit(() -> controls.awaitWriteReply(write)));
            assertTrue(written.accepted(), "a blocked history provider must not block robot controls");
            assertFalse(answer.isDone());
            release.countDown();
            ArcpWireClient.BackfillAnswer result = answer.get(2, TimeUnit.SECONDS);
            assertEquals(ArcpWireClient.STATUS_OK, result.status(), result.reason());
            assertArrayEquals(new byte[] {3, 1, 2, 3}, result.records());
        } finally {
            release.countDown();
        }
    }

    @Test
    void backfillUsesBoundedProviderAndReturnsExplicitProviderFailure() throws Exception {
        AtomicInteger budget = new AtomicInteger();
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), WriteHandler.ACCEPT_ALL,
                ActionHandler.ACCEPT_ALL);
        server.backfill(new ArcpBackfill() {
            @Override public List<byte[]> records(long from, long to) {
                throw new AssertionError("the bounded overload must be used");
            }
            @Override public List<byte[]> records(long from, long to, int maxBytes) {
                budget.set(maxBytes);
                if (from == 5) throw new IllegalStateException("history unavailable");
                if (from == 9) return List.of(new byte[maxBytes]); // Its length prefix cannot fit.
                return List.of(new byte[maxBytes - 3]);
            }
        });
        try (ArcpWireClient client = new ArcpWireClient(server.controlPort())) {
            client.hello(0);
            int first = client.sendBackfill(1, 3);
            Future<ArcpWireClient.BackfillAnswer> answer = pool.submit(() -> client.awaitBackfill(first));
            pumpUntil(answer::isDone);
            ArcpWireClient.BackfillAnswer result = answer.get(2, TimeUnit.SECONDS);
            assertEquals(ArcpWireClient.STATUS_OK, result.status(), result.reason());
            assertEquals((1 << 20) - 64, budget.get());
            assertEquals(budget.get(), result.records().length, "the length prefix is part of the byte budget");
            int failed = client.sendBackfill(5, 6);
            Future<ArcpWireClient.BackfillAnswer> failure = pool.submit(() -> client.awaitBackfill(failed));
            pumpUntil(failure::isDone);
            assertTrue(failure.get().reason().contains("history unavailable"));
            assertFalse(failure.get().status() == ArcpWireClient.STATUS_OK);
            int large = client.sendBackfill(9, 10);
            Future<ArcpWireClient.BackfillAnswer> oversized = pool.submit(() -> client.awaitBackfill(large));
            pumpUntil(oversized::isDone);
            assertTrue(oversized.get().reason().contains("response limit"));
        }
    }

    @Test
    void fullBackfillQueueRejectsExcessRequestsWithoutDroppingAcceptedOnes() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        AtomicInteger served = new AtomicInteger();
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), WriteHandler.ACCEPT_ALL,
                ActionHandler.ACCEPT_ALL);
        server.backfill((from, to) -> {
            served.incrementAndGet();
            started.countDown();
            try {
                if (!release.await(2, TimeUnit.SECONDS)) throw new IllegalStateException("worker not released");
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException(interrupted);
            }
            return List.of(new byte[] {(byte) from});
        });
        List<ArcpWireClient> clients = new ArrayList<>();
        List<Future<ArcpWireClient.BackfillAnswer>> answers = new ArrayList<>();
        try {
            for (int i = 0; i < 4; i++) {
                ArcpWireClient client = new ArcpWireClient(server.controlPort());
                clients.add(client);
                client.hello(0);
            }
            int first = clients.get(0).sendBackfill(1, 1);
            answers.add(pool.submit(() -> clients.get(0).awaitBackfill(first)));
            pumpUntil(() -> started.getCount() == 0);
            for (int i = 1; i < clients.size(); i++) {
                ArcpWireClient client = clients.get(i);
                int request = client.sendBackfill(i + 1, i + 1);
                answers.add(pool.submit(() -> client.awaitBackfill(request)));
            }
            pumpUntil(() -> answers.stream().anyMatch(Future::isDone));
            Future<ArcpWireClient.BackfillAnswer> rejected = answers.stream().filter(Future::isDone)
                    .findFirst().orElseThrow();
            assertTrue(rejected.get().reason().contains("queue full"), rejected.get().reason());
            release.countDown();
            int success = 0;
            for (Future<ArcpWireClient.BackfillAnswer> answer : answers) {
                ArcpWireClient.BackfillAnswer response = answer.get(2, TimeUnit.SECONDS);
                if (response.status() == ArcpWireClient.STATUS_OK) success++;
            }
            assertEquals(3, success);
            assertEquals(3, served.get());
        } finally {
            release.countDown();
            for (ArcpWireClient client : clients) client.close();
        }
    }

    @Test
    void closingDoesNotHoldServerMonitorWhileBackfillWorkerFinishes() throws Exception {
        CountDownLatch started = new CountDownLatch(1);
        CountDownLatch finish = new CountDownLatch(1);
        AtomicReference<ArcpStatus> seen = new AtomicReference<>();
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), WriteHandler.ACCEPT_ALL,
                ActionHandler.ACCEPT_ALL);
        server.backfill((from, to) -> {
            started.countDown();
            try {
                if (!finish.await(2, TimeUnit.SECONDS)) throw new IllegalStateException("worker not released");
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException(interrupted);
            }
            seen.set(server.status()); // Models a recorder lifecycle callback acquiring the server monitor.
            return List.of();
        });
        try (ArcpWireClient client = new ArcpWireClient(server.controlPort())) {
            client.hello(0);
            int request = client.sendBackfill(1, 3);
            Future<ArcpWireClient.BackfillAnswer> reply = pool.submit(() -> client.awaitBackfill(request));
            pumpUntil(() -> started.getCount() == 0);
            CountDownLatch closing = new CountDownLatch(1);
            Future<?> closed = pool.submit(() -> {
                closing.countDown();
                server.close();
            });
            assertTrue(closing.await(1, TimeUnit.SECONDS));
            Thread.sleep(20);
            finish.countDown();
            closed.get(1, TimeUnit.SECONDS);
            assertEquals(ArcpStatus.STOPPED, seen.get());
            assertEquals(ArcpWireClient.STATUS_OK, reply.get(1, TimeUnit.SECONDS).status());
        } finally {
            finish.countDown();
        }
    }

    @Test
    void callbackBatchStopsAfterOneSlowCallbackAndRejectsTheRestExplicitly() throws Exception {
        AtomicInteger callbacks = new AtomicInteger();
        server = new NativeArcpServer(NativeArcpServer.Config.defaults(), (id, value) -> {
            callbacks.incrementAndGet();
            java.util.concurrent.locks.LockSupport.parkNanos(TimeUnit.MILLISECONDS.toNanos(5));
            return WriteResult.accept(value);
        }, ActionHandler.ACCEPT_ALL);
        server.publishDictionary(dictionary(), List.of());
        List<ArcpWireClient> clients = new ArrayList<>();
        List<Future<ArcpWireClient.WriteAnswer>> answers = new ArrayList<>();
        try {
            for (int i = 0; i < 8; i++) {
                ArcpWireClient client = new ArcpWireClient(server.controlPort());
                clients.add(client);
                client.hello(0);
            }
            for (ArcpWireClient client : clients) {
                int request = client.sendWrite(ANGLE, 42.0);
                answers.add(pool.submit(() -> client.awaitWriteReply(request)));
            }
            Thread.sleep(40); // Let all independent native connection threads enqueue their requests.
            int before = callbacks.get();
            server.serviceRequests();
            assertEquals(before + 1, callbacks.get(), "budget checked between non-preemptible callbacks");
            pumpUntil(() -> answers.stream().allMatch(Future::isDone));
            assertTrue(server.rejectedCallbacks() > 0);
            assertTrue(server.maxCallbackNanos() >= TimeUnit.MILLISECONDS.toNanos(1));
            assertTrue(server.publicationFailure().isEmpty());
            long rejected = 0;
            for (var answer : answers) {
                var reply = answer.get(1, TimeUnit.SECONDS);
                if (!reply.accepted()) {
                    assertTrue(reply.reason().contains("budget")); rejected++;
                }
            }
            assertTrue(rejected > 0);
        } finally { for (ArcpWireClient client : clients) client.close(); }
    }

    private void pumpUntil(java.util.function.BooleanSupplier complete) throws Exception {
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
        while (!complete.getAsBoolean()) {
            assertTimeoutPreemptively(Duration.ofMillis(100), server::serviceRequests);
            assertTrue(System.nanoTime() < deadline, "request did not complete");
            Thread.sleep(2);
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
