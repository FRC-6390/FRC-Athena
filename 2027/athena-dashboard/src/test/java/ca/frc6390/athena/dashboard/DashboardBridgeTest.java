package ca.frc6390.athena.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.runtime.diagnostics.DiagnosticsChannel;
import ca.frc6390.athena.telemetry.TelemetryKey;
import ca.frc6390.athena.telemetry.TelemetryRegistry;
import ca.frc6390.athena.telemetry.TelemetryValue;

class DashboardBridgeTest {
    @Test
    void packetCapturesTelemetryAndDiagnostics() {
        var telemetry = new TelemetryRegistry()
                .booleanValue(TelemetryKey.bool("enabled"), () -> true)
                .snapshot();
        var diagnostics = new DiagnosticsChannel("shooter", 4)
                .summary("state", "speaker");
        diagnostics.error("encoder disconnected");

        var packet = DashboardPacket.now(telemetry, List.of(diagnostics.snapshot(4)));

        assertTrue(packet.hasErrors());
        assertEquals(TelemetryValue.of(true), packet.telemetry().find(TelemetryKey.bool("enabled")).orElseThrow());
        assertEquals("shooter", packet.diagnostics().get(0).channel());
    }

    @Test
    void publisherSendsPacketToSink() {
        List<DashboardPacket> packets = new ArrayList<>();
        var publisher = new DashboardPublisher(packets::add);
        var telemetry = new TelemetryRegistry().snapshot();

        var packet = publisher.publish(telemetry, List.of());

        assertEquals(packet, packets.get(0));
        assertFalse(packet.hasErrors());
    }

    @Test
    void controlRegistryDispatchesNamedMessages() {
        AtomicReference<String> selectedMode = new AtomicReference<>("idle");
        var registry = new DashboardControlRegistry()
                .on("setMode", message -> selectedMode.set(message.fields().get("mode")));

        boolean handled = registry.dispatch(DashboardControlMessage.of("setMode", "mode", "score"));
        boolean missing = registry.dispatch(DashboardControlMessage.of("unknown"));

        assertTrue(handled);
        assertFalse(missing);
        assertEquals("score", selectedMode.get());
    }

    @Test
    void wireCodecEncodesPacketsForTransport() {
        var telemetry = new TelemetryRegistry()
                .booleanValue(TelemetryKey.bool("enabled"), () -> true)
                .numberValue(TelemetryKey.number("rpm"), () -> 4500.0)
                .stringValue(TelemetryKey.string("mode"), () -> "speaker")
                .snapshot();
        var diagnostics = new DiagnosticsChannel("shooter", 4)
                .summary("state", "speaker");
        diagnostics.error("encoder \"A\" disconnected");

        String json = DashboardWireCodec.encodePacket(DashboardPacket.now(telemetry, List.of(diagnostics.snapshot(4))));

        assertTrue(json.contains("\"hasErrors\":true"));
        assertTrue(json.contains("\"path\":\"enabled\""));
        assertTrue(json.contains("\"type\":\"BOOLEAN\""));
        assertTrue(json.contains("\"value\":4500.0"));
        assertTrue(json.contains("\"channel\":\"shooter\""));
        assertTrue(json.contains("\"message\":\"encoder \\\"A\\\" disconnected\""));
    }

    @Test
    void wireCodecRoundTripsControlMessages() {
        var message = new DashboardControlMessage(
                "setMode",
                Map.of("mode", "score", "note", "speaker \"fast\""));

        String json = DashboardWireCodec.encodeControl(message);
        DashboardControlMessage decoded = DashboardWireCodec.decodeControl(json);

        assertEquals(message.name(), decoded.name());
        assertEquals(message.fields(), decoded.fields());
    }

    @Test
    void tcpServerPublishesPacketsToConnectedDashboard() throws Exception {
        try (var server = DashboardTcpServer.startLoopback(0, message -> { });
                var socket = new Socket(InetAddress.getLoopbackAddress(), server.port());
                var reader = new BufferedReader(
                        new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8))) {
            waitForClient(server);
            var telemetry = new TelemetryRegistry()
                    .numberValue(TelemetryKey.number("rpm"), () -> 4200.0)
                    .snapshot();

            server.publish(DashboardPacket.now(telemetry, List.of()));

            String payload = reader.readLine();
            assertTrue(payload.contains("\"path\":\"rpm\""));
            assertTrue(payload.contains("\"value\":4200.0"));
        }
    }

    @Test
    void tcpServerReceivesDashboardControls() throws Exception {
        CountDownLatch received = new CountDownLatch(1);
        AtomicReference<DashboardControlMessage> control = new AtomicReference<>();
        try (var server = DashboardTcpServer.startLoopback(0, message -> {
                    control.set(message);
                    received.countDown();
                });
                var socket = new Socket(InetAddress.getLoopbackAddress(), server.port());
                var writer = new BufferedWriter(
                        new OutputStreamWriter(socket.getOutputStream(), StandardCharsets.UTF_8))) {
            writer.write(DashboardWireCodec.encodeControl(DashboardControlMessage.of("setMode", "mode", "score")));
            writer.newLine();
            writer.flush();

            assertTrue(received.await(2, TimeUnit.SECONDS));
            assertEquals("setMode", control.get().name());
            assertEquals("score", control.get().fields().get("mode"));
        }
    }

    private static void waitForClient(DashboardTcpServer server) throws InterruptedException {
        long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
        while (server.clientCount() == 0 && System.nanoTime() < deadline) {
            Thread.sleep(10);
        }
        assertEquals(1, server.clientCount());
    }
}
