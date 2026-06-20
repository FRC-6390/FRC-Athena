package ca.frc6390.athena.examples;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import ca.frc6390.athena.dashboard.DashboardControlMessage;
import ca.frc6390.athena.dashboard.DashboardControlRegistry;
import ca.frc6390.athena.dashboard.DashboardPacket;
import ca.frc6390.athena.dashboard.DashboardPublisher;
import ca.frc6390.athena.dashboard.DashboardTcpServer;
import ca.frc6390.athena.dashboard.DashboardWireCodec;

/**
 * Example dashboard/control bridge usage.
 */
public final class DashboardBridgeExample {
    private DashboardBridgeExample() {
    }

    /**
     * Publishes telemetry and diagnostics to an in-memory dashboard sink.
     *
     * @return published dashboard packets
     */
    public static List<DashboardPacket> publishSnapshot() {
        List<DashboardPacket> packets = new ArrayList<>();
        var publisher = new DashboardPublisher(packets::add);
        publisher.publish(
                RobotTelemetry.create(true, 4500.0).snapshot(),
                List.of(DiagnosticsExample.shooterSnapshot()));
        return packets;
    }

    /**
     * Encodes the current dashboard packet as a transport payload.
     *
     * @return JSON dashboard packet payload
     */
    public static String encodeSnapshot() {
        return DashboardWireCodec.encodePacket(publishSnapshot().get(0));
    }

    /**
     * Handles one dashboard control message.
     *
     * @return selected mode after dispatch
     */
    public static String dispatchControl() {
        AtomicReference<String> selectedMode = new AtomicReference<>("idle");
        var controls = new DashboardControlRegistry()
                .on("setMode", message -> selectedMode.set(message.fields().get("mode")));

        controls.dispatch(DashboardControlMessage.of("setMode", "mode", "score"));
        return selectedMode.get();
    }

    /**
     * Round-trips a control message through the dashboard wire shape.
     *
     * @return decoded control message
     */
    public static DashboardControlMessage decodeControlPayload() {
        String payload = DashboardWireCodec.encodeControl(DashboardControlMessage.of("setMode", "mode", "score"));
        return DashboardWireCodec.decodeControl(payload);
    }

    /**
     * Starts the optional TCP dashboard transport and publishes one packet.
     *
     * @return started server, owned by the caller
     * @throws IOException when the dashboard port cannot be opened
     */
    public static DashboardTcpServer startTcpDashboard() throws IOException {
        DashboardTcpServer server = DashboardTcpServer.start(DashboardTcpServer.DEFAULT_PORT, message -> { });
        var publisher = new DashboardPublisher(server);
        publisher.publish(
                RobotTelemetry.create(true, 4500.0).snapshot(),
                List.of(DiagnosticsExample.shooterSnapshot()));
        return server;
    }
}
