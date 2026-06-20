package ca.frc6390.athena.dashboard;

/**
 * Transport implemented by dashboard or ARCP integrations.
 */
@FunctionalInterface
public interface DashboardSink {
    /**
     * Publishes a dashboard packet.
     *
     * @param packet packet to publish
     */
    void publish(DashboardPacket packet);
}
