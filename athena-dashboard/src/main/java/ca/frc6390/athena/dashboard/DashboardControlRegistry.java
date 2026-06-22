package ca.frc6390.athena.dashboard;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

/**
 * Registry of dashboard control handlers.
 */
public final class DashboardControlRegistry {
    private final Map<String, Consumer<DashboardControlMessage>> handlers = new ConcurrentHashMap<>();

    /**
     * Registers a handler for a control message.
     *
     * @param name control name
     * @param handler message handler
     * @return this registry
     */
    public DashboardControlRegistry on(String name, Consumer<DashboardControlMessage> handler) {
        if (handler != null) {
            handlers.put(normalize(name), handler);
        }
        return this;
    }

    /**
     * Finds a registered handler.
     *
     * @param name control name
     * @return handler if present
     */
    public Optional<Consumer<DashboardControlMessage>> find(String name) {
        return Optional.ofNullable(handlers.get(normalize(name)));
    }

    /**
     * Dispatches a message to its handler.
     *
     * @param message control message
     * @return true if handled
     */
    public boolean dispatch(DashboardControlMessage message) {
        DashboardControlMessage safeMessage = message == null ? DashboardControlMessage.of("control") : message;
        Consumer<DashboardControlMessage> handler = handlers.get(normalize(safeMessage.name()));
        if (handler == null) {
            return false;
        }
        handler.accept(safeMessage);
        return true;
    }

    private static String normalize(String name) {
        return name == null || name.isBlank() ? "control" : name.trim();
    }
}
