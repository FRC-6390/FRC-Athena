package ca.frc6390.athena.dashboard;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

/**
 * TCP dashboard transport using newline-delimited dashboard JSON payloads.
 */
public final class DashboardTcpServer implements DashboardSink, AutoCloseable {
    /** Default port for Athena dashboard traffic. */
    public static final int DEFAULT_PORT = 6390;

    private final ServerSocket serverSocket;
    private final Consumer<DashboardControlMessage> controls;
    private final ExecutorService executor;
    private final List<ClientConnection> clients = new CopyOnWriteArrayList<>();
    private final AtomicBoolean running = new AtomicBoolean();

    private DashboardTcpServer(
            ServerSocket serverSocket,
            Consumer<DashboardControlMessage> controls,
            ThreadFactory threadFactory) {
        this.serverSocket = Objects.requireNonNull(serverSocket, "serverSocket");
        this.controls = Objects.requireNonNull(controls, "controls");
        this.executor = Executors.newCachedThreadPool(threadFactory);
    }

    /**
     * Starts a server on the default Athena dashboard port.
     *
     * @param controls dashboard-to-robot control handler
     * @return started server
     * @throws IOException when the port cannot be opened
     */
    public static DashboardTcpServer start(Consumer<DashboardControlMessage> controls) throws IOException {
        return start(DEFAULT_PORT, controls);
    }

    /**
     * Starts a loopback server for tests and local tools.
     *
     * @param port dashboard port, or {@code 0} to allocate one
     * @param controls dashboard-to-robot control handler
     * @return started server
     * @throws IOException when the port cannot be opened
     */
    public static DashboardTcpServer startLoopback(int port, Consumer<DashboardControlMessage> controls)
            throws IOException {
        return start(new ServerSocket(port, 50, InetAddress.getLoopbackAddress()), controls);
    }

    /**
     * Starts a server on every local interface.
     *
     * @param port dashboard port, or {@code 0} to allocate one
     * @param controls dashboard-to-robot control handler
     * @return started server
     * @throws IOException when the port cannot be opened
     */
    public static DashboardTcpServer start(int port, Consumer<DashboardControlMessage> controls) throws IOException {
        return start(new ServerSocket(port), controls);
    }

    private static DashboardTcpServer start(ServerSocket serverSocket, Consumer<DashboardControlMessage> controls) {
        var server = new DashboardTcpServer(serverSocket, controls, new DashboardThreadFactory());
        server.startAccepting();
        return server;
    }

    /**
     * Returns the bound TCP port.
     *
     * @return local port
     */
    public int port() {
        return serverSocket.getLocalPort();
    }

    /**
     * Returns the number of connected dashboard clients.
     *
     * @return client count
     */
    public int clientCount() {
        return clients.size();
    }

    @Override
    public void publish(DashboardPacket packet) {
        String payload = DashboardWireCodec.encodePacket(packet);
        List<ClientConnection> stale = new ArrayList<>();
        for (ClientConnection client : clients) {
            if (!client.send(payload)) {
                stale.add(client);
            }
        }
        stale.forEach(this::removeClient);
    }

    @Override
    public void close() {
        running.set(false);
        try {
            serverSocket.close();
        } catch (IOException ignored) {
            // Closing is best-effort during robot shutdown.
        }
        clients.forEach(ClientConnection::close);
        clients.clear();
        executor.shutdownNow();
    }

    private void startAccepting() {
        running.set(true);
        executor.execute(this::acceptLoop);
    }

    private void acceptLoop() {
        while (running.get()) {
            try {
                Socket socket = serverSocket.accept();
                var client = new ClientConnection(socket);
                clients.add(client);
                executor.execute(() -> readControls(client));
            } catch (IOException exception) {
                if (running.get()) {
                    throw new DashboardTransportException("Dashboard server accept failed.", exception);
                }
            }
        }
    }

    private void readControls(ClientConnection client) {
        try (client) {
            String line;
            while (running.get() && (line = client.readLine()) != null) {
                if (!line.isBlank()) {
                    controls.accept(DashboardWireCodec.decodeControl(line));
                }
            }
        } catch (IOException exception) {
            if (running.get()) {
                throw new DashboardTransportException("Dashboard client read failed.", exception);
            }
        } finally {
            removeClient(client);
        }
    }

    private void removeClient(ClientConnection client) {
        clients.remove(client);
        client.close();
    }

    private static final class ClientConnection implements AutoCloseable {
        private final Socket socket;
        private final BufferedReader reader;
        private final BufferedWriter writer;

        ClientConnection(Socket socket) throws IOException {
            this.socket = socket;
            reader = new BufferedReader(new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));
            writer = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream(), StandardCharsets.UTF_8));
        }

        String readLine() throws IOException {
            return reader.readLine();
        }

        boolean send(String payload) {
            try {
                synchronized (writer) {
                    writer.write(payload);
                    writer.newLine();
                    writer.flush();
                }
                return true;
            } catch (IOException exception) {
                return false;
            }
        }

        @Override
        public void close() {
            try {
                socket.close();
            } catch (IOException ignored) {
                // Socket may already be closed by the peer.
            }
        }
    }

    private static final class DashboardThreadFactory implements ThreadFactory {
        @Override
        public Thread newThread(Runnable runnable) {
            Thread thread = new Thread(runnable, "athena-dashboard-tcp");
            thread.setDaemon(true);
            return thread;
        }
    }
}
