package ca.frc6390.athena.core;

import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.config.MechanismConfigExport;
import ca.frc6390.athena.mechanisms.config.MechanismConfigFile;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

/**
 * Small HTTP server that exposes data-only mechanism configs from the running robot.
 *
 * <p>Endpoints:
 * - /athena/config/index.json
 * - /athena/config/mechanisms/{name}.json
 * - /athena/config/mechanisms/{name}.toml
 * - /athena/config/all.zip
 */
public final class AthenaConfigServer {
    private static final ObjectMapper INDEX_MAPPER = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

    private final RobotCore<?> robot;
    private final HttpServer server;
    private final int port;

    private AthenaConfigServer(RobotCore<?> robot, HttpServer server, int port) {
        this.robot = robot;
        this.server = server;
        this.port = port;
    }

    public static AthenaConfigServer start(RobotCore<?> robot, int port) {
        Objects.requireNonNull(robot, "robot");
        int p = port > 0 ? port : 5806;
        try {
            HttpServer server = HttpServer.create(new InetSocketAddress(p), 0);
            AthenaConfigServer instance = new AthenaConfigServer(robot, server, p);
            server.createContext("/athena/config/index.json", instance::handleIndex);
            server.createContext("/athena/config/all.zip", instance::handleAllZip);
            server.createContext("/athena/config/mechanisms", instance::handleMechanisms);
            server.setExecutor(Executors.newSingleThreadExecutor(r -> {
                Thread t = new Thread(r, "AthenaConfigServer");
                t.setDaemon(true);
                return t;
            }));
            server.start();
            return instance;
        } catch (IOException e) {
            throw new RuntimeException("Failed to start Athena config HTTP server on port " + p, e);
        }
    }

    public void stop() {
        server.stop(0);
    }

    public int port() {
        return port;
    }

    /**
     * Best-effort base URL that should resolve on the robot network.
     */
    public String baseUrl() {
        int team = edu.wpi.first.wpilibj.RobotController.getTeamNumber();
        String host = team > 0 ? ("roborio-" + team + "-frc.local") : "localhost";
        return "http://" + host + ":" + port;
    }

    private void handleIndex(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        List<Object> mechanisms = new ArrayList<>();
        String base = baseUrl();
        for (Map.Entry<String, Mechanism> e : robot.getMechanisms().entrySet()) {
            if (e == null || e.getKey() == null || e.getValue() == null) {
                continue;
            }
            String name = e.getKey();
            String enc = java.net.URLEncoder.encode(name, StandardCharsets.UTF_8);
            mechanisms.add(Map.of(
                    "name", name,
                    "json", base + "/athena/config/mechanisms/" + enc + ".json",
                    "toml", base + "/athena/config/mechanisms/" + enc + ".toml"));
        }
        Map<String, Object> index = Map.of(
                "baseUrl", base,
                "mechanisms", mechanisms);
        String json;
        try {
            json = INDEX_MAPPER.writeValueAsString(index);
        } catch (JsonProcessingException err) {
            json = "{\"error\":\"failed to build index\"}";
        }
        sendText(ex, 200, json + "\n", "application/json; charset=utf-8");
    }

    private void handleMechanisms(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        URI uri = ex.getRequestURI();
        String path = uri != null ? uri.getPath() : "";
        // Expect: /athena/config/mechanisms/{name}.{ext}
        String prefix = "/athena/config/mechanisms/";
        if (!path.startsWith(prefix)) {
            sendText(ex, 404, "Not Found\n", "text/plain; charset=utf-8");
            return;
        }
        String tail = path.substring(prefix.length());
        int dot = tail.lastIndexOf('.');
        if (dot < 0) {
            sendText(ex, 404, "Not Found\n", "text/plain; charset=utf-8");
            return;
        }
        String rawName = tail.substring(0, dot);
        String ext = tail.substring(dot + 1).toLowerCase();
        String name = URLDecoder.decode(rawName, StandardCharsets.UTF_8);
        Mechanism mech = robot.getMechanisms().get(name);
        if (mech == null) {
            sendText(ex, 404, "Unknown mechanism '" + name + "'\n", "text/plain; charset=utf-8");
            return;
        }
        MechanismConfigFile file = MechanismConfigExport.export(mech);
        if ("json".equals(ext)) {
            sendText(ex, 200, MechanismConfigExport.toJson(file) + "\n", "application/json; charset=utf-8");
            return;
        }
        if ("toml".equals(ext)) {
            sendText(ex, 200, MechanismConfigExport.toToml(file) + "\n", "application/toml; charset=utf-8");
            return;
        }
        sendText(ex, 404, "Not Found\n", "text/plain; charset=utf-8");
    }

    private void handleAllZip(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        try (ZipOutputStream zip = new ZipOutputStream(baos, StandardCharsets.UTF_8)) {
            // Include the index for convenience.
            ZipEntry index = new ZipEntry("index.json");
            zip.putNextEntry(index);
            String idx = INDEX_MAPPER.writeValueAsString(Map.of(
                    "baseUrl", baseUrl(),
                    "mechanisms", robot.getMechanisms().keySet()));
            zip.write((idx + "\n").getBytes(StandardCharsets.UTF_8));
            zip.closeEntry();

            for (Map.Entry<String, Mechanism> e : robot.getMechanisms().entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                String name = e.getKey();
                String safeName = name.replaceAll("[^A-Za-z0-9_\\-]", "_");
                MechanismConfigFile file = MechanismConfigExport.export(e.getValue());

                ZipEntry json = new ZipEntry("mechanisms/" + safeName + ".json");
                zip.putNextEntry(json);
                zip.write((MechanismConfigExport.toJson(file) + "\n").getBytes(StandardCharsets.UTF_8));
                zip.closeEntry();

                ZipEntry toml = new ZipEntry("mechanisms/" + safeName + ".toml");
                zip.putNextEntry(toml);
                zip.write((MechanismConfigExport.toToml(file) + "\n").getBytes(StandardCharsets.UTF_8));
                zip.closeEntry();
            }
        }
        byte[] bytes = baos.toByteArray();
        Headers headers = ex.getResponseHeaders();
        headers.add("Content-Type", "application/zip");
        headers.add("Content-Disposition", "attachment; filename=\"athena-configs.zip\"");
        ex.sendResponseHeaders(200, bytes.length);
        try (OutputStream os = ex.getResponseBody()) {
            os.write(bytes);
        }
    }

    private static void sendText(HttpExchange ex, int status, String body, String contentType) throws IOException {
        byte[] bytes = body != null ? body.getBytes(StandardCharsets.UTF_8) : new byte[0];
        Headers headers = ex.getResponseHeaders();
        if (contentType != null && !contentType.isBlank()) {
            headers.add("Content-Type", contentType);
        }
        ex.sendResponseHeaders(status, bytes.length);
        try (OutputStream os = ex.getResponseBody()) {
            os.write(bytes);
        }
    }
}
