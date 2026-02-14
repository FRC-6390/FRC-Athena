package ca.frc6390.athena.core;

import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.config.MechanismConfigExport;
import ca.frc6390.athena.mechanisms.config.MechanismConfigFile;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.Executors;
import java.util.function.IntFunction;
import java.util.function.Supplier;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Small HTTP server that exposes data-only mechanism configs from the running robot.
 *
 * <p>Endpoints:
 * - /Athena/config/index.json (also available at /athena/config/index.json)
 * - /Athena/config/index.toml (also available at /athena/config/index.toml)
 * - /Athena/config/mechanisms/{name}.json (also available at /athena/config/mechanisms/{name}.json)
 * - /Athena/config/mechanisms/{name}.toml (also available at /athena/config/mechanisms/{name}.toml)
 * - /Athena/config/all.zip (also available at /athena/config/all.zip)
 * - /Athena/config/custom (also available at /athena/config/custom)
 * - /Athena/config/custom/{key} (also available at /athena/config/custom/{key})
 * - /Athena/diagnostics (also available at /athena/diagnostics)
 * - /Athena/diagnostics/{key}.json (also available at /athena/diagnostics/{key}.json)
 * - /Athena/diagnostics/history.json (also available at /athena/diagnostics/history.json)
 * - /Athena/auto/log (also available at /athena/auto/log)
 * - /Athena/mechanisms/log (also available at /athena/mechanisms/log)
 * - /Athena/mechanisms/log/{name}.json (also available at /athena/mechanisms/log/{name}.json)
 */
public final class AthenaRuntimeServer {
    private static final ObjectMapper INDEX_MAPPER = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    private static final int MAX_CUSTOM_PAYLOAD_BYTES = 1024 * 1024;
    private static final int MAX_HISTORY_LIMIT = 16384;
    private static final String MECHANISM_LOG_HTML_RESOURCE = "/ca/frc6390/athena/core/mechanism-log.html";

    private final RobotCore<?> robot;
    private final HttpServer server;
    private final int port;
    private final ConcurrentMap<String, CustomConfigEntry> customConfigEntries = new ConcurrentHashMap<>();
    private final ConcurrentMap<String, DiagnosticsProviderEntry> diagnosticsProviders = new ConcurrentHashMap<>();

    private record CustomConfigEntry(String contentType, byte[] body, double updatedAtSeconds) {
    }

    private record DiagnosticsProviderEntry(
            Supplier<Map<String, Object>> summarySupplier,
            IntFunction<Map<String, Object>> snapshotSupplier,
            Runnable clearAction) {
    }

    private AthenaRuntimeServer(RobotCore<?> robot, HttpServer server, int port) {
        this.robot = robot;
        this.server = server;
        this.port = port;
    }

    public static AthenaRuntimeServer start(RobotCore<?> robot, int port) {
        Objects.requireNonNull(robot, "robot");
        int p = port > 0 ? port : 5806;
        try {
            HttpServer server = HttpServer.create(new InetSocketAddress(p), 0);
            AthenaRuntimeServer instance = new AthenaRuntimeServer(robot, server, p);
            // Provide both /athena and /Athena paths for convenience (HTTP paths are case-sensitive).
            server.createContext("/athena/config/index.json", instance::handleIndexJson);
            server.createContext("/Athena/config/index.json", instance::handleIndexJson);
            server.createContext("/athena/config/index.toml", instance::handleIndexToml);
            server.createContext("/Athena/config/index.toml", instance::handleIndexToml);

            server.createContext("/athena/config/all.zip", instance::handleAllZip);
            server.createContext("/Athena/config/all.zip", instance::handleAllZip);

            server.createContext("/athena/config/mechanisms", instance::handleMechanisms);
            server.createContext("/Athena/config/mechanisms", instance::handleMechanisms);
            server.createContext("/athena/config/custom", instance::handleCustomConfig);
            server.createContext("/Athena/config/custom", instance::handleCustomConfig);
            server.createContext("/athena/diagnostics", instance::handleDiagnostics);
            server.createContext("/Athena/diagnostics", instance::handleDiagnostics);
            server.createContext("/athena/auto/log", instance::handleAutoLog);
            server.createContext("/Athena/auto/log", instance::handleAutoLog);
            server.createContext("/athena/mechanisms/log", instance::handleMechanismLogs);
            server.createContext("/Athena/mechanisms/log", instance::handleMechanismLogs);
            server.setExecutor(Executors.newSingleThreadExecutor(r -> {
                Thread t = new Thread(r, "AthenaRuntimeServer");
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

    public void putCustomJson(String key, Object payload) {
        Objects.requireNonNull(payload, "payload");
        byte[] body;
        try {
            body = INDEX_MAPPER.writeValueAsBytes(payload);
        } catch (JsonProcessingException ex) {
            throw new IllegalArgumentException("Failed to serialize custom JSON payload: " + ex.getMessage(), ex);
        }
        putCustomBytes(key, "application/json; charset=utf-8", body);
    }

    public void putCustomText(String key, String text) {
        String value = text != null ? text : "";
        putCustomBytes(key, "text/plain; charset=utf-8", value.getBytes(StandardCharsets.UTF_8));
    }

    public void putCustomBytes(String key, String contentType, byte[] payload) {
        String normalizedKey = normalizeCustomKey(key);
        byte[] body = payload != null ? payload.clone() : new byte[0];
        if (body.length > MAX_CUSTOM_PAYLOAD_BYTES) {
            throw new IllegalArgumentException("custom payload exceeds " + MAX_CUSTOM_PAYLOAD_BYTES + " bytes");
        }
        String resolvedContentType =
                contentType != null && !contentType.isBlank()
                        ? contentType.trim()
                        : "application/octet-stream";
        customConfigEntries.put(
                normalizedKey,
                new CustomConfigEntry(
                        resolvedContentType,
                        body,
                        edu.wpi.first.wpilibj.Timer.getFPGATimestamp()));
    }

    public boolean removeCustom(String key) {
        String normalizedKey = normalizeCustomKey(key);
        return customConfigEntries.remove(normalizedKey) != null;
    }

    public void clearCustom() {
        customConfigEntries.clear();
    }

    public void registerDiagnosticsProvider(
            String key,
            Supplier<Map<String, Object>> summarySupplier,
            IntFunction<Map<String, Object>> snapshotSupplier) {
        registerDiagnosticsProvider(key, summarySupplier, snapshotSupplier, null);
    }

    public void registerDiagnosticsProvider(
            String key,
            Supplier<Map<String, Object>> summarySupplier,
            IntFunction<Map<String, Object>> snapshotSupplier,
            Runnable clearAction) {
        String normalizedKey = normalizeDiagnosticsKey(key);
        diagnosticsProviders.put(
                normalizedKey,
                new DiagnosticsProviderEntry(
                        Objects.requireNonNull(summarySupplier, "summarySupplier"),
                        Objects.requireNonNull(snapshotSupplier, "snapshotSupplier"),
                        clearAction));
    }

    public boolean removeDiagnosticsProvider(String key) {
        String normalizedKey = normalizeDiagnosticsKey(key);
        return diagnosticsProviders.remove(normalizedKey) != null;
    }

    public void clearDiagnosticsProviders() {
        diagnosticsProviders.clear();
    }

    private static String athenaPathPrefix(URI uri) {
        // Decide which path prefix to advertise in links. Prefer /Athena by default.
        String path = uri != null ? uri.getPath() : "";
        if (path != null && path.startsWith("/athena/")) {
            return "/athena";
        }
        return "/Athena";
    }

    private void handleIndexJson(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        List<Object> mechanisms = new ArrayList<>();
        String base = baseUrl();
        String prefix = athenaPathPrefix(ex.getRequestURI());
        for (Map.Entry<String, Mechanism> e : robot.getMechanisms().entrySet()) {
            if (e == null || e.getKey() == null || e.getValue() == null) {
                continue;
            }
            String name = e.getKey();
            String enc = java.net.URLEncoder.encode(name, StandardCharsets.UTF_8);
            mechanisms.add(Map.of(
                    "name", name,
                    "json", base + prefix + "/config/mechanisms/" + enc + ".json",
                    "toml", base + prefix + "/config/mechanisms/" + enc + ".toml",
                    "log", base + prefix + "/mechanisms/log/" + enc + ".json"));
        }
        Map<String, Object> index = new LinkedHashMap<>();
        index.put("baseUrl", base);
        index.put("indexJsonUrl", base + prefix + "/config/index.json");
        index.put("indexTomlUrl", base + prefix + "/config/index.toml");
        index.put("allZipUrl", base + prefix + "/config/all.zip");
        index.put("mechanismsBaseUrl", base + prefix + "/config/mechanisms/");
        index.put("customConfigUrl", base + prefix + "/config/custom");
        index.put("customConfigBaseUrl", base + prefix + "/config/custom/");
        index.put("diagnosticsUrl", base + prefix + "/diagnostics");
        index.put("diagnosticsBaseUrl", base + prefix + "/diagnostics/");
        index.put("diagnosticsHistoryUrl", base + prefix + "/diagnostics/history.json");
        index.put("autoLogUrl", base + prefix + "/auto/log");
        index.put("mechanismLogUrl", base + prefix + "/mechanisms/log");
        index.put("mechanismLogBaseUrl", base + prefix + "/mechanisms/log/");
        index.put("mechanisms", mechanisms);
        String json;
        try {
            json = INDEX_MAPPER.writeValueAsString(index);
        } catch (JsonProcessingException err) {
            json = "{\"error\":\"failed to build index\"}";
        }
        sendText(ex, 200, json + "\n", "application/json; charset=utf-8");
    }

    private void handleIndexToml(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        String base = baseUrl();
        String prefix = athenaPathPrefix(ex.getRequestURI());

        StringBuilder sb = new StringBuilder(2048);
        sb.append("baseUrl = ").append(tomlQuote(base)).append("\n");
        sb.append("indexJsonUrl = ").append(tomlQuote(base + prefix + "/config/index.json")).append("\n");
        sb.append("indexTomlUrl = ").append(tomlQuote(base + prefix + "/config/index.toml")).append("\n");
        sb.append("allZipUrl = ").append(tomlQuote(base + prefix + "/config/all.zip")).append("\n");
        sb.append("mechanismsBaseUrl = ").append(tomlQuote(base + prefix + "/config/mechanisms/")).append("\n");
        sb.append("customConfigUrl = ").append(tomlQuote(base + prefix + "/config/custom")).append("\n");
        sb.append("customConfigBaseUrl = ").append(tomlQuote(base + prefix + "/config/custom/")).append("\n");
        sb.append("diagnosticsUrl = ").append(tomlQuote(base + prefix + "/diagnostics")).append("\n");
        sb.append("diagnosticsBaseUrl = ").append(tomlQuote(base + prefix + "/diagnostics/")).append("\n");
        sb.append("diagnosticsHistoryUrl = ").append(tomlQuote(base + prefix + "/diagnostics/history.json")).append("\n");
        sb.append("autoLogUrl = ").append(tomlQuote(base + prefix + "/auto/log")).append("\n");
        sb.append("mechanismLogUrl = ").append(tomlQuote(base + prefix + "/mechanisms/log")).append("\n");
        sb.append("mechanismLogBaseUrl = ").append(tomlQuote(base + prefix + "/mechanisms/log/")).append("\n");
        sb.append("\n");

        for (Map.Entry<String, Mechanism> e : robot.getMechanisms().entrySet()) {
            if (e == null || e.getKey() == null || e.getValue() == null) {
                continue;
            }
            String name = e.getKey();
            String enc = java.net.URLEncoder.encode(name, StandardCharsets.UTF_8);
            sb.append("[[mechanisms]]\n");
            sb.append("name = ").append(tomlQuote(name)).append("\n");
            sb.append("json = ").append(tomlQuote(base + prefix + "/config/mechanisms/" + enc + ".json")).append("\n");
            sb.append("toml = ").append(tomlQuote(base + prefix + "/config/mechanisms/" + enc + ".toml")).append("\n");
            sb.append("log = ").append(tomlQuote(base + prefix + "/mechanisms/log/" + enc + ".json")).append("\n");
            sb.append("\n");
        }

        sendText(ex, 200, sb + "\n", "application/toml; charset=utf-8");
    }

    private void handleMechanisms(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        URI uri = ex.getRequestURI();
        String path = uri != null ? uri.getPath() : "";
        // Expect: /athena/config/mechanisms/{name}.{ext} (or /Athena/config/...)
        String prefixLower = "/athena/config/mechanisms/";
        String prefixUpper = "/Athena/config/mechanisms/";
        String prefix;
        if (path.startsWith(prefixLower)) {
            prefix = prefixLower;
        } else if (path.startsWith(prefixUpper)) {
            prefix = prefixUpper;
        } else {
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

    private static String tomlQuote(String s) {
        if (s == null) {
            return "\"\"";
        }
        StringBuilder out = new StringBuilder(s.length() + 2);
        out.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (c == '\\' || c == '"') {
                out.append('\\').append(c);
            } else if (c == '\n') {
                out.append("\\n");
            } else if (c == '\r') {
                out.append("\\r");
            } else if (c == '\t') {
                out.append("\\t");
            } else {
                out.append(c);
            }
        }
        out.append('"');
        return out.toString();
    }

    private void handleAllZip(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        try (ZipOutputStream zip = new ZipOutputStream(baos, StandardCharsets.UTF_8)) {
            Map<String, CustomConfigEntry> customEntries = snapshotCustomConfigEntries();
            // Include the index for convenience.
            ZipEntry index = new ZipEntry("index.json");
            zip.putNextEntry(index);
            String idx = INDEX_MAPPER.writeValueAsString(Map.of(
                    "baseUrl", baseUrl(),
                    "mechanisms", robot.getMechanisms().keySet(),
                    "custom", customEntries.keySet()));
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

            for (Map.Entry<String, CustomConfigEntry> custom : customEntries.entrySet()) {
                String key = custom.getKey();
                CustomConfigEntry entry = custom.getValue();
                if (key == null || key.isBlank() || entry == null || entry.body() == null) {
                    continue;
                }
                ZipEntry customZip = new ZipEntry("custom/" + key);
                zip.putNextEntry(customZip);
                zip.write(entry.body());
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

    private void handleCustomConfig(HttpExchange ex) throws IOException {
        URI uri = ex.getRequestURI();
        String path = uri != null ? uri.getPath() : "";
        String prefixLower = "/athena/config/custom";
        String prefixUpper = "/Athena/config/custom";
        String prefix;
        if (path.startsWith(prefixLower)) {
            prefix = prefixLower;
        } else if (path.startsWith(prefixUpper)) {
            prefix = prefixUpper;
        } else {
            sendText(ex, 404, "Not Found\n", "text/plain; charset=utf-8");
            return;
        }

        String method = ex.getRequestMethod() != null ? ex.getRequestMethod().toUpperCase() : "";
        String tail = path.substring(prefix.length());
        if (tail.isEmpty() || "/".equals(tail)) {
            if (!"GET".equals(method)) {
                sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
                return;
            }
            sendCustomConfigIndex(ex, uri);
            return;
        }
        if (tail.startsWith("/")) {
            tail = tail.substring(1);
        }

        final String key;
        try {
            key = normalizeCustomKey(URLDecoder.decode(tail, StandardCharsets.UTF_8));
        } catch (IllegalArgumentException err) {
            sendText(ex, 400, err.getMessage() + "\n", "text/plain; charset=utf-8");
            return;
        }

        if ("GET".equals(method)) {
            CustomConfigEntry entry = customConfigEntries.get(key);
            if (entry == null) {
                sendText(ex, 404, "Unknown custom config key '" + key + "'\n", "text/plain; charset=utf-8");
                return;
            }
            sendBytes(ex, 200, entry.body(), entry.contentType());
            return;
        }
        if ("POST".equals(method) || "PUT".equals(method)) {
            byte[] body;
            try {
                body = readRequestBody(ex, MAX_CUSTOM_PAYLOAD_BYTES);
            } catch (IllegalArgumentException tooLarge) {
                sendText(ex, 413, tooLarge.getMessage() + "\n", "text/plain; charset=utf-8");
                return;
            }
            String contentType = ex.getRequestHeaders().getFirst("Content-Type");
            putCustomBytes(key, contentType, body);
            Map<String, Object> payload = new LinkedHashMap<>();
            payload.put("key", key);
            payload.put("sizeBytes", body.length);
            payload.put("contentType", customConfigEntries.get(key).contentType());
            payload.put("url", baseUrl() + athenaPathPrefix(uri) + "/config/custom/" + encodeUrlSegment(key));
            sendJson(ex, 200, payload);
            return;
        }
        if ("DELETE".equals(method)) {
            boolean removed = removeCustom(key);
            Map<String, Object> payload = new LinkedHashMap<>();
            payload.put("key", key);
            payload.put("removed", removed);
            sendJson(ex, 200, payload);
            return;
        }

        sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
    }

    private void sendCustomConfigIndex(HttpExchange ex, URI uri) throws IOException {
        String base = baseUrl();
        String prefix = athenaPathPrefix(uri);
        Map<String, CustomConfigEntry> entries = snapshotCustomConfigEntries();
        List<Map<String, Object>> items = new ArrayList<>(entries.size());
        for (Map.Entry<String, CustomConfigEntry> entry : entries.entrySet()) {
            String key = entry.getKey();
            CustomConfigEntry value = entry.getValue();
            if (key == null || key.isBlank() || value == null || value.body() == null) {
                continue;
            }
            Map<String, Object> item = new LinkedHashMap<>();
            item.put("key", key);
            item.put("contentType", value.contentType());
            item.put("sizeBytes", value.body().length);
            item.put("updatedAtSeconds", value.updatedAtSeconds());
            item.put("url", base + prefix + "/config/custom/" + encodeUrlSegment(key));
            items.add(item);
        }
        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("baseUrl", base);
        payload.put("customConfigUrl", base + prefix + "/config/custom");
        payload.put("customConfigBaseUrl", base + prefix + "/config/custom/");
        payload.put("count", items.size());
        payload.put("items", items);
        sendJson(ex, 200, payload);
    }

    private Map<String, CustomConfigEntry> snapshotCustomConfigEntries() {
        List<Map.Entry<String, CustomConfigEntry>> ordered = new ArrayList<>(customConfigEntries.entrySet());
        ordered.sort(Comparator.comparing(Map.Entry::getKey, String.CASE_INSENSITIVE_ORDER));
        Map<String, CustomConfigEntry> out = new LinkedHashMap<>(ordered.size());
        for (Map.Entry<String, CustomConfigEntry> entry : ordered) {
            if (entry == null || entry.getKey() == null || entry.getValue() == null) {
                continue;
            }
            out.put(entry.getKey(), entry.getValue());
        }
        return out;
    }

    private void handleDiagnostics(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        URI uri = ex.getRequestURI();
        String path = uri != null ? uri.getPath() : "";
        String prefixLower = "/athena/diagnostics";
        String prefixUpper = "/Athena/diagnostics";
        String prefix;
        if (path.startsWith(prefixLower)) {
            prefix = prefixLower;
        } else if (path.startsWith(prefixUpper)) {
            prefix = prefixUpper;
        } else {
            sendText(ex, 404, "Not Found\n", "text/plain; charset=utf-8");
            return;
        }

        String tail = path.substring(prefix.length());
        Map<String, String> query = parseQuery(uri);
        if (tail.isEmpty() || "/".equals(tail)) {
            sendDiagnosticsIndexJson(ex, uri);
            return;
        }

        if (tail.startsWith("/")) {
            tail = tail.substring(1);
        }
        if (tail.isBlank()) {
            sendDiagnosticsIndexJson(ex, uri);
            return;
        }
        if (tail.toLowerCase().endsWith(".json")) {
            tail = tail.substring(0, tail.length() - ".json".length());
        }
        String tailLower = tail.toLowerCase();
        if ("history".equals(tailLower) || "all".equals(tailLower)) {
            sendDiagnosticsHistoryJson(ex, uri, query);
            return;
        }

        final String key;
        try {
            key = normalizeDiagnosticsKey(URLDecoder.decode(tail, StandardCharsets.UTF_8));
        } catch (IllegalArgumentException err) {
            sendText(ex, 400, err.getMessage() + "\n", "text/plain; charset=utf-8");
            return;
        }

        DiagnosticsProviderEntry provider = resolveDiagnosticsProvider(key);
        if (provider == null) {
            sendText(ex, 404, "Unknown diagnostics key '" + key + "'\n", "text/plain; charset=utf-8");
            return;
        }

        int limit = parsePositiveInt(query.get("limit"), 120);
        if (limit > 2048) {
            limit = 2048;
        }
        boolean clear = parseBoolean(query.get("clear"));
        boolean cleared = false;
        if (clear && provider.clearAction() != null) {
            try {
                provider.clearAction().run();
                cleared = true;
            } catch (RuntimeException exClear) {
                DriverStation.reportWarning(
                        "Diagnostics clear action failed for '" + key + "': " + exClear.getMessage(),
                        false);
            }
        }

        String base = baseUrl();
        String athenaPrefix = athenaPathPrefix(uri);
        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("key", key);
        payload.put("url", base + athenaPrefix + "/diagnostics/" + encodeUrlSegment(key) + ".json");
        payload.put("limit", limit);
        payload.put("clearSupported", provider.clearAction() != null);
        payload.put("cleared", cleared);
        payload.put("summary", safeDiagnosticsSummary(key, provider));
        Map<String, Object> diagnostics = safeDiagnosticsSnapshot(key, provider, limit);
        payload.put("diagnostics", diagnostics);
        payload.put("events", normalizeDiagnosticsEvents(key, diagnostics, limit));
        sendJson(ex, 200, payload);
    }

    private void sendDiagnosticsIndexJson(HttpExchange ex, URI uri) throws IOException {
        String base = baseUrl();
        String prefix = athenaPathPrefix(uri);
        Map<String, DiagnosticsProviderEntry> providers = collectDiagnosticsProviders();
        List<Map<String, Object>> entries = new ArrayList<>(providers.size());
        for (Map.Entry<String, DiagnosticsProviderEntry> entry : providers.entrySet()) {
            String key = entry.getKey();
            DiagnosticsProviderEntry provider = entry.getValue();
            if (key == null || key.isBlank() || provider == null) {
                continue;
            }
            Map<String, Object> item = new LinkedHashMap<>();
            item.put("key", key);
            item.put("url", base + prefix + "/diagnostics/" + encodeUrlSegment(key) + ".json");
            item.put("clearSupported", provider.clearAction() != null);
            item.put("summary", safeDiagnosticsSummary(key, provider));
            entries.add(item);
        }
        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("baseUrl", base);
        payload.put("diagnosticsUrl", base + prefix + "/diagnostics");
        payload.put("diagnosticsBaseUrl", base + prefix + "/diagnostics/");
        payload.put("diagnosticsHistoryUrl", base + prefix + "/diagnostics/history.json");
        payload.put("count", entries.size());
        payload.put("items", entries);
        sendJson(ex, 200, payload);
    }

    private void sendDiagnosticsHistoryJson(HttpExchange ex, URI uri, Map<String, String> query) throws IOException {
        int limit = parsePositiveInt(query.get("limit"), MAX_HISTORY_LIMIT);
        if (limit > MAX_HISTORY_LIMIT) {
            limit = MAX_HISTORY_LIMIT;
        }
        Map<String, DiagnosticsProviderEntry> providers = collectDiagnosticsProviders();
        List<Map<String, Object>> merged = new ArrayList<>();
        for (Map.Entry<String, DiagnosticsProviderEntry> providerEntry : providers.entrySet()) {
            if (providerEntry == null || providerEntry.getKey() == null || providerEntry.getValue() == null) {
                continue;
            }
            String key = providerEntry.getKey();
            DiagnosticsProviderEntry provider = providerEntry.getValue();
            Map<String, Object> snapshot = safeDiagnosticsSnapshot(key, provider, limit);
            merged.addAll(normalizeDiagnosticsEvents(key, snapshot, limit));
        }
        merged.sort((a, b) -> {
            double aTs = readTimestampSeconds(a);
            double bTs = readTimestampSeconds(b);
            int tsCmp = Double.compare(aTs, bTs);
            if (tsCmp != 0) {
                return tsCmp;
            }
            long aSeq = readSequence(a);
            long bSeq = readSequence(b);
            int seqCmp = Long.compare(aSeq, bSeq);
            if (seqCmp != 0) {
                return seqCmp;
            }
            String aKey = String.valueOf(a.getOrDefault("systemKey", ""));
            String bKey = String.valueOf(b.getOrDefault("systemKey", ""));
            return aKey.compareToIgnoreCase(bKey);
        });

        boolean truncated = merged.size() > limit;
        List<Map<String, Object>> events = merged;
        if (truncated) {
            events = merged.subList(merged.size() - limit, merged.size());
        }

        String base = baseUrl();
        String prefix = athenaPathPrefix(uri);
        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("baseUrl", base);
        payload.put("historyUrl", base + prefix + "/diagnostics/history.json");
        payload.put("sourceCount", providers.size());
        payload.put("requestedLimit", limit);
        payload.put("returnedCount", events.size());
        payload.put("truncated", truncated);
        payload.put("events", events);
        sendJson(ex, 200, payload);
    }

    private static double readTimestampSeconds(Map<String, Object> event) {
        Object raw = event != null ? event.get("timestampSeconds") : null;
        if (raw instanceof Number number) {
            double value = number.doubleValue();
            return Double.isFinite(value) ? value : Double.POSITIVE_INFINITY;
        }
        if (raw instanceof String text) {
            try {
                double value = Double.parseDouble(text.trim());
                return Double.isFinite(value) ? value : Double.POSITIVE_INFINITY;
            } catch (NumberFormatException ignored) {
                return Double.POSITIVE_INFINITY;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    private static long readSequence(Map<String, Object> event) {
        Object raw = event != null ? event.get("sequence") : null;
        if (raw instanceof Number number) {
            return number.longValue();
        }
        if (raw instanceof String text) {
            try {
                return Long.parseLong(text.trim());
            } catch (NumberFormatException ignored) {
                return Long.MAX_VALUE;
            }
        }
        return Long.MAX_VALUE;
    }

    private List<Map<String, Object>> normalizeDiagnosticsEvents(String key, Map<String, Object> snapshot, int limit) {
        List<Map<String, Object>> normalized = new ArrayList<>();
        if (snapshot == null || snapshot.isEmpty()) {
            return normalized;
        }
        int maxEvents = limit > 0 ? Math.min(limit, MAX_HISTORY_LIMIT) : MAX_HISTORY_LIMIT;
        boolean hasEventsField = snapshot.get("events") instanceof List<?>;
        collectNormalizedEventsFromField(normalized, key, snapshot, "events", maxEvents);
        collectNormalizedEventsFromField(normalized, key, snapshot, "logs", maxEvents);
        if (!hasEventsField) {
            collectNormalizedEventsFromField(normalized, key, snapshot, "entries", maxEvents);
        }
        normalized.sort((a, b) -> {
            double aTs = readTimestampSeconds(a);
            double bTs = readTimestampSeconds(b);
            int tsCmp = Double.compare(aTs, bTs);
            if (tsCmp != 0) {
                return tsCmp;
            }
            return Long.compare(readSequence(a), readSequence(b));
        });
        return normalized;
    }

    private void collectNormalizedEventsFromField(
            List<Map<String, Object>> out,
            String key,
            Map<String, Object> snapshot,
            String fieldName,
            int maxEvents) {
        if (out == null || key == null || snapshot == null || fieldName == null || fieldName.isBlank()) {
            return;
        }
        Object raw = snapshot.get(fieldName);
        if (!(raw instanceof List<?> events)) {
            return;
        }
        int toIndex = Math.min(events.size(), Math.max(0, maxEvents));
        for (int i = 0; i < toIndex; i++) {
            Object entry = events.get(i);
            Map<String, Object> normalized = normalizeDiagnosticsEvent(key, fieldName, entry);
            if (!normalized.isEmpty()) {
                out.add(normalized);
            }
        }
    }

    private Map<String, Object> normalizeDiagnosticsEvent(String key, String sourceField, Object rawEvent) {
        Map<String, Object> eventMap = coerceMap(rawEvent);
        String systemKey = key != null ? key : "";
        String systemName = diagnosticsSystemName(systemKey);
        String level = readStringField(eventMap, "level", "severity");
        if (level.isBlank()) {
            level = "INFO";
        }
        String category = readStringField(eventMap, "category", "type");
        if (category.isBlank()) {
            category = sourceField != null && !sourceField.isBlank() ? sourceField : "general";
        }
        String message = readStringField(eventMap, "message", "payload", "text", "detail");
        if (message.isBlank() && rawEvent != null) {
            message = String.valueOf(rawEvent);
        }
        double timestampSeconds = readDoubleField(eventMap, "timestampSeconds", "timestamp", "timeSeconds", "time");
        long sequence = readLongField(eventMap, "sequence", "seq", "index");

        Map<String, Object> normalized = new LinkedHashMap<>();
        if (Double.isFinite(timestampSeconds)) {
            normalized.put("timestampSeconds", timestampSeconds);
        }
        if (sequence != Long.MIN_VALUE) {
            normalized.put("sequence", sequence);
        }
        normalized.put("systemKey", systemKey);
        normalized.put("system", systemName);
        normalized.put("level", level);
        normalized.put("category", category);
        normalized.put("message", message);
        normalized.put(
                "line",
                "[" + systemName + "] "
                        + level.toLowerCase(java.util.Locale.ROOT)
                        + " " + category + ": " + message);
        if (!eventMap.isEmpty()) {
            normalized.put("raw", eventMap);
        }
        return normalized;
    }

    private static Map<String, Object> coerceMap(Object rawEvent) {
        if (rawEvent == null) {
            return Map.of();
        }
        if (rawEvent instanceof Map<?, ?> map) {
            Map<String, Object> out = new LinkedHashMap<>();
            for (Map.Entry<?, ?> entry : map.entrySet()) {
                if (entry == null || entry.getKey() == null) {
                    continue;
                }
                out.put(String.valueOf(entry.getKey()), entry.getValue());
            }
            return out;
        }
        try {
            return INDEX_MAPPER.convertValue(rawEvent, new TypeReference<Map<String, Object>>() {});
        } catch (IllegalArgumentException ignored) {
            return Map.of("value", rawEvent);
        }
    }

    private static String diagnosticsSystemName(String key) {
        if (key == null || key.isBlank()) {
            return "unknown";
        }
        int slash = key.lastIndexOf('/');
        if (slash >= 0 && slash < key.length() - 1) {
            return key.substring(slash + 1);
        }
        return key;
    }

    private static String readStringField(Map<String, Object> map, String... keys) {
        if (map == null || keys == null) {
            return "";
        }
        for (String key : keys) {
            if (key == null || key.isBlank()) {
                continue;
            }
            Object value = map.get(key);
            if (value == null) {
                continue;
            }
            String text = String.valueOf(value).trim();
            if (!text.isEmpty()) {
                return text;
            }
        }
        return "";
    }

    private static double readDoubleField(Map<String, Object> map, String... keys) {
        if (map == null || keys == null) {
            return Double.NaN;
        }
        for (String key : keys) {
            if (key == null || key.isBlank()) {
                continue;
            }
            Object value = map.get(key);
            if (value == null) {
                continue;
            }
            if (value instanceof Number number) {
                double out = number.doubleValue();
                if (Double.isFinite(out)) {
                    return out;
                }
                continue;
            }
            if (value instanceof String text) {
                try {
                    double out = Double.parseDouble(text.trim());
                    if (Double.isFinite(out)) {
                        return out;
                    }
                } catch (NumberFormatException ignored) {
                    // Try next key
                }
            }
        }
        return Double.NaN;
    }

    private static long readLongField(Map<String, Object> map, String... keys) {
        if (map == null || keys == null) {
            return Long.MIN_VALUE;
        }
        for (String key : keys) {
            if (key == null || key.isBlank()) {
                continue;
            }
            Object value = map.get(key);
            if (value == null) {
                continue;
            }
            if (value instanceof Number number) {
                return number.longValue();
            }
            if (value instanceof String text) {
                try {
                    return Long.parseLong(text.trim());
                } catch (NumberFormatException ignored) {
                    // Try next key
                }
            }
        }
        return Long.MIN_VALUE;
    }

    private Map<String, DiagnosticsProviderEntry> collectDiagnosticsProviders() {
        Map<String, DiagnosticsProviderEntry> out = new LinkedHashMap<>();
        out.put("auto", autoDiagnosticsProvider());

        List<String> mechanismNames = new ArrayList<>(robot.getMechanisms().keySet());
        mechanismNames.sort(String::compareToIgnoreCase);
        for (String mechanismName : mechanismNames) {
            if (mechanismName == null || mechanismName.isBlank()) {
                continue;
            }
            out.put("mechanisms/" + mechanismName, mechanismDiagnosticsProvider(mechanismName));
        }

        List<Map.Entry<String, DiagnosticsProviderEntry>> custom = new ArrayList<>(diagnosticsProviders.entrySet());
        custom.sort(Comparator.comparing(Map.Entry::getKey, String.CASE_INSENSITIVE_ORDER));
        for (Map.Entry<String, DiagnosticsProviderEntry> entry : custom) {
            if (entry == null || entry.getKey() == null || entry.getValue() == null) {
                continue;
            }
            out.putIfAbsent(entry.getKey(), entry.getValue());
        }
        return out;
    }

    private DiagnosticsProviderEntry resolveDiagnosticsProvider(String key) {
        if ("auto".equals(key)) {
            return autoDiagnosticsProvider();
        }
        if (key != null && key.startsWith("mechanisms/")) {
            String mechanismName = key.substring("mechanisms/".length());
            if (mechanismName.isBlank()) {
                return null;
            }
            Mechanism mechanism = robot.getMechanisms().get(mechanismName);
            if (mechanism == null) {
                return null;
            }
            return mechanismDiagnosticsProvider(mechanismName);
        }
        return diagnosticsProviders.get(key);
    }

    private DiagnosticsProviderEntry autoDiagnosticsProvider() {
        return new DiagnosticsProviderEntry(
                () -> {
                    RobotAuto autos = robot.getAutos();
                    Map<String, Object> summary = new LinkedHashMap<>();
                    summary.put("autoTraceEnabled", autos.isAutoTraceEnabled());
                    summary.put("capacity", autos.getAutoTraceLogCapacity());
                    summary.put("totalCount", autos.getAutoTraceLogCount());
                    return summary;
                },
                limit -> {
                    RobotAuto autos = robot.getAutos();
                    int requestedLimit = limit > 0 ? limit : 200;
                    List<RobotAuto.AutoTraceEvent> entries = autos.getAutoTraceLog(requestedLimit);
                    Map<String, Object> diagnostics = new LinkedHashMap<>();
                    diagnostics.put("autoTraceEnabled", autos.isAutoTraceEnabled());
                    diagnostics.put("capacity", autos.getAutoTraceLogCapacity());
                    diagnostics.put("totalCount", autos.getAutoTraceLogCount());
                    diagnostics.put("returnedCount", entries.size());
                    diagnostics.put("entries", entries);
                    diagnostics.put("events", entries);
                    return diagnostics;
                },
                () -> robot.getAutos().clearAutoTraceLog());
    }

    private DiagnosticsProviderEntry mechanismDiagnosticsProvider(String mechanismName) {
        return new DiagnosticsProviderEntry(
                () -> {
                    Mechanism mechanism = robot.getMechanisms().get(mechanismName);
                    if (mechanism == null) {
                        return Map.of("error", "mechanism unavailable");
                    }
                    return mechanism.diagnostics().summary();
                },
                limit -> {
                    Mechanism mechanism = robot.getMechanisms().get(mechanismName);
                    if (mechanism == null) {
                        return Map.of("error", "mechanism unavailable");
                    }
                    int requestedLimit = limit > 0 ? Math.min(limit, 2048) : 120;
                    return mechanism.diagnostics().snapshot(requestedLimit);
                },
                () -> {
                    Mechanism mechanism = robot.getMechanisms().get(mechanismName);
                    if (mechanism != null) {
                        mechanism.diagnostics().clear();
                    }
                });
    }

    private static Map<String, Object> safeDiagnosticsSummary(String key, DiagnosticsProviderEntry provider) {
        try {
            Map<String, Object> summary = provider.summarySupplier().get();
            return summary != null ? summary : Map.of();
        } catch (RuntimeException ex) {
            return Map.of("error", "summary failed for '" + key + "': " + ex.getMessage());
        }
    }

    private static Map<String, Object> safeDiagnosticsSnapshot(String key, DiagnosticsProviderEntry provider, int limit) {
        try {
            Map<String, Object> snapshot = provider.snapshotSupplier().apply(limit);
            return snapshot != null ? snapshot : Map.of();
        } catch (RuntimeException ex) {
            return Map.of("error", "snapshot failed for '" + key + "': " + ex.getMessage());
        }
    }

    private void handleAutoLog(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }

        Map<String, String> query = parseQuery(ex.getRequestURI());
        int requestedLimit = parsePositiveInt(query.get("limit"), 200);
        boolean clear = parseBoolean(query.get("clear"));

        RobotAuto autos = robot.getAutos();
        List<RobotAuto.AutoTraceEvent> entries = autos.getAutoTraceLog(requestedLimit);
        int totalCount = autos.getAutoTraceLogCount();
        if (clear) {
            autos.clearAutoTraceLog();
        }

        Map<String, Object> payload = Map.of(
                "autoTraceEnabled", autos.isAutoTraceEnabled(),
                "capacity", autos.getAutoTraceLogCapacity(),
                "totalCount", totalCount,
                "returnedCount", entries.size(),
                "cleared", clear,
                "entries", entries,
                "events", entries);

        String json;
        try {
            json = INDEX_MAPPER.writeValueAsString(payload);
        } catch (JsonProcessingException err) {
            json = "{\"error\":\"failed to build auto log\"}";
        }
        sendText(ex, 200, json + "\n", "application/json; charset=utf-8");
    }

    private void handleMechanismLogs(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            sendText(ex, 405, "Method Not Allowed\n", "text/plain; charset=utf-8");
            return;
        }
        URI uri = ex.getRequestURI();
        String path = uri != null ? uri.getPath() : "";
        String prefixLower = "/athena/mechanisms/log";
        String prefixUpper = "/Athena/mechanisms/log";
        String prefix;
        if (path.startsWith(prefixLower)) {
            prefix = prefixLower;
        } else if (path.startsWith(prefixUpper)) {
            prefix = prefixUpper;
        } else {
            sendText(ex, 404, "Not Found\n", "text/plain; charset=utf-8");
            return;
        }

        Map<String, String> query = parseQuery(uri);
        String tail = path.substring(prefix.length());
        if (tail.isEmpty() || "/".equals(tail)) {
            if (wantsJsonResponse(ex, query)) {
                sendMechanismLogIndexJson(ex, uri);
            } else {
                sendMechanismLogHtml(ex);
            }
            return;
        }
        if (tail.startsWith("/")) {
            tail = tail.substring(1);
        }
        if (!tail.toLowerCase().endsWith(".json")) {
            sendText(ex, 404, "Not Found\n", "text/plain; charset=utf-8");
            return;
        }
        String rawName = tail.substring(0, tail.length() - ".json".length());
        String name = URLDecoder.decode(rawName, StandardCharsets.UTF_8);
        Mechanism mechanism = robot.getMechanisms().get(name);
        if (mechanism == null) {
            sendText(ex, 404, "Unknown mechanism '" + name + "'\n", "text/plain; charset=utf-8");
            return;
        }

        int limit = parsePositiveInt(query.get("limit"), 120);
        if (limit > 512) {
            limit = 512;
        }
        boolean clear = parseBoolean(query.get("clear"));
        if (clear) {
            mechanism.diagnostics().clear();
        }

        String base = baseUrl();
        String athenaPrefix = athenaPathPrefix(uri);
        String encodedName = java.net.URLEncoder.encode(name, StandardCharsets.UTF_8);
        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("name", name);
        payload.put("logUrl", base + athenaPrefix + "/mechanisms/log/" + encodedName + ".json");
        payload.put("limit", limit);
        payload.put("cleared", clear);
        payload.put("diagnostics", mechanism.diagnostics().snapshot(limit));

        String json;
        try {
            json = INDEX_MAPPER.writeValueAsString(payload);
        } catch (JsonProcessingException err) {
            json = "{\"error\":\"failed to build mechanism diagnostics\"}";
        }
        sendText(ex, 200, json + "\n", "application/json; charset=utf-8");
    }

    private void sendMechanismLogIndexJson(HttpExchange ex, URI uri) throws IOException {
        String base = baseUrl();
        String prefix = athenaPathPrefix(uri);
        List<String> names = new ArrayList<>(robot.getMechanisms().keySet());
        names.sort(String::compareToIgnoreCase);
        List<Object> mechanisms = new ArrayList<>();
        for (String name : names) {
            if (name == null) {
                continue;
            }
            Mechanism mechanism = robot.getMechanisms().get(name);
            if (mechanism == null) {
                continue;
            }
            String encodedName = java.net.URLEncoder.encode(name, StandardCharsets.UTF_8);
            Map<String, Object> entry = new LinkedHashMap<>();
            entry.put("name", name);
            entry.put("logJson", base + prefix + "/mechanisms/log/" + encodedName + ".json");
            entry.put("summary", mechanism.diagnostics().summary());
            mechanisms.add(entry);
        }
        Map<String, Object> payload = new LinkedHashMap<>();
        payload.put("baseUrl", base);
        payload.put("mechanismLogUrl", base + prefix + "/mechanisms/log");
        payload.put("mechanismLogBaseUrl", base + prefix + "/mechanisms/log/");
        payload.put("mechanismCount", mechanisms.size());
        payload.put("mechanisms", mechanisms);
        String json;
        try {
            json = INDEX_MAPPER.writeValueAsString(payload);
        } catch (JsonProcessingException err) {
            json = "{\"error\":\"failed to build mechanism diagnostics index\"}";
        }
        sendText(ex, 200, json + "\n", "application/json; charset=utf-8");
    }

    private void sendMechanismLogHtml(HttpExchange ex) throws IOException {
        byte[] html = readClasspathResource(MECHANISM_LOG_HTML_RESOURCE);
        if (html == null) {
            sendText(ex, 500, "Missing resource: " + MECHANISM_LOG_HTML_RESOURCE + "\n", "text/plain; charset=utf-8");
            return;
        }
        sendBytes(ex, 200, html, "text/html; charset=utf-8");
    }

    private static boolean wantsJsonResponse(HttpExchange ex, Map<String, String> query) {
        if (parseBoolean(query.get("json"))) {
            return true;
        }
        String format = query.get("format");
        if (format != null && "json".equalsIgnoreCase(format.trim())) {
            return true;
        }
        Headers headers = ex.getRequestHeaders();
        if (headers == null) {
            return false;
        }
        String accept = headers.getFirst("Accept");
        return accept != null && accept.toLowerCase().contains("application/json");
    }

    private static Map<String, String> parseQuery(URI uri) {
        if (uri == null || uri.getRawQuery() == null || uri.getRawQuery().isBlank()) {
            return Map.of();
        }
        Map<String, String> out = new java.util.LinkedHashMap<>();
        String rawQuery = uri.getRawQuery();
        String[] pairs = rawQuery.split("&");
        for (String pair : pairs) {
            if (pair == null || pair.isBlank()) {
                continue;
            }
            int eq = pair.indexOf('=');
            if (eq < 0) {
                String key = URLDecoder.decode(pair, StandardCharsets.UTF_8);
                out.put(key, "");
                continue;
            }
            String key = URLDecoder.decode(pair.substring(0, eq), StandardCharsets.UTF_8);
            String value = URLDecoder.decode(pair.substring(eq + 1), StandardCharsets.UTF_8);
            out.put(key, value);
        }
        return out;
    }

    private static int parsePositiveInt(String raw, int fallback) {
        if (raw == null || raw.isBlank()) {
            return fallback;
        }
        try {
            int value = Integer.parseInt(raw.trim());
            return value > 0 ? value : fallback;
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }

    private static boolean parseBoolean(String raw) {
        if (raw == null) {
            return false;
        }
        String normalized = raw.trim().toLowerCase();
        return "1".equals(normalized)
                || "true".equals(normalized)
                || "yes".equals(normalized)
                || "on".equals(normalized);
    }

    private static String normalizeCustomKey(String rawKey) {
        if (rawKey == null) {
            throw new IllegalArgumentException("custom config key must not be null");
        }
        String key = rawKey.trim();
        while (key.startsWith("/")) {
            key = key.substring(1);
        }
        while (key.endsWith("/")) {
            key = key.substring(0, key.length() - 1);
        }
        if (key.isEmpty()) {
            throw new IllegalArgumentException("custom config key must not be blank");
        }
        if (key.contains("..")) {
            throw new IllegalArgumentException("custom config key must not contain '..'");
        }
        if (key.indexOf('\\') >= 0) {
            throw new IllegalArgumentException("custom config key must not contain '\\'");
        }
        String[] segments = key.split("/");
        StringBuilder normalized = new StringBuilder(key.length());
        for (String segment : segments) {
            String trimmed = segment != null ? segment.trim() : "";
            if (trimmed.isEmpty()) {
                throw new IllegalArgumentException("custom config key must not contain empty path segments");
            }
            if (trimmed.indexOf('?') >= 0 || trimmed.indexOf('#') >= 0) {
                throw new IllegalArgumentException("custom config key must not contain '?' or '#'");
            }
            if (normalized.length() > 0) {
                normalized.append('/');
            }
            normalized.append(trimmed);
        }
        return normalized.toString();
    }

    private static String normalizeDiagnosticsKey(String rawKey) {
        if (rawKey == null) {
            throw new IllegalArgumentException("diagnostics key must not be null");
        }
        String key = rawKey.trim();
        while (key.startsWith("/")) {
            key = key.substring(1);
        }
        while (key.endsWith("/")) {
            key = key.substring(0, key.length() - 1);
        }
        if (key.isEmpty()) {
            throw new IllegalArgumentException("diagnostics key must not be blank");
        }
        if (key.contains("..")) {
            throw new IllegalArgumentException("diagnostics key must not contain '..'");
        }
        if (key.indexOf('\\') >= 0) {
            throw new IllegalArgumentException("diagnostics key must not contain '\\'");
        }
        String[] segments = key.split("/");
        StringBuilder normalized = new StringBuilder(key.length());
        for (String segment : segments) {
            String trimmed = segment != null ? segment.trim() : "";
            if (trimmed.isEmpty()) {
                throw new IllegalArgumentException("diagnostics key must not contain empty path segments");
            }
            if (trimmed.indexOf('?') >= 0 || trimmed.indexOf('#') >= 0) {
                throw new IllegalArgumentException("diagnostics key must not contain '?' or '#'");
            }
            if (normalized.length() > 0) {
                normalized.append('/');
            }
            normalized.append(trimmed);
        }
        return normalized.toString();
    }

    private static String encodeUrlSegment(String value) {
        if (value == null || value.isEmpty()) {
            return "";
        }
        return java.net.URLEncoder.encode(value, StandardCharsets.UTF_8);
    }

    private static byte[] readRequestBody(HttpExchange ex, int maxBytes) throws IOException {
        if (maxBytes <= 0) {
            throw new IllegalArgumentException("maxBytes must be > 0");
        }
        try (InputStream is = ex.getRequestBody(); ByteArrayOutputStream out = new ByteArrayOutputStream()) {
            byte[] buffer = new byte[4096];
            int total = 0;
            int read;
            while ((read = is.read(buffer)) >= 0) {
                total += read;
                if (total > maxBytes) {
                    throw new IllegalArgumentException("request payload exceeds " + maxBytes + " bytes");
                }
                out.write(buffer, 0, read);
            }
            return out.toByteArray();
        }
    }

    private static byte[] readClasspathResource(String resourcePath) throws IOException {
        try (InputStream is = AthenaRuntimeServer.class.getResourceAsStream(resourcePath)) {
            return is != null ? is.readAllBytes() : null;
        }
    }

    private static void sendJson(HttpExchange ex, int status, Object payload) throws IOException {
        String json;
        try {
            json = INDEX_MAPPER.writeValueAsString(payload);
        } catch (JsonProcessingException err) {
            json = "{\"error\":\"failed to serialize json payload\"}";
            status = 500;
        }
        sendText(ex, status, json + "\n", "application/json; charset=utf-8");
    }

    private static void sendBytes(HttpExchange ex, int status, byte[] body, String contentType) throws IOException {
        byte[] bytes = body != null ? body : new byte[0];
        Headers headers = ex.getResponseHeaders();
        headers.add("Content-Type",
                contentType != null && !contentType.isBlank()
                        ? contentType
                        : "application/octet-stream");
        ex.sendResponseHeaders(status, bytes.length);
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
