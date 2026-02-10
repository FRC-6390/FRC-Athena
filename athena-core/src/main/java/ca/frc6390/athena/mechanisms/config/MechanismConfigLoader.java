package ca.frc6390.athena.mechanisms.config;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import org.tomlj.Toml;
import org.tomlj.TomlArray;
import org.tomlj.TomlParseResult;
import org.tomlj.TomlTable;

/**
 * Loads JSON/TOML mechanism configs from deploy files.
 *
 * <p>Uses snake_case on disk and maps to camelCase Java record components.
 *
 * <p>Overlay behavior:
 * - Objects are deep-merged
 * - Arrays are replaced by default
 * - Arrays of tables that contain a {@code name} field are merged by name (overlay wins)
 */
public final class MechanismConfigLoader {
    private static final ObjectMapper MAPPER = buildMapper();

    private MechanismConfigLoader() {
    }

    public static MechanismConfigFile load(Path path) {
        Objects.requireNonNull(path, "path");
        JsonNode node = loadTree(path);
        try {
            return MAPPER.treeToValue(node, MechanismConfigFile.class);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to bind mechanism config: " + path, e);
        }
    }

    public static MechanismConfigFile loadMerged(Path base, Path... overlays) {
        Objects.requireNonNull(base, "base");
        JsonNode merged = loadTree(base);
        if (overlays != null) {
            for (Path overlay : overlays) {
                if (overlay == null) {
                    continue;
                }
                merged = merge(merged, loadTree(overlay));
            }
        }
        try {
            return MAPPER.treeToValue(merged, MechanismConfigFile.class);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to bind merged mechanism config", e);
        }
    }

    public static JsonNode loadTree(Path path) {
        String ext = extension(path);
        return switch (ext) {
            case "toml" -> loadTomlTree(path);
            case "json" -> loadJsonTree(path);
            default -> throw new IllegalArgumentException(
                    "Unsupported mechanism config extension '." + ext + "' for " + path);
        };
    }

    private static JsonNode loadJsonTree(Path path) {
        try {
            JsonFactory factory = MAPPER.getFactory();
            try (JsonParser parser = factory.createParser(path.toFile())) {
                return MAPPER.readTree(parser);
            }
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to load JSON mechanism config from " + path, e);
        }
    }

    private static JsonNode loadTomlTree(Path path) {
        try {
            String text = Files.readString(path);
            TomlParseResult result = Toml.parse(text);
            if (result.hasErrors()) {
                throw new IllegalArgumentException("TOML parse errors in " + path + ": " + result.errors());
            }
            ObjectNode root = MAPPER.createObjectNode();
            writeTomlTable(root, result);
            return root;
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to load TOML mechanism config from " + path, e);
        }
    }

    private static void writeTomlTable(ObjectNode target, TomlTable table) {
        for (String key : table.keySet()) {
            Object value = table.get(key);
            target.set(key, toJson(value));
        }
    }

    private static JsonNode toJson(Object value) {
        if (value == null) {
            return MAPPER.nullNode();
        }
        if (value instanceof TomlTable tt) {
            ObjectNode obj = MAPPER.createObjectNode();
            writeTomlTable(obj, tt);
            return obj;
        }
        if (value instanceof TomlArray ta) {
            ArrayNode arr = MAPPER.createArrayNode();
            for (int i = 0; i < ta.size(); i++) {
                arr.add(toJson(ta.get(i)));
            }
            return arr;
        }
        // tomlj scalars: String, Long, Double, Boolean, LocalDate, LocalTime, OffsetDateTime
        return MAPPER.valueToTree(value);
    }

    private static JsonNode merge(JsonNode base, JsonNode overlay) {
        if (overlay == null || overlay.isNull()) {
            return base;
        }
        if (base == null || base.isNull()) {
            return overlay;
        }
        if (base.isObject() && overlay.isObject()) {
            ObjectNode out = ((ObjectNode) base).deepCopy();
            Iterator<Map.Entry<String, JsonNode>> fields = overlay.fields();
            while (fields.hasNext()) {
                Map.Entry<String, JsonNode> e = fields.next();
                String key = e.getKey();
                JsonNode baseChild = out.get(key);
                out.set(key, merge(baseChild, e.getValue()));
            }
            return out;
        }
        if (base.isArray() && overlay.isArray()) {
            // If it looks like an array of named tables, merge by "name".
            if (isNamedTableArray(base) && isNamedTableArray(overlay)) {
                return mergeNamedArrays((ArrayNode) base, (ArrayNode) overlay);
            }
            // Otherwise: overlay replaces.
            return overlay;
        }
        // Scalars: overlay replaces.
        return overlay;
    }

    private static boolean isNamedTableArray(JsonNode node) {
        if (node == null || !node.isArray()) {
            return false;
        }
        for (JsonNode child : node) {
            if (!child.isObject()) {
                return false;
            }
            JsonNode name = child.get("name");
            if (name == null || !name.isTextual() || name.asText().isBlank()) {
                return false;
            }
        }
        return node.size() > 0;
    }

    private static ArrayNode mergeNamedArrays(ArrayNode base, ArrayNode overlay) {
        Map<String, ObjectNode> byName = new LinkedHashMap<>();
        for (JsonNode child : base) {
            ObjectNode obj = (ObjectNode) child;
            byName.put(obj.get("name").asText(), obj.deepCopy());
        }
        for (JsonNode child : overlay) {
            ObjectNode obj = (ObjectNode) child;
            String name = obj.get("name").asText();
            ObjectNode existing = byName.get(name);
            byName.put(name, (ObjectNode) merge(existing, obj));
        }
        ArrayNode out = MAPPER.createArrayNode();
        byName.values().forEach(out::add);
        return out;
    }

    private static String extension(Path path) {
        String name = path.getFileName().toString();
        int idx = name.lastIndexOf('.');
        if (idx < 0 || idx == name.length() - 1) {
            return "";
        }
        return name.substring(idx + 1).toLowerCase(Locale.ROOT);
    }

    private static ObjectMapper buildMapper() {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        mapper.enable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
        mapper.enable(DeserializationFeature.FAIL_ON_NULL_FOR_PRIMITIVES);
        mapper.enable(JsonParser.Feature.ALLOW_COMMENTS);
        return mapper;
    }
}

