package ca.frc6390.athena.core.arcp;

import ca.frc6390.athena.arcp.ArcpRuntime;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalLong;
import java.util.TreeMap;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.LongConsumer;
import java.util.zip.CRC32;

/**
 * Native ARCP signal registry/publisher used by Athena runtime code.
 *
 * <p>This surface is intentionally small for callers:
 * <ul>
 *     <li>{@link #put(String, boolean)} and value overloads for publish</li>
 *     <li>{@link #writable(String, Class)} and {@link #command(String)} for control writes/invokes</li>
 *     <li>{@link #getDouble(String)} and {@link #get(String, Class)} for local cached reads</li>
 *     <li>{@link #layout(String)} and {@link Widgets} for layout/widget metadata</li>
 * </ul>
 *
 * <p>All reads are local-cache only and never block on network I/O.</p>
 */
public final class ARCP {
    private static final int MAX_SIGNAL_ID = 0xFFFF;
    private static final int EVENT_BUFFER_BYTES = 64 * 1024;
    private static final int ARCP_PROTOCOL_VERSION = 1;
    private static final int EVENT_TUNABLE_SET = 1;
    private static final int EVENT_ACTION = 2;

    private static final String META_PATCH_SUFFIX = "$meta/patch";
    private static final String META_VERSION_SUFFIX = "$meta/version";
    private static final String META_HASH_SUFFIX = "$meta/hash";
    private static final String META_FULL_SUFFIX = "$meta/full";

    private static final ObjectMapper JSON = new ObjectMapper();

    @FunctionalInterface
    public interface BooleanSetter {
        void accept(boolean value);
    }

    private final ArcpRuntime runtime;
    private final Map<SignalKey, SignalRegistration> signalIds;
    private final Map<SignalKey, SignalRegistration> canonicalSignalIds;
    private final Map<Integer, SignalRegistration> registrationsById;

    private final Map<Integer, BooleanSetter> boolSetHandlers;
    private final Map<Integer, LongConsumer> i64SetHandlers;
    private final Map<Integer, DoubleConsumer> f64SetHandlers;
    private final Map<Integer, Consumer<String>> stringSetHandlers;
    private final Map<Integer, Runnable> actionHandlers;

    private final Map<String, Object> valueCache;
    private final Map<String, ArcpRuntime.SignalType> valueTypes;

    private final Map<String, LinkedHashMap<String, Object>> metadataByPath;
    private final Map<String, Long> metadataVersionByPath;

    private final ByteBuffer eventBuffer;
    private int nextSignalId;

    public ARCP(ArcpRuntime runtime) {
        this.runtime = Objects.requireNonNull(runtime, "runtime");
        this.signalIds = new HashMap<>();
        this.canonicalSignalIds = new HashMap<>();
        this.registrationsById = new HashMap<>();

        this.boolSetHandlers = new HashMap<>();
        this.i64SetHandlers = new HashMap<>();
        this.f64SetHandlers = new HashMap<>();
        this.stringSetHandlers = new HashMap<>();
        this.actionHandlers = new HashMap<>();

        this.valueCache = new HashMap<>();
        this.valueTypes = new HashMap<>();

        this.metadataByPath = new HashMap<>();
        this.metadataVersionByPath = new HashMap<>();

        this.eventBuffer = ByteBuffer.allocateDirect(EVENT_BUFFER_BYTES).order(ByteOrder.LITTLE_ENDIAN);
        this.nextSignalId = 1;
    }

    public PutRef put(String path, boolean value) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.BOOL);
        runtime.publishBoolean(signalId, value);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.BOOL, value);
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.BOOL);
    }

    public PutRef put(String path, double value) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.F64);
        runtime.publishDouble(signalId, value);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.F64, value);
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.F64);
    }

    public PutRef put(String path, long value) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.I64);
        runtime.publishI64(signalId, value);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.I64, value);
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.I64);
    }

    public PutRef put(String path, String value) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.STRING);
        String normalizedValue = value != null ? value : "";
        runtime.publishString(signalId, normalizedValue);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.STRING, normalizedValue);
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.STRING);
    }

    public PutRef put(String path, boolean[] values) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.BOOL_ARRAY);
        boolean[] normalizedValues = values != null ? values : new boolean[0];
        runtime.publishBooleanArray(signalId, normalizedValues);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.BOOL_ARRAY, normalizedValues.clone());
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.BOOL_ARRAY);
    }

    public PutRef put(String path, long[] values) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.I64_ARRAY);
        long[] normalizedValues = values != null ? values : new long[0];
        runtime.publishI64Array(signalId, normalizedValues);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.I64_ARRAY, normalizedValues.clone());
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.I64_ARRAY);
    }

    public PutRef put(String path, double[] values) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.F64_ARRAY);
        double[] normalizedValues = values != null ? values : new double[0];
        runtime.publishF64Array(signalId, normalizedValues);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.F64_ARRAY, normalizedValues.clone());
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.F64_ARRAY);
    }

    public PutRef put(String path, String[] values) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(normalizedPath, ArcpRuntime.SignalType.STRING_ARRAY);
        String[] normalizedValues = values != null ? values : new String[0];
        runtime.publishStringArray(signalId, normalizedValues);
        cacheValue(normalizedPath, ArcpRuntime.SignalType.STRING_ARRAY, normalizedValues.clone());
        return new PutRef(normalizedPath, signalId, ArcpRuntime.SignalType.STRING_ARRAY);
    }

    public PutRef put(String path, Object value) {
        Objects.requireNonNull(value, "value");
        if (value instanceof Boolean boolValue) {
            return put(path, boolValue.booleanValue());
        }
        if (value instanceof Double doubleValue) {
            return put(path, doubleValue.doubleValue());
        }
        if (value instanceof Float floatValue) {
            return put(path, floatValue.doubleValue());
        }
        if (value instanceof Long longValue) {
            return put(path, longValue.longValue());
        }
        if (value instanceof Integer intValue) {
            return put(path, intValue.longValue());
        }
        if (value instanceof Short shortValue) {
            return put(path, shortValue.longValue());
        }
        if (value instanceof Byte byteValue) {
            return put(path, byteValue.longValue());
        }
        if (value instanceof String stringValue) {
            return put(path, stringValue);
        }
        if (value instanceof boolean[] boolArray) {
            return put(path, boolArray);
        }
        if (value instanceof long[] longArray) {
            return put(path, longArray);
        }
        if (value instanceof double[] doubleArray) {
            return put(path, doubleArray);
        }
        if (value instanceof String[] stringArray) {
            return put(path, stringArray);
        }
        throw new IllegalArgumentException("Unsupported ARCP put value type: " + value.getClass().getName());
    }

    public <T> WritableRef<T> writable(String path, Class<T> valueClass) {
        Class<T> boxed = boxedClass(Objects.requireNonNull(valueClass, "valueClass"));
        ArcpRuntime.SignalType signalType = signalTypeForClass(boxed);
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(
                normalizedPath,
                signalType,
                ArcpRuntime.SignalKind.TELEMETRY,
                ArcpRuntime.SignalAccess.WRITE,
                ArcpRuntime.SignalPolicy.ON_CHANGE,
                ArcpRuntime.SignalDurability.VOLATILE);
        return new WritableRef<>(normalizedPath, signalId, signalType, boxed);
    }

    public WritableRef<Boolean> writableBoolean(String path) {
        return writable(path, Boolean.class);
    }

    public WritableRef<Long> writableI64(String path) {
        return writable(path, Long.class);
    }

    public WritableRef<Double> writableDouble(String path) {
        return writable(path, Double.class);
    }

    public WritableRef<String> writableString(String path) {
        return writable(path, String.class);
    }

    public CommandRef command(String path) {
        String normalizedPath = normalizePath(path);
        int signalId = ensureSignal(
                normalizedPath,
                ArcpRuntime.SignalType.BOOL,
                ArcpRuntime.SignalKind.COMMAND,
                ArcpRuntime.SignalAccess.INVOKE,
                ArcpRuntime.SignalPolicy.SNAPSHOT_ONLY,
                ArcpRuntime.SignalDurability.VOLATILE);
        return new CommandRef(normalizedPath, signalId);
    }

    public OptionalDouble getDouble(String path) {
        Optional<Double> value = get(path, Double.class);
        if (value.isPresent()) {
            return OptionalDouble.of(value.get());
        }
        return OptionalDouble.empty();
    }

    public OptionalLong getI64(String path) {
        Optional<Long> value = get(path, Long.class);
        if (value.isPresent()) {
            return OptionalLong.of(value.get());
        }
        return OptionalLong.empty();
    }

    public Optional<Boolean> getBoolean(String path) {
        return get(path, Boolean.class);
    }

    public Optional<String> getString(String path) {
        return get(path, String.class);
    }

    public <T> Optional<T> get(String path, Class<T> valueClass) {
        Objects.requireNonNull(valueClass, "valueClass");
        String normalizedPath = normalizePath(path);
        Class<T> boxed = boxedClass(valueClass);
        synchronized (signalIds) {
            Object value = valueCache.get(normalizedPath);
            if (value == null) {
                return Optional.empty();
            }
            Object coerced = coerceValue(value, boxed);
            if (coerced == null) {
                return Optional.empty();
            }
            return Optional.of(boxed.cast(coerced));
        }
    }

    public MetaRef meta(String path) {
        return new MetaRef(normalizePath(path));
    }

    public TopicDouble topicDouble(String path) {
        return new TopicDouble(path);
    }

    public TopicBoolean topicBoolean(String path) {
        return new TopicBoolean(path);
    }

    public TopicI64 topicI64(String path) {
        return new TopicI64(path);
    }

    public TopicString topicString(String path) {
        return new TopicString(path);
    }

    public LayoutRef layout(String profileName) {
        return new LayoutRef(profileName);
    }

    public int signalId(String path, ArcpRuntime.SignalType signalType) {
        Objects.requireNonNull(signalType, "signalType");
        return ensureSignal(path, signalType);
    }

    public int signalId(String path) {
        String normalizedPath = normalizePath(path);
        int existing = existingSignalIdAnyType(normalizedPath);
        if (existing != 0) {
            return existing;
        }
        return ensureSignal(normalizedPath, ArcpRuntime.SignalType.F64);
    }

    public int existingSignalId(String path, ArcpRuntime.SignalType signalType) {
        Objects.requireNonNull(signalType, "signalType");
        return existingSignalIdInternal(path, signalType);
    }

    public int existingSignalId(String path) {
        return existingSignalIdAnyType(normalizePath(path));
    }

    public void dispatchPendingEvents() {
        synchronized (signalIds) {
            eventBuffer.clear();
            int bytesRead = runtime.pollEvents(eventBuffer);
            if (bytesRead <= 0) {
                return;
            }
            eventBuffer.position(0);
            eventBuffer.limit(Math.min(bytesRead, eventBuffer.capacity()));

            while (eventBuffer.remaining() >= 2) {
                int recordLen = Short.toUnsignedInt(eventBuffer.getShort());
                if (recordLen <= 0 || recordLen > eventBuffer.remaining()) {
                    break;
                }
                ByteBuffer record = eventBuffer.slice().order(ByteOrder.LITTLE_ENDIAN);
                record.limit(recordLen);
                eventBuffer.position(eventBuffer.position() + recordLen);
                decodeEventRecord(record);
            }
        }
    }

    public class PutRef {
        private final String path;
        private final int signalId;
        private final ArcpRuntime.SignalType signalType;

        private PutRef(String path, int signalId, ArcpRuntime.SignalType signalType) {
            this.path = path;
            this.signalId = signalId;
            this.signalType = signalType;
        }

        public String path() {
            return path;
        }

        public int signalId() {
            return signalId;
        }

        public ArcpRuntime.SignalType signalType() {
            return signalType;
        }

        public PutRef widget(WidgetSpec widgetSpec) {
            if (widgetSpec == null) {
                return this;
            }
            LinkedHashMap<String, Object> patch = new LinkedHashMap<>();
            patch.put("ui.widget", widgetSpec.kind());
            if (!widgetSpec.bindings().isEmpty()) {
                patch.put("ui.bindings", new LinkedHashMap<>(widgetSpec.bindings()));
            }
            if (!widgetSpec.config().isEmpty()) {
                patch.put("ui.config", new LinkedHashMap<>(widgetSpec.config()));
            }
            patchMetadata(path, patch);
            return this;
        }

        public PutRef meta(String key, Object value) {
            if (key == null || key.isBlank()) {
                return this;
            }
            LinkedHashMap<String, Object> patch = new LinkedHashMap<>();
            patch.put(key, value);
            patchMetadata(path, patch);
            return this;
        }

        public PutRef meta(Map<String, ?> metadataPatch) {
            patchMetadata(path, metadataPatch);
            return this;
        }
    }

    public final class WritableRef<T> extends PutRef {
        private final Class<T> valueClass;

        private WritableRef(String path, int signalId, ArcpRuntime.SignalType signalType, Class<T> valueClass) {
            super(path, signalId, signalType);
            this.valueClass = valueClass;
        }

        public WritableRef<T> onSet(Consumer<T> handler) {
            if (handler == null) {
                return this;
            }
            int signalId = signalId();
            ArcpRuntime.SignalType signalType = signalType();
            synchronized (signalIds) {
                switch (signalType) {
                    case BOOL -> boolSetHandlers.put(signalId, value -> {
                        cacheValueForSignalId(signalId, ArcpRuntime.SignalType.BOOL, value);
                        handler.accept(valueClass.cast(value));
                    });
                    case I64 -> i64SetHandlers.put(signalId, value -> {
                        cacheValueForSignalId(signalId, ArcpRuntime.SignalType.I64, value);
                        handler.accept(valueClass.cast(value));
                    });
                    case F64 -> f64SetHandlers.put(signalId, value -> {
                        cacheValueForSignalId(signalId, ArcpRuntime.SignalType.F64, value);
                        handler.accept(valueClass.cast(value));
                    });
                    case STRING -> stringSetHandlers.put(signalId, value -> {
                        cacheValueForSignalId(signalId, ArcpRuntime.SignalType.STRING, value);
                        handler.accept(valueClass.cast(value));
                    });
                    default -> throw new IllegalStateException("Unsupported writable signal type: " + signalType);
                }
            }
            return this;
        }

        public WritableRef<T> onSetDouble(DoubleConsumer handler) {
            if (handler == null) {
                return this;
            }
            if (signalType() != ArcpRuntime.SignalType.F64) {
                throw new IllegalStateException("onSetDouble only valid for F64 writable signals");
            }
            int signalId = signalId();
            synchronized (signalIds) {
                f64SetHandlers.put(signalId, value -> {
                    cacheValueForSignalId(signalId, ArcpRuntime.SignalType.F64, value);
                    handler.accept(value);
                });
            }
            return this;
        }

        public WritableRef<T> onSetI64(LongConsumer handler) {
            if (handler == null) {
                return this;
            }
            if (signalType() != ArcpRuntime.SignalType.I64) {
                throw new IllegalStateException("onSetI64 only valid for I64 writable signals");
            }
            int signalId = signalId();
            synchronized (signalIds) {
                i64SetHandlers.put(signalId, value -> {
                    cacheValueForSignalId(signalId, ArcpRuntime.SignalType.I64, value);
                    handler.accept(value);
                });
            }
            return this;
        }

        public WritableRef<T> onSetBoolean(BooleanSetter handler) {
            if (handler == null) {
                return this;
            }
            if (signalType() != ArcpRuntime.SignalType.BOOL) {
                throw new IllegalStateException("onSetBoolean only valid for BOOL writable signals");
            }
            int signalId = signalId();
            synchronized (signalIds) {
                boolSetHandlers.put(signalId, value -> {
                    cacheValueForSignalId(signalId, ArcpRuntime.SignalType.BOOL, value);
                    handler.accept(value);
                });
            }
            return this;
        }
    }

    public final class CommandRef extends PutRef {
        private CommandRef(String path, int signalId) {
            super(path, signalId, ArcpRuntime.SignalType.BOOL);
        }

        public CommandRef onInvoke(Runnable handler) {
            if (handler == null) {
                return this;
            }
            synchronized (signalIds) {
                actionHandlers.put(signalId(), handler);
            }
            return this;
        }

        public CommandRef invoke() {
            runtime.publishBoolean(signalId(), true);
            cacheValue(path(), ArcpRuntime.SignalType.BOOL, true);
            return this;
        }
    }

    public final class MetaRef {
        private final String path;

        private MetaRef(String path) {
            this.path = path;
        }

        public MetaRef patch(Map<String, ?> metadataPatch) {
            patchMetadata(path, metadataPatch);
            return this;
        }

        public MetaRef set(String key, Object value) {
            if (key == null || key.isBlank()) {
                return this;
            }
            LinkedHashMap<String, Object> patch = new LinkedHashMap<>();
            patch.put(key, value);
            patchMetadata(path, patch);
            return this;
        }

        public Map<String, Object> snapshot() {
            synchronized (signalIds) {
                LinkedHashMap<String, Object> current = metadataByPath.get(path);
                if (current == null || current.isEmpty()) {
                    return Map.of();
                }
                return Map.copyOf(current);
            }
        }
    }

    public final class TopicDouble {
        private final String path;
        private int signalId;

        private TopicDouble(String path) {
            this.path = normalizePath(path);
        }

        public TopicDouble bind() {
            signalId = ensureSignal(path, ArcpRuntime.SignalType.F64);
            return this;
        }

        public void set(double value) {
            int id = signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.F64);
            runtime.publishDouble(id, value);
            cacheValue(path, ArcpRuntime.SignalType.F64, value);
        }

        public OptionalDouble get() {
            return getDouble(path);
        }

        public int signalId() {
            return signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.F64);
        }
    }

    public final class TopicBoolean {
        private final String path;
        private int signalId;

        private TopicBoolean(String path) {
            this.path = normalizePath(path);
        }

        public TopicBoolean bind() {
            signalId = ensureSignal(path, ArcpRuntime.SignalType.BOOL);
            return this;
        }

        public void set(boolean value) {
            int id = signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.BOOL);
            runtime.publishBoolean(id, value);
            cacheValue(path, ArcpRuntime.SignalType.BOOL, value);
        }

        public Optional<Boolean> get() {
            return getBoolean(path);
        }

        public int signalId() {
            return signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.BOOL);
        }
    }

    public final class TopicI64 {
        private final String path;
        private int signalId;

        private TopicI64(String path) {
            this.path = normalizePath(path);
        }

        public TopicI64 bind() {
            signalId = ensureSignal(path, ArcpRuntime.SignalType.I64);
            return this;
        }

        public void set(long value) {
            int id = signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.I64);
            runtime.publishI64(id, value);
            cacheValue(path, ArcpRuntime.SignalType.I64, value);
        }

        public OptionalLong get() {
            return getI64(path);
        }

        public int signalId() {
            return signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.I64);
        }
    }

    public final class TopicString {
        private final String path;
        private int signalId;

        private TopicString(String path) {
            this.path = normalizePath(path);
        }

        public TopicString bind() {
            signalId = ensureSignal(path, ArcpRuntime.SignalType.STRING);
            return this;
        }

        public void set(String value) {
            int id = signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.STRING);
            String normalizedValue = value != null ? value : "";
            runtime.publishString(id, normalizedValue);
            cacheValue(path, ArcpRuntime.SignalType.STRING, normalizedValue);
        }

        public Optional<String> get() {
            return getString(path);
        }

        public int signalId() {
            return signalId != 0 ? signalId : ensureSignal(path, ArcpRuntime.SignalType.STRING);
        }
    }

    public static final class Widgets {
        private Widgets() {
        }

        public static WidgetSpec toggle() {
            return custom("toggle");
        }

        public static NumberInputWidgetSpec numberInput() {
            return new NumberInputWidgetSpec("number_input");
        }

        public static WidgetSpec button(String label) {
            return custom("button").config("label", label != null ? label : "Invoke");
        }

        public static WidgetSpec motor() {
            return custom("motor").layout(0, 0, 3, 5);
        }

        public static WidgetSpec encoder() {
            return custom("encoder").layout(0, 0, 3, 7);
        }

        public static WidgetSpec imu() {
            return custom("imu").layout(0, 0, 3, 5);
        }

        public static WidgetSpec dio() {
            return custom("dio").layout(0, 0, 3, 3);
        }

        public static WidgetSpec custom(String kind) {
            return new WidgetSpec(kind);
        }
    }

    public static class WidgetSpec {
        private final String kind;
        private String id;
        private String title;
        private String topicPath;
        private String parentLayoutId;
        private int signalId;
        private ArcpDashboardLayout.LayoutRect layout;
        private boolean layoutExplicit;
        private final Map<String, String> bindings;
        private final Map<String, Object> config;

        protected WidgetSpec(String kind) {
            this.kind = sanitizeWidgetKind(kind);
            this.signalId = 0;
            this.layout = new ArcpDashboardLayout.LayoutRect(0, 0, 2, 1);
            this.layoutExplicit = false;
            this.bindings = new LinkedHashMap<>();
            this.config = new LinkedHashMap<>();
        }

        public String kind() {
            return kind;
        }

        public String id() {
            return id;
        }

        public String title() {
            return title;
        }

        public String topicPath() {
            return topicPath;
        }

        public String parentLayoutId() {
            return parentLayoutId;
        }

        public int signalId() {
            return signalId;
        }

        public ArcpDashboardLayout.LayoutRect layout() {
            return layout;
        }

        public boolean layoutExplicit() {
            return layoutExplicit;
        }

        public Map<String, String> bindings() {
            return bindings;
        }

        public Map<String, Object> config() {
            return config;
        }

        public WidgetSpec id(String id) {
            if (id != null && !id.isBlank()) {
                this.id = id.trim();
            }
            return this;
        }

        public WidgetSpec title(String title) {
            if (title != null && !title.isBlank()) {
                this.title = title.trim();
            }
            return this;
        }

        public WidgetSpec topic(String topicPath) {
            String normalized = normalizeTopicPath(topicPath);
            if (!normalized.isEmpty()) {
                this.topicPath = normalized;
            }
            return this;
        }

        public WidgetSpec signalId(int signalId) {
            this.signalId = Math.max(0, signalId);
            return this;
        }

        public WidgetSpec parentLayoutId(String parentLayoutId) {
            if (parentLayoutId != null && !parentLayoutId.isBlank()) {
                this.parentLayoutId = parentLayoutId.trim();
            } else {
                this.parentLayoutId = null;
            }
            return this;
        }

        public WidgetSpec layout(int x, int y, int w, int h) {
            this.layout = new ArcpDashboardLayout.LayoutRect(x, y, w, h);
            this.layoutExplicit = true;
            return this;
        }

        public WidgetSpec bind(String key, String signalPath) {
            if (key == null || key.isBlank()) {
                return this;
            }
            String normalizedPath = normalizeTopicPath(signalPath);
            if (normalizedPath.isEmpty()) {
                return this;
            }
            bindings.put(key.trim(), normalizedPath);
            return this;
        }

        public WidgetSpec config(String key, Object value) {
            if (key == null || key.isBlank()) {
                return this;
            }
            config.put(key.trim(), value);
            return this;
        }
    }

    public static final class NumberInputWidgetSpec extends WidgetSpec {
        private NumberInputWidgetSpec(String kind) {
            super(kind);
        }

        public NumberInputWidgetSpec min(double min) {
            config("min", min);
            return this;
        }

        public NumberInputWidgetSpec max(double max) {
            config("max", max);
            return this;
        }

        public NumberInputWidgetSpec step(double step) {
            config("step", step);
            return this;
        }
    }

    public final class LayoutRef {
        private final String profileName;
        private final List<PageRef> pages;

        private LayoutRef(String profileName) {
            this.profileName = normalizeLayoutProfileName(profileName);
            this.pages = new ArrayList<>();
        }

        public PageRef page(String pageName) {
            PageRef page = new PageRef(this, pageName);
            pages.add(page);
            return page;
        }

        public LayoutRef publish() {
            if (pages.isEmpty()) {
                page("Dashboard");
            }
            ArcpDashboardLayout.Builder builder = ArcpDashboardLayout.builder()
                    .activeTabId(pages.get(0).pageId());
            for (PageRef page : pages) {
                builder.page(page.build());
            }
            runtime.saveLayout(profileName, builder.build().toJson());
            return this;
        }
    }

    public final class PageRef {
        private final LayoutRef owner;
        private final String pageName;
        private final String pageId;
        private final List<ArcpDashboardLayout.Widget> widgets;
        private int autoCursorX;
        private int autoCursorY;
        private int autoColumns;

        private PageRef(LayoutRef owner, String pageName) {
            this.owner = owner;
            String resolved = pageName != null && !pageName.isBlank() ? pageName.trim() : "Dashboard";
            this.pageName = resolved;
            this.pageId = "tab-" + slug(resolved);
            this.widgets = new ArrayList<>();
            this.autoCursorX = 0;
            this.autoCursorY = 0;
            this.autoColumns = 12;
        }

        public String pageName() {
            return pageName;
        }

        public String pageId() {
            return pageId;
        }

        public PageRef gridAuto() {
            // Uses default base-canvas density in host dashboard.
            return this;
        }

        public PageRef gridColumns(int columns) {
            this.autoColumns = Math.max(1, columns);
            return this;
        }

        public PageRef put(WidgetSpec spec) {
            if (spec == null) {
                return this;
            }

            ArcpDashboardLayout.LayoutRect layout = spec.layout();
            if (!spec.layoutExplicit()) {
                int w = Math.max(1, layout.w());
                int h = Math.max(1, layout.h());
                if (autoCursorX + w > autoColumns) {
                    autoCursorX = 0;
                    autoCursorY += 1;
                }
                layout = new ArcpDashboardLayout.LayoutRect(autoCursorX, autoCursorY, w, h);
                autoCursorX += w;
                if (autoCursorX >= autoColumns) {
                    autoCursorX = 0;
                    autoCursorY += h;
                }
            }

            ArcpDashboardLayout.Widget.Builder widget = ArcpDashboardLayout.Widget.builder()
                    .signalId(spec.signalId())
                    .kind(spec.kind())
                    .title(spec.title() != null ? spec.title() : defaultWidgetTitle(spec.kind()))
                    .layout(layout.x(), layout.y(), layout.w(), layout.h());
            if (spec.id() != null && !spec.id().isBlank()) {
                widget.id(spec.id());
            }
            if (spec.parentLayoutId() != null && !spec.parentLayoutId().isBlank()) {
                widget.parentLayoutId(spec.parentLayoutId());
            }
            if (spec.topicPath() != null && !spec.topicPath().isBlank()) {
                widget.config("topicPath", spec.topicPath());
            }
            if (!spec.bindings().isEmpty()) {
                widget.config("bindings", new LinkedHashMap<>(spec.bindings()));
            }
            if (!spec.config().isEmpty()) {
                widget.config(spec.config());
            }
            widgets.add(widget.build());
            return this;
        }

        public LayoutRef endPage() {
            return owner;
        }

        private ArcpDashboardLayout.Page build() {
            ArcpDashboardLayout.Page.Builder page = ArcpDashboardLayout.Page.builder()
                    .id(pageId)
                    .name(pageName);
            for (ArcpDashboardLayout.Widget widget : widgets) {
                page.widget(widget);
            }
            return page.build();
        }
    }

    private void decodeEventRecord(ByteBuffer record) {
        if (record.remaining() < 5) {
            return;
        }

        int version = Byte.toUnsignedInt(record.get());
        if (version != ARCP_PROTOCOL_VERSION) {
            return;
        }
        int eventKind = Byte.toUnsignedInt(record.get());
        int signalId = Short.toUnsignedInt(record.getShort());

        if (eventKind == EVENT_ACTION) {
            Runnable action = actionHandlers.get(signalId);
            if (action != null) {
                action.run();
            }
            return;
        }
        if (eventKind != EVENT_TUNABLE_SET) {
            return;
        }
        if (record.remaining() < 1) {
            return;
        }

        int signalType = Byte.toUnsignedInt(record.get());
        switch (signalType) {
            case 1 -> {
                boolean value = record.remaining() >= 1 && record.get() != 0;
                cacheValueForSignalId(signalId, ArcpRuntime.SignalType.BOOL, value);
                BooleanSetter handler = boolSetHandlers.get(signalId);
                if (handler != null) {
                    handler.accept(value);
                }
            }
            case 2 -> {
                if (record.remaining() < Long.BYTES) {
                    return;
                }
                long value = record.getLong();
                cacheValueForSignalId(signalId, ArcpRuntime.SignalType.I64, value);
                LongConsumer handler = i64SetHandlers.get(signalId);
                if (handler != null) {
                    handler.accept(value);
                }
            }
            case 3 -> {
                if (record.remaining() < Double.BYTES) {
                    return;
                }
                double value = record.getDouble();
                cacheValueForSignalId(signalId, ArcpRuntime.SignalType.F64, value);
                DoubleConsumer handler = f64SetHandlers.get(signalId);
                if (handler != null) {
                    handler.accept(value);
                }
            }
            case 4 -> {
                String value = decodeString(record);
                if (value == null) {
                    return;
                }
                cacheValueForSignalId(signalId, ArcpRuntime.SignalType.STRING, value);
                Consumer<String> handler = stringSetHandlers.get(signalId);
                if (handler != null) {
                    handler.accept(value);
                }
            }
            default -> {
                // Ignore unsupported event payload types for now.
            }
        }
    }

    private void patchMetadata(String rawPath, Map<String, ?> metadataPatch) {
        if (metadataPatch == null || metadataPatch.isEmpty()) {
            return;
        }
        String path = normalizePath(rawPath);
        synchronized (signalIds) {
            LinkedHashMap<String, Object> current = metadataByPath.computeIfAbsent(path, ignored -> new LinkedHashMap<>());
            LinkedHashMap<String, Object> appliedPatch = new LinkedHashMap<>();

            for (Map.Entry<String, ?> entry : metadataPatch.entrySet()) {
                if (entry == null || entry.getKey() == null || entry.getKey().isBlank()) {
                    continue;
                }
                String key = entry.getKey().trim();
                Object value = entry.getValue();
                if (value == null) {
                    if (current.remove(key) != null) {
                        appliedPatch.put(key, null);
                    }
                    continue;
                }
                Object existing = current.get(key);
                if (Objects.equals(existing, value)) {
                    continue;
                }
                current.put(key, value);
                appliedPatch.put(key, value);
            }

            if (appliedPatch.isEmpty()) {
                return;
            }

            long version = metadataVersionByPath.getOrDefault(path, 0L) + 1L;
            metadataVersionByPath.put(path, version);

            String hash = metadataHash(current);
            publishMetadataPatch(path, version, hash, appliedPatch, current);
        }
    }

    private void publishMetadataPatch(
            String path,
            long version,
            String hash,
            Map<String, Object> patch,
            Map<String, Object> full) {
        String patchPath = path + "/" + META_PATCH_SUFFIX;
        String versionPath = path + "/" + META_VERSION_SUFFIX;
        String hashPath = path + "/" + META_HASH_SUFFIX;
        String fullPath = path + "/" + META_FULL_SUFFIX;

        int patchSignalId = ensureSignal(
                patchPath,
                ArcpRuntime.SignalType.STRING,
                ArcpRuntime.SignalKind.TELEMETRY,
                ArcpRuntime.SignalAccess.OBSERVE,
                ArcpRuntime.SignalPolicy.ON_CHANGE,
                ArcpRuntime.SignalDurability.RETAINED);
        int versionSignalId = ensureSignal(
                versionPath,
                ArcpRuntime.SignalType.I64,
                ArcpRuntime.SignalKind.TELEMETRY,
                ArcpRuntime.SignalAccess.OBSERVE,
                ArcpRuntime.SignalPolicy.ON_CHANGE,
                ArcpRuntime.SignalDurability.RETAINED);
        int hashSignalId = ensureSignal(
                hashPath,
                ArcpRuntime.SignalType.STRING,
                ArcpRuntime.SignalKind.TELEMETRY,
                ArcpRuntime.SignalAccess.OBSERVE,
                ArcpRuntime.SignalPolicy.ON_CHANGE,
                ArcpRuntime.SignalDurability.RETAINED);
        int fullSignalId = ensureSignal(
                fullPath,
                ArcpRuntime.SignalType.STRING,
                ArcpRuntime.SignalKind.TELEMETRY,
                ArcpRuntime.SignalAccess.OBSERVE,
                ArcpRuntime.SignalPolicy.ON_CHANGE,
                ArcpRuntime.SignalDurability.RETAINED);

        LinkedHashMap<String, Object> patchEnvelope = new LinkedHashMap<>();
        patchEnvelope.put("path", path);
        patchEnvelope.put("version", version);
        patchEnvelope.put("patch", patch);

        LinkedHashMap<String, Object> fullEnvelope = new LinkedHashMap<>();
        fullEnvelope.put("path", path);
        fullEnvelope.put("version", version);
        fullEnvelope.put("meta", full);

        String patchJson = encodeJson(patchEnvelope);
        String fullJson = encodeJson(fullEnvelope);

        runtime.publishString(patchSignalId, patchJson);
        runtime.publishI64(versionSignalId, version);
        runtime.publishString(hashSignalId, hash);

        // Keep a retained full copy to help clients recover from version/hash mismatch.
        runtime.publishString(fullSignalId, fullJson);

        cacheValue(patchPath, ArcpRuntime.SignalType.STRING, patchJson);
        cacheValue(versionPath, ArcpRuntime.SignalType.I64, version);
        cacheValue(hashPath, ArcpRuntime.SignalType.STRING, hash);
        cacheValue(fullPath, ArcpRuntime.SignalType.STRING, fullJson);
    }

    private int ensureSignal(String rawPath, ArcpRuntime.SignalType signalType) {
        return ensureSignal(
                rawPath,
                signalType,
                ArcpRuntime.SignalKind.TELEMETRY,
                ArcpRuntime.SignalAccess.OBSERVE,
                ArcpRuntime.SignalPolicy.ON_CHANGE,
                ArcpRuntime.SignalDurability.VOLATILE);
    }

    private int ensureSignal(
            String rawPath,
            ArcpRuntime.SignalType signalType,
            ArcpRuntime.SignalKind signalKind,
            ArcpRuntime.SignalAccess signalAccess,
            ArcpRuntime.SignalPolicy signalPolicy,
            ArcpRuntime.SignalDurability signalDurability) {
        String path = normalizePath(rawPath);
        SignalKey key = new SignalKey(path, signalType);
        String canonicalPath = canonicalPath(path);
        SignalKey canonicalKey = new SignalKey(canonicalPath, signalType);
        synchronized (signalIds) {
            SignalRegistration existing = signalIds.get(key);
            if (existing == null && !canonicalPath.isEmpty()) {
                existing = canonicalSignalIds.get(canonicalKey);
            }
            if (existing != null) {
                if (existing.signalKind() != signalKind || existing.signalAccess() != signalAccess) {
                    if (existing.signalKind() == ArcpRuntime.SignalKind.TELEMETRY
                            && signalKind == ArcpRuntime.SignalKind.TELEMETRY
                            && ((existing.signalAccess() == ArcpRuntime.SignalAccess.WRITE
                                            && signalAccess == ArcpRuntime.SignalAccess.OBSERVE)
                                    || (existing.signalAccess() == ArcpRuntime.SignalAccess.OBSERVE
                                            && signalAccess == ArcpRuntime.SignalAccess.WRITE))) {
                        ArcpRuntime.SignalAccess upgradedAccess =
                                (existing.signalAccess() == ArcpRuntime.SignalAccess.WRITE
                                                || signalAccess == ArcpRuntime.SignalAccess.WRITE)
                                        ? ArcpRuntime.SignalAccess.WRITE
                                        : ArcpRuntime.SignalAccess.OBSERVE;
                        if (upgradedAccess != existing.signalAccess()) {
                            int existingSignalId = existing.signalId();
                            runtime.registerSignal(
                                    existingSignalId,
                                    existing.signalType(),
                                    existing.signalKind(),
                                    upgradedAccess,
                                    existing.signalPolicy(),
                                    existing.signalDurability(),
                                    existing.path());
                            SignalRegistration upgraded = new SignalRegistration(
                                    existing.signalId(),
                                    existing.path(),
                                    existing.signalType(),
                                    existing.signalKind(),
                                    upgradedAccess,
                                    existing.signalPolicy(),
                                    existing.signalDurability());
                            signalIds.replaceAll((ignored, registration) ->
                                    registration != null && registration.signalId() == existingSignalId
                                            ? upgraded
                                            : registration);
                            canonicalSignalIds.replaceAll((ignored, registration) ->
                                    registration != null && registration.signalId() == existingSignalId
                                            ? upgraded
                                            : registration);
                            registrationsById.put(existingSignalId, upgraded);
                            existing = upgraded;
                        }
                        signalIds.putIfAbsent(key, existing);
                        if (!canonicalPath.isEmpty()) {
                            canonicalSignalIds.putIfAbsent(canonicalKey, existing);
                        }
                        return existing.signalId();
                    }
                    throw new IllegalStateException(
                            "ARCP signal conflict for path '"
                                    + path
                                    + "' (existing="
                                    + existing.signalKind()
                                    + "/"
                                    + existing.signalAccess()
                                    + ", requested="
                                    + signalKind
                                    + "/"
                                    + signalAccess
                                    + ")");
                }
                signalIds.putIfAbsent(key, existing);
                if (!canonicalPath.isEmpty()) {
                    canonicalSignalIds.putIfAbsent(canonicalKey, existing);
                }
                return existing.signalId();
            }

            int signalId = nextSignalId();
            runtime.registerSignal(
                    signalId,
                    signalType,
                    signalKind,
                    signalAccess,
                    signalPolicy,
                    signalDurability,
                    path);
            nextSignalId = signalId + 1;

            SignalRegistration registration = new SignalRegistration(
                    signalId,
                    path,
                    signalType,
                    signalKind,
                    signalAccess,
                    signalPolicy,
                    signalDurability);

            signalIds.put(key, registration);
            registrationsById.put(signalId, registration);
            if (!canonicalPath.isEmpty()) {
                canonicalSignalIds.putIfAbsent(canonicalKey, registration);
            }
            return signalId;
        }
    }

    private int existingSignalIdInternal(String rawPath, ArcpRuntime.SignalType signalType) {
        String path = normalizePath(rawPath);
        SignalKey key = new SignalKey(path, signalType);
        String canonicalPath = canonicalPath(path);
        SignalKey canonicalKey = new SignalKey(canonicalPath, signalType);
        synchronized (signalIds) {
            SignalRegistration existing = signalIds.get(key);
            if (existing == null && !canonicalPath.isEmpty()) {
                existing = canonicalSignalIds.get(canonicalKey);
            }
            if (existing != null) {
                signalIds.putIfAbsent(key, existing);
                if (!canonicalPath.isEmpty()) {
                    canonicalSignalIds.putIfAbsent(canonicalKey, existing);
                }
                return existing.signalId();
            }
            return 0;
        }
    }

    private int existingSignalIdAnyType(String rawPath) {
        String path = normalizePath(rawPath);
        String canonicalPath = canonicalPath(path);
        synchronized (signalIds) {
            for (Map.Entry<SignalKey, SignalRegistration> entry : signalIds.entrySet()) {
                if (entry == null) {
                    continue;
                }
                SignalKey key = entry.getKey();
                if (key == null) {
                    continue;
                }
                if (path.equals(key.path())) {
                    SignalRegistration registration = entry.getValue();
                    if (registration != null) {
                        return registration.signalId();
                    }
                }
            }
            if (!canonicalPath.isEmpty()) {
                for (Map.Entry<SignalKey, SignalRegistration> entry : canonicalSignalIds.entrySet()) {
                    if (entry == null) {
                        continue;
                    }
                    SignalKey key = entry.getKey();
                    if (key == null) {
                        continue;
                    }
                    if (canonicalPath.equals(key.path())) {
                        SignalRegistration registration = entry.getValue();
                        if (registration != null) {
                            return registration.signalId();
                        }
                    }
                }
            }
            return 0;
        }
    }

    private int nextSignalId() {
        if (nextSignalId > MAX_SIGNAL_ID) {
            throw new IllegalStateException("ARCP signal ID space exhausted (max=" + MAX_SIGNAL_ID + ")");
        }
        return nextSignalId;
    }

    private void cacheValueForSignalId(int signalId, ArcpRuntime.SignalType signalType, Object value) {
        SignalRegistration registration = registrationsById.get(signalId);
        if (registration == null) {
            return;
        }
        cacheValue(registration.path(), signalType, value);
    }

    private void cacheValue(String rawPath, ArcpRuntime.SignalType signalType, Object value) {
        String path = normalizePath(rawPath);
        synchronized (signalIds) {
            valueCache.put(path, value);
            valueTypes.put(path, signalType);
        }
    }

    private static String decodeString(ByteBuffer source) {
        if (source.remaining() < 2) {
            return null;
        }
        int len = Short.toUnsignedInt(source.getShort());
        if (len < 0 || len > source.remaining()) {
            return null;
        }
        byte[] bytes = new byte[len];
        source.get(bytes);
        return new String(bytes, StandardCharsets.UTF_8);
    }

    private static String normalizePath(String rawPath) {
        if (rawPath == null) {
            return "Athena/unknown";
        }
        String trimmed = rawPath.trim();
        if (trimmed.isEmpty()) {
            return "Athena/unknown";
        }
        if (trimmed.startsWith("/")) {
            return trimmed.substring(1);
        }
        return trimmed;
    }

    private static String canonicalPath(String path) {
        if (path == null || path.isBlank()) {
            return "";
        }
        String normalized = normalizePath(path);
        String[] segments = normalized.split("/");
        StringBuilder out = new StringBuilder(normalized.length());
        for (int i = 0; i < segments.length; i++) {
            String segment = segments[i];
            if (segment == null || segment.isBlank()) {
                continue;
            }
            StringBuilder clean = new StringBuilder(segment.length());
            for (int j = 0; j < segment.length(); j++) {
                char ch = Character.toLowerCase(segment.charAt(j));
                if ((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9')) {
                    clean.append(ch);
                }
            }
            if (clean.length() == 0) {
                continue;
            }
            if (out.length() > 0) {
                out.append('/');
            }
            out.append(clean);
        }
        return out.toString();
    }

    @SuppressWarnings("unchecked")
    private static <T> Class<T> boxedClass(Class<T> clazz) {
        if (!clazz.isPrimitive()) {
            return clazz;
        }
        if (clazz == boolean.class) {
            return (Class<T>) Boolean.class;
        }
        if (clazz == long.class) {
            return (Class<T>) Long.class;
        }
        if (clazz == double.class) {
            return (Class<T>) Double.class;
        }
        if (clazz == int.class) {
            return (Class<T>) Integer.class;
        }
        if (clazz == short.class) {
            return (Class<T>) Short.class;
        }
        if (clazz == byte.class) {
            return (Class<T>) Byte.class;
        }
        if (clazz == float.class) {
            return (Class<T>) Float.class;
        }
        if (clazz == char.class) {
            return (Class<T>) Character.class;
        }
        throw new IllegalArgumentException("Unsupported primitive class: " + clazz.getName());
    }

    private static ArcpRuntime.SignalType signalTypeForClass(Class<?> valueClass) {
        if (valueClass == Boolean.class) {
            return ArcpRuntime.SignalType.BOOL;
        }
        if (valueClass == Long.class
                || valueClass == Integer.class
                || valueClass == Short.class
                || valueClass == Byte.class) {
            return ArcpRuntime.SignalType.I64;
        }
        if (valueClass == Double.class || valueClass == Float.class) {
            return ArcpRuntime.SignalType.F64;
        }
        if (valueClass == String.class) {
            return ArcpRuntime.SignalType.STRING;
        }
        if (valueClass == boolean[].class) {
            return ArcpRuntime.SignalType.BOOL_ARRAY;
        }
        if (valueClass == long[].class) {
            return ArcpRuntime.SignalType.I64_ARRAY;
        }
        if (valueClass == double[].class) {
            return ArcpRuntime.SignalType.F64_ARRAY;
        }
        if (valueClass == String[].class) {
            return ArcpRuntime.SignalType.STRING_ARRAY;
        }
        throw new IllegalArgumentException("Unsupported ARCP value class: " + valueClass.getName());
    }

    private static Object coerceValue(Object value, Class<?> requestedClass) {
        if (requestedClass.isInstance(value)) {
            return value;
        }
        if (value instanceof Number number) {
            if (requestedClass == Double.class) {
                return number.doubleValue();
            }
            if (requestedClass == Long.class) {
                return number.longValue();
            }
            if (requestedClass == Integer.class) {
                return number.intValue();
            }
        }
        return null;
    }

    private static String metadataHash(Map<String, Object> metadata) {
        TreeMap<String, Object> sorted = new TreeMap<>(metadata);
        String json = encodeJson(sorted);
        CRC32 crc = new CRC32();
        byte[] bytes = json.getBytes(StandardCharsets.UTF_8);
        crc.update(bytes, 0, bytes.length);
        return Long.toHexString(crc.getValue());
    }

    private static String encodeJson(Object value) {
        try {
            return JSON.writeValueAsString(value);
        } catch (JsonProcessingException ex) {
            throw new IllegalStateException("Failed to encode ARCP JSON payload", ex);
        }
    }

    private static String sanitizeWidgetKind(String kind) {
        if (kind == null || kind.isBlank()) {
            return "metric";
        }
        return kind.trim();
    }

    private static String normalizeTopicPath(String rawPath) {
        if (rawPath == null) {
            return "";
        }
        String trimmed = rawPath.trim();
        if (trimmed.isEmpty()) {
            return "";
        }
        String normalized = trimmed.replace('\\', '/');
        while (normalized.startsWith("/")) {
            normalized = normalized.substring(1);
        }
        while (normalized.endsWith("/")) {
            normalized = normalized.substring(0, normalized.length() - 1);
        }
        return normalized;
    }

    private static String normalizeLayoutProfileName(String profileName) {
        if (profileName == null || profileName.isBlank()) {
            return "atheana-generated";
        }
        return profileName.trim();
    }

    private static String defaultWidgetTitle(String kind) {
        if (kind == null || kind.isBlank()) {
            return "Widget";
        }
        if (kind.indexOf('/') >= 0) {
            String[] segments = kind.split("/");
            return segments[segments.length - 1];
        }
        return kind;
    }

    private static String slug(String value) {
        String lower = value == null ? "" : value.trim().toLowerCase();
        if (lower.isEmpty()) {
            return "page";
        }
        StringBuilder out = new StringBuilder(lower.length());
        boolean lastDash = false;
        for (int i = 0; i < lower.length(); i++) {
            char ch = lower.charAt(i);
            boolean valid = (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9');
            if (valid) {
                out.append(ch);
                lastDash = false;
                continue;
            }
            if (!lastDash) {
                out.append('-');
                lastDash = true;
            }
        }
        String slug = out.toString();
        while (slug.startsWith("-")) {
            slug = slug.substring(1);
        }
        while (slug.endsWith("-")) {
            slug = slug.substring(0, slug.length() - 1);
        }
        return slug.isEmpty() ? "page" : slug;
    }

    private record SignalKey(String path, ArcpRuntime.SignalType signalType) {}

    private record SignalRegistration(
            int signalId,
            String path,
            ArcpRuntime.SignalType signalType,
            ArcpRuntime.SignalKind signalKind,
            ArcpRuntime.SignalAccess signalAccess,
            ArcpRuntime.SignalPolicy signalPolicy,
            ArcpRuntime.SignalDurability signalDurability) {}
}
