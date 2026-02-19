package ca.frc6390.athena.networktables;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DriverStation;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Athena-first NetworkTables facade.
 *
 * <p>All keys are normalized under {@code /Athena/...} regardless of callsite scope.
 */
public final class AthenaNT {
    public static final String ROOT = "Athena";
    private static final String ROOT_PATH = "/" + ROOT;

    private static final ObjectMapper MAPPER = new ObjectMapper();
    private static volatile NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static final ScopeImpl ROOT_SCOPE = new ScopeImpl(ROOT_PATH);
    private static final CopyOnWriteArrayList<BindingImpl> BINDINGS = new CopyOnWriteArrayList<>();

    private AthenaNT() {}

    /**
     * Returns the global root scope ({@code /Athena}).
     */
    public static NtScope root() {
        return ROOT_SCOPE;
    }

    /**
     * Returns a scope rooted under {@code /Athena/<scope>}.
     */
    public static NtScope scope(String scope) {
        return ROOT_SCOPE.scope(scope);
    }

    /**
     * Top-level unified put API.
     */
    public static void put(String key, Object value) {
        ROOT_SCOPE.put(key, value);
    }

    public static boolean getBoolean(String key, boolean defaultValue) {
        return ROOT_SCOPE.getBoolean(key, defaultValue);
    }

    public static long getInt(String key, long defaultValue) {
        return ROOT_SCOPE.getInt(key, defaultValue);
    }

    public static double getDouble(String key, double defaultValue) {
        return ROOT_SCOPE.getDouble(key, defaultValue);
    }

    public static String getString(String key, String defaultValue) {
        return ROOT_SCOPE.getString(key, defaultValue);
    }

    public static <T> T get(String key, Class<T> type, T defaultValue) {
        return ROOT_SCOPE.get(key, type, defaultValue);
    }

    public static <T> T get(String key, TypeReference<T> type, T defaultValue) {
        return ROOT_SCOPE.get(key, type, defaultValue);
    }

    public static <T> Optional<T> tryGet(String key, Class<T> type) {
        return ROOT_SCOPE.tryGet(key, type);
    }

    public static boolean contains(String key) {
        return ROOT_SCOPE.contains(key);
    }

    /**
     * Binds AthenaNT annotations using the root scope.
     */
    public static AthenaNTBinding bind(Object target) {
        return ROOT_SCOPE.bind(target);
    }

    /**
     * Binds AthenaNT annotations using the provided relative scope under {@code /Athena}.
     */
    public static AthenaNTBinding bind(String scope, Object target) {
        return ROOT_SCOPE.scope(scope).bind(target);
    }

    /**
     * Advances all active annotation bindings.
     */
    public static void tick() {
        if (BINDINGS.isEmpty()) {
            return;
        }
        long nowMs = System.currentTimeMillis();
        for (BindingImpl binding : BINDINGS) {
            if (binding == null) {
                continue;
            }
            binding.tick(nowMs);
        }
    }

    static void putAtPath(String absolutePath, Object value) {
        if (value == null) {
            return;
        }
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return;
        }
        if (value instanceof Boolean bool) {
            entry.setBoolean(bool);
            return;
        }
        if (value instanceof Byte || value instanceof Short || value instanceof Integer || value instanceof Long) {
            entry.setInteger(((Number) value).longValue());
            return;
        }
        if (value instanceof Float || value instanceof Double) {
            entry.setDouble(((Number) value).doubleValue());
            return;
        }
        if (value instanceof CharSequence sequence) {
            entry.setString(sequence.toString());
            return;
        }
        if (value instanceof Enum<?> enumValue) {
            entry.setString(enumValue.name());
            return;
        }
        if (value instanceof boolean[] arr) {
            entry.setBooleanArray(arr);
            return;
        }
        if (value instanceof double[] arr) {
            entry.setDoubleArray(arr);
            return;
        }
        if (value instanceof float[] arr) {
            double[] converted = new double[arr.length];
            for (int i = 0; i < arr.length; i++) {
                converted[i] = arr[i];
            }
            entry.setDoubleArray(converted);
            return;
        }
        if (value instanceof int[] arr) {
            entry.setIntegerArray(toLongArray(arr));
            return;
        }
        if (value instanceof long[] arr) {
            entry.setIntegerArray(arr);
            return;
        }
        if (value instanceof short[] arr) {
            long[] converted = new long[arr.length];
            for (int i = 0; i < arr.length; i++) {
                converted[i] = arr[i];
            }
            entry.setIntegerArray(converted);
            return;
        }
        if (value instanceof byte[] arr) {
            long[] converted = new long[arr.length];
            for (int i = 0; i < arr.length; i++) {
                converted[i] = arr[i];
            }
            entry.setIntegerArray(converted);
            return;
        }
        if (value instanceof String[] arr) {
            entry.setStringArray(arr);
            return;
        }
        Class<?> valueType = value.getClass();
        if (valueType.isArray()) {
            int length = Array.getLength(value);
            String[] out = new String[length];
            for (int i = 0; i < length; i++) {
                Object element = Array.get(value, i);
                out[i] = element != null ? element.toString() : "";
            }
            entry.setStringArray(out);
            return;
        }
        try {
            entry.setString(MAPPER.writeValueAsString(value));
        } catch (Exception ex) {
            DriverStation.reportWarning("AthenaNT put JSON failed for " + absolutePath + ": " + ex.getMessage(), false);
        }
    }

    static boolean getBooleanAtPath(String absolutePath, boolean defaultValue) {
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return defaultValue;
        }
        Object raw = readRaw(entry);
        if (raw == null) {
            return defaultValue;
        }
        if (raw instanceof Boolean value) {
            return value;
        }
        if (raw instanceof Number number) {
            return number.doubleValue() != 0.0;
        }
        if (raw instanceof String text) {
            if ("true".equalsIgnoreCase(text)) {
                return true;
            }
            if ("false".equalsIgnoreCase(text)) {
                return false;
            }
        }
        return defaultValue;
    }

    static long getIntAtPath(String absolutePath, long defaultValue) {
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return defaultValue;
        }
        Object raw = readRaw(entry);
        if (raw == null) {
            return defaultValue;
        }
        if (raw instanceof Number number) {
            return number.longValue();
        }
        if (raw instanceof Boolean flag) {
            return flag ? 1L : 0L;
        }
        if (raw instanceof String text) {
            try {
                return Long.parseLong(text.trim());
            } catch (NumberFormatException ignored) {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    static double getDoubleAtPath(String absolutePath, double defaultValue) {
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return defaultValue;
        }
        Object raw = readRaw(entry);
        if (raw == null) {
            return defaultValue;
        }
        if (raw instanceof Number number) {
            return number.doubleValue();
        }
        if (raw instanceof Boolean flag) {
            return flag ? 1.0 : 0.0;
        }
        if (raw instanceof String text) {
            try {
                return Double.parseDouble(text.trim());
            } catch (NumberFormatException ignored) {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    static String getStringAtPath(String absolutePath, String defaultValue) {
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return defaultValue;
        }
        Object raw = readRaw(entry);
        if (raw == null) {
            return defaultValue;
        }
        if (raw instanceof String text) {
            return text;
        }
        if (raw.getClass().isArray()) {
            if (raw instanceof Object[] objArray) {
                return Arrays.toString(objArray);
            }
            if (raw instanceof long[] arr) {
                return Arrays.toString(arr);
            }
            if (raw instanceof double[] arr) {
                return Arrays.toString(arr);
            }
            if (raw instanceof boolean[] arr) {
                return Arrays.toString(arr);
            }
        }
        return raw.toString();
    }

    static <T> T getAtPath(String absolutePath, Class<T> type, T defaultValue) {
        if (type == null) {
            return defaultValue;
        }
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return defaultValue;
        }
        Object raw = readRaw(entry);
        if (raw == null) {
            return defaultValue;
        }
        try {
            Object converted = convertRawValue(raw, type, defaultValue);
            if (converted == null) {
                return defaultValue;
            }
            return type.cast(converted);
        } catch (Exception ex) {
            DriverStation.reportWarning("AthenaNT get failed for " + absolutePath + ": " + ex.getMessage(), false);
            return defaultValue;
        }
    }

    static Object getAtPathDynamic(String absolutePath, Class<?> type, Object defaultValue) {
        if (type == null) {
            return defaultValue;
        }
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return defaultValue;
        }
        Object raw = readRaw(entry);
        if (raw == null) {
            return defaultValue;
        }
        try {
            Object converted = convertRawValue(raw, type, defaultValue);
            return converted != null ? converted : defaultValue;
        } catch (Exception ex) {
            DriverStation.reportWarning("AthenaNT get failed for " + absolutePath + ": " + ex.getMessage(), false);
            return defaultValue;
        }
    }

    static <T> T getAtPath(String absolutePath, TypeReference<T> type, T defaultValue) {
        if (type == null) {
            return defaultValue;
        }
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return defaultValue;
        }
        Object raw = readRaw(entry);
        if (raw == null) {
            return defaultValue;
        }
        try {
            if (raw instanceof String text) {
                return MAPPER.readValue(text, type);
            }
            return MAPPER.convertValue(raw, type);
        } catch (Exception ex) {
            DriverStation.reportWarning("AthenaNT TypeReference get failed for " + absolutePath + ": " + ex.getMessage(), false);
            return defaultValue;
        }
    }

    static boolean containsPath(String absolutePath) {
        NetworkTableEntry entry = entryForAbsolutePath(absolutePath);
        if (entry == null) {
            return false;
        }
        return entry.getType() != NetworkTableType.kUnassigned;
    }

    private static Object convertRawValue(Object raw, Class<?> type, Object defaultValue) throws Exception {
        if (raw == null || type == null) {
            return defaultValue;
        }
        if (type.isInstance(raw)) {
            return raw;
        }
        if (type == String.class) {
            return raw.toString();
        }
        if (type == boolean.class || type == Boolean.class) {
            if (raw instanceof Boolean b) {
                return b;
            }
            if (raw instanceof Number n) {
                return n.doubleValue() != 0.0;
            }
            if (raw instanceof String s) {
                return Boolean.parseBoolean(s.trim());
            }
            return defaultValue;
        }
        if (type == int.class || type == Integer.class) {
            if (raw instanceof Number n) {
                return n.intValue();
            }
            if (raw instanceof String s) {
                return Integer.parseInt(s.trim());
            }
            return defaultValue;
        }
        if (type == long.class || type == Long.class) {
            if (raw instanceof Number n) {
                return n.longValue();
            }
            if (raw instanceof String s) {
                return Long.parseLong(s.trim());
            }
            return defaultValue;
        }
        if (type == double.class || type == Double.class) {
            if (raw instanceof Number n) {
                return n.doubleValue();
            }
            if (raw instanceof String s) {
                return Double.parseDouble(s.trim());
            }
            return defaultValue;
        }
        if (type == float.class || type == Float.class) {
            if (raw instanceof Number n) {
                return n.floatValue();
            }
            if (raw instanceof String s) {
                return Float.parseFloat(s.trim());
            }
            return defaultValue;
        }
        if (type == short.class || type == Short.class) {
            if (raw instanceof Number n) {
                return n.shortValue();
            }
            if (raw instanceof String s) {
                return Short.parseShort(s.trim());
            }
            return defaultValue;
        }
        if (type == byte.class || type == Byte.class) {
            if (raw instanceof Number n) {
                return n.byteValue();
            }
            if (raw instanceof String s) {
                return Byte.parseByte(s.trim());
            }
            return defaultValue;
        }
        if (type.isEnum()) {
            if (raw instanceof String name) {
                @SuppressWarnings({"rawtypes", "unchecked"})
                Enum<?> value = Enum.valueOf((Class<? extends Enum>) type, name.trim());
                return value;
            }
            return defaultValue;
        }
        if (type == long[].class) {
            if (raw instanceof long[] arr) {
                return arr;
            }
            if (raw instanceof int[] arr) {
                return toLongArray(arr);
            }
            return defaultValue;
        }
        if (type == int[].class) {
            if (raw instanceof int[] arr) {
                return arr;
            }
            if (raw instanceof long[] arr) {
                int[] converted = new int[arr.length];
                for (int i = 0; i < arr.length; i++) {
                    converted[i] = (int) arr[i];
                }
                return converted;
            }
            return defaultValue;
        }
        if (type == double[].class) {
            if (raw instanceof double[] arr) {
                return arr;
            }
            return defaultValue;
        }
        if (type == boolean[].class) {
            if (raw instanceof boolean[] arr) {
                return arr;
            }
            return defaultValue;
        }
        if (type == String[].class) {
            if (raw instanceof String[] arr) {
                return arr;
            }
            return defaultValue;
        }
        if (raw instanceof String text) {
            return MAPPER.readValue(text, type);
        }
        return MAPPER.convertValue(raw, type);
    }

    private static Object readRaw(NetworkTableEntry entry) {
        if (entry == null) {
            return null;
        }
        NetworkTableType type = entry.getType();
        if (type == null || type == NetworkTableType.kUnassigned) {
            return null;
        }
        return switch (type) {
            case kBoolean -> entry.getBoolean(false);
            case kDouble -> entry.getDouble(0.0);
            case kInteger -> entry.getInteger(0L);
            case kString -> entry.getString("");
            case kBooleanArray -> entry.getBooleanArray(new boolean[0]);
            case kDoubleArray -> entry.getDoubleArray(new double[0]);
            case kIntegerArray -> entry.getIntegerArray(new long[0]);
            case kStringArray -> entry.getStringArray(new String[0]);
            case kRaw -> entry.getRaw(new byte[0]);
            default -> null;
        };
    }

    private static NetworkTableEntry entryForAbsolutePath(String absolutePath) {
        String normalized = normalizeAbsolutePath(absolutePath);
        String relative = toRelativeKey(normalized);
        if (relative.isEmpty()) {
            return null;
        }
        return rootTable().getEntry(relative);
    }

    private static NetworkTable rootTable() {
        return instance.getTable(ROOT);
    }

    static String normalizeScopePath(String scope) {
        if (scope == null || scope.isBlank()) {
            return ROOT_PATH;
        }
        String normalized = collapsePath(scope);
        if (normalized.isEmpty()) {
            return ROOT_PATH;
        }
        if (normalized.equals(ROOT) || normalized.equals(ROOT_PATH)) {
            return ROOT_PATH;
        }
        String stripped = stripLeadingSlash(normalized);
        if (stripped.equals(ROOT)) {
            return ROOT_PATH;
        }
        if (stripped.startsWith(ROOT + "/")) {
            return "/" + stripped;
        }
        return ROOT_PATH + "/" + stripped;
    }

    static String resolveAbsoluteKey(String scopePath, String key) {
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("key must not be blank");
        }
        String normalizedScope = normalizeScopePath(scopePath);
        String keyNormalized = collapsePath(key);
        if (keyNormalized.isEmpty()) {
            throw new IllegalArgumentException("key must not be blank");
        }
        if (keyNormalized.startsWith(ROOT_PATH) || keyNormalized.equals(ROOT_PATH)) {
            return normalizeAbsolutePath(keyNormalized);
        }
        String keyNoLeading = stripLeadingSlash(keyNormalized);
        if (keyNoLeading.equals(ROOT) || keyNoLeading.startsWith(ROOT + "/")) {
            return normalizeAbsolutePath("/" + keyNoLeading);
        }
        String relative = stripLeadingSlash(keyNoLeading);
        if (relative.isEmpty()) {
            throw new IllegalArgumentException("key must not be blank");
        }
        if (ROOT_PATH.equals(normalizedScope)) {
            return normalizeAbsolutePath(ROOT_PATH + "/" + relative);
        }
        return normalizeAbsolutePath(normalizedScope + "/" + relative);
    }

    private static String normalizeAbsolutePath(String absolutePath) {
        String normalized = collapsePath(absolutePath);
        if (normalized.isEmpty()) {
            return ROOT_PATH;
        }
        if (!normalized.startsWith("/")) {
            normalized = "/" + normalized;
        }
        String noLeading = stripLeadingSlash(normalized);
        if (noLeading.equals(ROOT)) {
            return ROOT_PATH;
        }
        if (noLeading.startsWith(ROOT + "/")) {
            return "/" + noLeading;
        }
        return ROOT_PATH + "/" + noLeading;
    }

    private static String toRelativeKey(String absolutePath) {
        if (absolutePath == null) {
            return "";
        }
        String normalized = normalizeAbsolutePath(absolutePath);
        if (ROOT_PATH.equals(normalized)) {
            return "";
        }
        String prefix = ROOT_PATH + "/";
        if (!normalized.startsWith(prefix)) {
            return "";
        }
        return normalized.substring(prefix.length());
    }

    private static String collapsePath(String raw) {
        if (raw == null) {
            return "";
        }
        String trimmed = raw.trim().replace('\\', '/');
        if (trimmed.isEmpty()) {
            return "";
        }
        StringBuilder out = new StringBuilder(trimmed.length());
        boolean lastSlash = false;
        for (int i = 0; i < trimmed.length(); i++) {
            char c = trimmed.charAt(i);
            if (c == '/') {
                if (!lastSlash) {
                    out.append(c);
                    lastSlash = true;
                }
            } else {
                out.append(c);
                lastSlash = false;
            }
        }
        while (out.length() > 1 && out.charAt(out.length() - 1) == '/') {
            out.setLength(out.length() - 1);
        }
        return out.toString();
    }

    private static String stripLeadingSlash(String value) {
        if (value == null || value.isEmpty()) {
            return "";
        }
        int index = 0;
        while (index < value.length() && value.charAt(index) == '/') {
            index++;
        }
        return value.substring(index);
    }

    private static long[] toLongArray(int[] source) {
        if (source == null) {
            return new long[0];
        }
        long[] out = new long[source.length];
        for (int i = 0; i < source.length; i++) {
            out[i] = source[i];
        }
        return out;
    }

    private static boolean valuesEqual(Object left, Object right, double epsilon) {
        if (left == right) {
            return true;
        }
        if (left == null || right == null) {
            return false;
        }
        if (left instanceof Number a && right instanceof Number b) {
            if (epsilon > 0.0) {
                return Math.abs(a.doubleValue() - b.doubleValue()) <= epsilon;
            }
            return Double.compare(a.doubleValue(), b.doubleValue()) == 0;
        }
        if (left instanceof boolean[] a && right instanceof boolean[] b) {
            return Arrays.equals(a, b);
        }
        if (left instanceof double[] a && right instanceof double[] b) {
            if (epsilon <= 0.0) {
                return Arrays.equals(a, b);
            }
            if (a.length != b.length) {
                return false;
            }
            for (int i = 0; i < a.length; i++) {
                if (Math.abs(a[i] - b[i]) > epsilon) {
                    return false;
                }
            }
            return true;
        }
        if (left instanceof long[] a && right instanceof long[] b) {
            return Arrays.equals(a, b);
        }
        if (left instanceof String[] a && right instanceof String[] b) {
            return Arrays.equals(a, b);
        }
        if (left.getClass().isArray() && right.getClass().isArray()) {
            return Objects.deepEquals(toObjectArray(left), toObjectArray(right));
        }
        return Objects.equals(left, right);
    }

    private static Object[] toObjectArray(Object array) {
        if (array == null || !array.getClass().isArray()) {
            return new Object[0];
        }
        int len = Array.getLength(array);
        Object[] out = new Object[len];
        for (int i = 0; i < len; i++) {
            out[i] = Array.get(array, i);
        }
        return out;
    }

    private static void reportAnnotationWarning(String text) {
        DriverStation.reportWarning("AthenaNT binding: " + text, false);
    }

    /**
     * Test-only hook used by unit tests in this package.
     */
    static void useInstanceForTests(NetworkTableInstance instance) {
        AthenaNT.instance = instance != null ? instance : NetworkTableInstance.getDefault();
    }

    /**
     * Test-only hook used by unit tests in this package.
     */
    static void clearBindingsForTests() {
        BINDINGS.clear();
    }

    private static final class ScopeImpl implements NtScope {
        private final String scopePath;

        private ScopeImpl(String scopePath) {
            this.scopePath = normalizeScopePath(scopePath);
        }

        @Override
        public String path() {
            return scopePath;
        }

        @Override
        public NtScope root() {
            return ROOT_SCOPE;
        }

        @Override
        public NtScope scope(String relative) {
            if (relative == null || relative.isBlank()) {
                return this;
            }
            String nested = resolveAbsoluteKey(scopePath, relative);
            return new ScopeImpl(nested);
        }

        @Override
        public void put(String key, Object value) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            putAtPath(absolute, value);
        }

        @Override
        public boolean getBoolean(String key, boolean defaultValue) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            return getBooleanAtPath(absolute, defaultValue);
        }

        @Override
        public long getInt(String key, long defaultValue) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            return getIntAtPath(absolute, defaultValue);
        }

        @Override
        public double getDouble(String key, double defaultValue) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            return getDoubleAtPath(absolute, defaultValue);
        }

        @Override
        public String getString(String key, String defaultValue) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            return getStringAtPath(absolute, defaultValue);
        }

        @Override
        public <T> T get(String key, Class<T> type, T defaultValue) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            return getAtPath(absolute, type, defaultValue);
        }

        @Override
        public <T> T get(String key, TypeReference<T> type, T defaultValue) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            return getAtPath(absolute, type, defaultValue);
        }

        @Override
        public <T> Optional<T> tryGet(String key, Class<T> type) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            if (!containsPath(absolute)) {
                return Optional.empty();
            }
            T value = getAtPath(absolute, type, null);
            return Optional.ofNullable(value);
        }

        @Override
        public boolean contains(String key) {
            String absolute = resolveAbsoluteKey(scopePath, key);
            return containsPath(absolute);
        }

        @Override
        public AthenaNTBinding bind(Object target) {
            if (target == null) {
                throw new IllegalArgumentException("target must not be null");
            }
            String bindingScope = scopePath;
            AthenaNTScope scopeAnnotation = target.getClass().getAnnotation(AthenaNTScope.class);
            if (scopeAnnotation != null && scopeAnnotation.value() != null && !scopeAnnotation.value().isBlank()) {
                bindingScope = resolveAbsoluteKey(bindingScope, scopeAnnotation.value());
            }
            BindingImpl binding = BindingImpl.create(bindingScope, target);
            BINDINGS.add(binding);
            return binding;
        }
    }

    private static final class BindingImpl implements AthenaNTBinding {
        private final String baseScopePath;
        private final Object target;
        private final List<MetricBinding> metrics;
        private final List<TunableBinding> tunables;
        private final List<ActionBinding> actions;
        private volatile boolean closed;

        private BindingImpl(
                String baseScopePath,
                Object target,
                List<MetricBinding> metrics,
                List<TunableBinding> tunables,
                List<ActionBinding> actions) {
            this.baseScopePath = baseScopePath;
            this.target = target;
            this.metrics = metrics;
            this.tunables = tunables;
            this.actions = actions;
        }

        static BindingImpl create(String baseScopePath, Object target) {
            MapBuilder callbackMap = new MapBuilder();
            List<MetricBinding> metrics = new ArrayList<>();
            List<TunableBinding> tunables = new ArrayList<>();
            List<ActionBinding> actions = new ArrayList<>();

            for (Method method : allMethods(target.getClass())) {
                AthenaNTOnChange onChange = method.getAnnotation(AthenaNTOnChange.class);
                if (onChange == null) {
                    continue;
                }
                if (method.getParameterCount() > 1) {
                    reportAnnotationWarning("@AthenaNTOnChange method must have 0 or 1 args: " + method.getName());
                    continue;
                }
                method.setAccessible(true);
                String fullKey = resolveAnnotationKey(baseScopePath, onChange.scope(), onChange.key());
                callbackMap.add(fullKey, method);
            }

            for (Field field : allFields(target.getClass())) {
                AthenaNTMetric metric = field.getAnnotation(AthenaNTMetric.class);
                if (metric != null) {
                    field.setAccessible(true);
                    String fullKey = resolveAnnotationKey(baseScopePath, metric.scope(), metric.key());
                    int periodMs = metric.periodMs() > 0 ? metric.periodMs() : 100;
                    metrics.add(new MetricBinding(fullKey, periodMs, metric.epsilon(), () -> field.get(target)));
                }
            }

            for (Method method : allMethods(target.getClass())) {
                AthenaNTMetric metric = method.getAnnotation(AthenaNTMetric.class);
                if (metric == null) {
                    continue;
                }
                if (method.getParameterCount() != 0) {
                    reportAnnotationWarning("@AthenaNTMetric method must be zero-arg: " + method.getName());
                    continue;
                }
                method.setAccessible(true);
                String fullKey = resolveAnnotationKey(baseScopePath, metric.scope(), metric.key());
                int periodMs = metric.periodMs() > 0 ? metric.periodMs() : 100;
                metrics.add(new MetricBinding(fullKey, periodMs, metric.epsilon(), () -> method.invoke(target)));
            }

            for (Field field : allFields(target.getClass())) {
                AthenaNTTunable tunable = field.getAnnotation(AthenaNTTunable.class);
                if (tunable == null) {
                    continue;
                }
                field.setAccessible(true);
                String fullKey = resolveAnnotationKey(baseScopePath, tunable.scope(), tunable.key());
                List<Method> callbacks = callbackMap.get(fullKey);
                TunableBinding binding = new TunableBinding(
                        fullKey,
                        field,
                        target,
                        tunable.min(),
                        tunable.max(),
                        tunable.step(),
                        callbacks);
                binding.initialize();
                tunables.add(binding);
            }

            for (Method method : allMethods(target.getClass())) {
                AthenaNTAction action = method.getAnnotation(AthenaNTAction.class);
                if (action == null) {
                    continue;
                }
                if (method.getParameterCount() != 0) {
                    reportAnnotationWarning("@AthenaNTAction method must be zero-arg: " + method.getName());
                    continue;
                }
                method.setAccessible(true);
                String fullKey = resolveAnnotationKey(baseScopePath, action.scope(), action.key());
                actions.add(new ActionBinding(fullKey, action.risingEdgeOnly(), target, method));
            }

            return new BindingImpl(
                    baseScopePath,
                    target,
                    Collections.unmodifiableList(metrics),
                    Collections.unmodifiableList(tunables),
                    Collections.unmodifiableList(actions));
        }

        private static List<Field> allFields(Class<?> type) {
            List<Field> out = new ArrayList<>();
            for (Class<?> c = type; c != null && c != Object.class; c = c.getSuperclass()) {
                out.addAll(Arrays.asList(c.getDeclaredFields()));
            }
            return out;
        }

        private static List<Method> allMethods(Class<?> type) {
            List<Method> out = new ArrayList<>();
            for (Class<?> c = type; c != null && c != Object.class; c = c.getSuperclass()) {
                out.addAll(Arrays.asList(c.getDeclaredMethods()));
            }
            return out;
        }

        private static String resolveAnnotationKey(String baseScopePath, String annotationScope, String key) {
            String scopePath = baseScopePath;
            if (annotationScope != null && !annotationScope.isBlank()) {
                scopePath = resolveAbsoluteKey(scopePath, annotationScope);
            }
            return resolveAbsoluteKey(scopePath, key);
        }

        void tick(long nowMs) {
            if (closed) {
                return;
            }
            for (MetricBinding metric : metrics) {
                if (metric == null) {
                    continue;
                }
                metric.tick(nowMs);
            }
            for (TunableBinding tunable : tunables) {
                if (tunable == null) {
                    continue;
                }
                tunable.tick();
            }
            for (ActionBinding action : actions) {
                if (action == null) {
                    continue;
                }
                action.tick();
            }
        }

        @Override
        public void close() {
            if (closed) {
                return;
            }
            closed = true;
            BINDINGS.remove(this);
        }
    }

    private interface ValueReader {
        Object read() throws Exception;
    }

    private static final class MetricBinding {
        private final String fullKey;
        private final int periodMs;
        private final double epsilon;
        private final ValueReader reader;
        private long lastPublishMs = 0L;
        private Object lastValue;

        private MetricBinding(String fullKey, int periodMs, double epsilon, ValueReader reader) {
            this.fullKey = fullKey;
            this.periodMs = periodMs;
            this.epsilon = epsilon;
            this.reader = reader;
        }

        void tick(long nowMs) {
            if ((nowMs - lastPublishMs) < periodMs) {
                return;
            }
            Object current;
            try {
                current = reader.read();
            } catch (Exception ex) {
                reportAnnotationWarning("metric read failed for " + fullKey + ": " + ex.getMessage());
                lastPublishMs = nowMs;
                return;
            }
            if (current == null) {
                lastPublishMs = nowMs;
                return;
            }
            if (lastValue != null && valuesEqual(lastValue, current, epsilon)) {
                lastPublishMs = nowMs;
                return;
            }
            putAtPath(fullKey, current);
            lastValue = snapshot(current);
            lastPublishMs = nowMs;
        }

        private static Object snapshot(Object value) {
            if (value == null) {
                return null;
            }
            if (value instanceof double[] arr) {
                return Arrays.copyOf(arr, arr.length);
            }
            if (value instanceof long[] arr) {
                return Arrays.copyOf(arr, arr.length);
            }
            if (value instanceof boolean[] arr) {
                return Arrays.copyOf(arr, arr.length);
            }
            if (value instanceof String[] arr) {
                return Arrays.copyOf(arr, arr.length);
            }
            return value;
        }
    }

    private static final class TunableBinding {
        private final String fullKey;
        private final Field field;
        private final Object target;
        private final double min;
        private final double max;
        private final double step;
        private final List<Method> callbacks;

        private TunableBinding(
                String fullKey,
                Field field,
                Object target,
                double min,
                double max,
                double step,
                List<Method> callbacks) {
            this.fullKey = fullKey;
            this.field = field;
            this.target = target;
            this.min = min;
            this.max = max;
            this.step = step;
            this.callbacks = callbacks != null ? callbacks : List.of();
        }

        void initialize() {
            Object defaultValue;
            try {
                defaultValue = field.get(target);
            } catch (IllegalAccessException ex) {
                reportAnnotationWarning("tunable init read failed for " + fullKey + ": " + ex.getMessage());
                return;
            }
            if (!containsPath(fullKey)) {
                putAtPath(fullKey, defaultValue);
            } else {
                Object converted = getAtPathDynamic(fullKey, field.getType(), defaultValue);
                Object clamped = clampValue(converted);
                setFieldValue(clamped, false);
            }
        }

        void tick() {
            Object current;
            try {
                current = field.get(target);
            } catch (IllegalAccessException ex) {
                reportAnnotationWarning("tunable read failed for " + fullKey + ": " + ex.getMessage());
                return;
            }
            Object incoming = getAtPathDynamic(fullKey, field.getType(), current);
            Object clamped = clampValue(incoming);
            if (valuesEqual(current, clamped, 0.0)) {
                return;
            }
            setFieldValue(clamped, true);
        }

        private Object clampValue(Object value) {
            if (!(value instanceof Number number)) {
                return value;
            }
            double numeric = number.doubleValue();
            if (Double.isFinite(min)) {
                numeric = Math.max(min, numeric);
            }
            if (Double.isFinite(max)) {
                numeric = Math.min(max, numeric);
            }
            if (step > 0.0 && Double.isFinite(step)) {
                numeric = Math.round(numeric / step) * step;
            }

            Class<?> type = field.getType();
            if (type == int.class || type == Integer.class) {
                return (int) Math.round(numeric);
            }
            if (type == long.class || type == Long.class) {
                return (long) Math.round(numeric);
            }
            if (type == float.class || type == Float.class) {
                return (float) numeric;
            }
            if (type == short.class || type == Short.class) {
                return (short) Math.round(numeric);
            }
            if (type == byte.class || type == Byte.class) {
                return (byte) Math.round(numeric);
            }
            return numeric;
        }

        private void setFieldValue(Object value, boolean invokeCallbacks) {
            try {
                field.set(target, value);
            } catch (IllegalAccessException ex) {
                reportAnnotationWarning("tunable write failed for " + fullKey + ": " + ex.getMessage());
                return;
            }
            if (!invokeCallbacks || callbacks.isEmpty()) {
                return;
            }
            for (Method callback : callbacks) {
                try {
                    if (callback.getParameterCount() == 0) {
                        callback.invoke(target);
                    } else {
                        Class<?> parameterType = callback.getParameterTypes()[0];
                        Object converted = convertRawValue(value, parameterType, null);
                        callback.invoke(target, converted);
                    }
                } catch (IllegalAccessException | InvocationTargetException ex) {
                    reportAnnotationWarning(
                            "onChange callback failed for " + fullKey + " on " + callback.getName() + ": " + ex.getMessage());
                } catch (Exception ex) {
                    reportAnnotationWarning(
                            "onChange conversion failed for " + fullKey + " on " + callback.getName() + ": " + ex.getMessage());
                }
            }
        }
    }

    private static final class ActionBinding {
        private final String fullKey;
        private final boolean risingEdgeOnly;
        private final Object target;
        private final Method method;
        private boolean lastValue;

        private ActionBinding(String fullKey, boolean risingEdgeOnly, Object target, Method method) {
            this.fullKey = fullKey;
            this.risingEdgeOnly = risingEdgeOnly;
            this.target = target;
            this.method = method;
            this.lastValue = getBooleanAtPath(fullKey, false);
        }

        void tick() {
            boolean current = getBooleanAtPath(fullKey, false);
            boolean trigger = risingEdgeOnly ? (current && !lastValue) : (current != lastValue);
            lastValue = current;
            if (!trigger) {
                return;
            }
            try {
                method.invoke(target);
            } catch (IllegalAccessException | InvocationTargetException ex) {
                reportAnnotationWarning("action failed for " + fullKey + " on " + method.getName() + ": " + ex.getMessage());
            }
        }
    }

    private static final class MapBuilder {
        private final java.util.Map<String, List<Method>> callbacks = new java.util.HashMap<>();

        void add(String key, Method callback) {
            callbacks.computeIfAbsent(key, __ -> new ArrayList<>()).add(callback);
        }

        List<Method> get(String key) {
            List<Method> result = callbacks.get(key);
            if (result == null || result.isEmpty()) {
                return List.of();
            }
            return List.copyOf(result);
        }
    }
}
