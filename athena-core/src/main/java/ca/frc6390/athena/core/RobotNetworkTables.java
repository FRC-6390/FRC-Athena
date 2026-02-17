package ca.frc6390.athena.core;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Centralized NetworkTables publishing + runtime gating for Athena.
 *
 * <p>This intentionally does not use WPILib Shuffleboard container APIs. We publish raw topics with
 * standardized paths so dashboards can consume them without Athena being "limited by Shuffleboard".</p>
 */
public final class RobotNetworkTables {
    public static final String ROOT = "Athena";
    private final NetworkTableInstance nt;

    public enum Flag {
        // Auto publishing (RobotCore)
        AUTO_PUBLISH_CORE(true),
        AUTO_PUBLISH_MECHANISMS(true),

        // Drivetrain
        DRIVETRAIN_SPEED_WIDGETS(false),

        // Hardware detail fans (very expensive at scale)
        HW_MOTOR_TUNING_WIDGETS(false),
        HW_ENCODER_TUNING_WIDGETS(false),
        HW_IMU_TUNING_WIDGETS(false),
        MOTOR_GROUP_PER_MOTOR(false),
        ENCODER_GROUP_PER_ENCODER(false),

        SWERVE_MODULE_DEBUG(false),

        // Localization / vision
        LOCALIZATION_HEALTH_WIDGETS(true),
        LOCALIZATION_POSE_TOPICS(true),
        LOCALIZATION_FIELD_WIDGET(false),
        VISION_CAMERA_WIDGETS(false),
        ;

        private final boolean defaultValue;

        Flag(boolean defaultValue) {
            this.defaultValue = defaultValue;
        }

        public boolean defaultValue() {
            return defaultValue;
        }
    }

    private final AtomicLong revision = new AtomicLong(0);

    private final EnumMap<Flag, AtomicBoolean> cached = new EnumMap<>(Flag.class);
    private final EnumMap<Flag, NetworkTableEntry> flagEntries = new EnumMap<>(Flag.class);
    private final EnumMap<Flag, Boolean> desiredFlagDefaults = new EnumMap<>(Flag.class);

    private final Object mechLock = new Object();
    private final LinkedHashMap<String, MechanismToggles> mechanismToggles = new LinkedHashMap<>();

    private volatile boolean publishedConfig = false;
    private volatile boolean cachedPublishingEnabled = true;
    private volatile double cachedDefaultPeriodSeconds = 1.0;
    private NetworkTable configTable;
    private NetworkTable flagsTable;
    private NetworkTableEntry publishingEnabledEntry;
    private NetworkTableEntry defaultPeriodEntry;

    // Raw topic publishers for data output (not config toggles).
    private final ConcurrentHashMap<String, BooleanPublisher> booleanPublishers = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, DoublePublisher> doublePublishers = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, StringPublisher> stringPublishers = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, Boolean> lastBooleanValues = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, Long> lastDoubleBits = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, String> lastStringValues = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, Long> topicLastSeenCycle = new ConcurrentHashMap<>();
    private static final long TOPIC_STALE_CYCLES = 750L;
    private static final int TOPIC_PRUNE_SCAN_BUDGET_PER_CYCLE = 64;
    private final AtomicLong publishCycle = new AtomicLong(0L);
    private volatile Iterator<Map.Entry<String, Long>> topicPruneIterator;
    private static final double CONFIG_REFRESH_PERIOD_SECONDS = 0.1;
    private double lastConfigRefreshSeconds = Double.NaN;
    private final Node rootNode = new Node("/" + ROOT);

    public RobotNetworkTables() {
        this(NetworkTableInstance.getDefault());
    }

    RobotNetworkTables(NetworkTableInstance nt) {
        this.nt = Objects.requireNonNull(nt, "nt");
        for (Flag flag : Flag.values()) {
            cached.put(flag, new AtomicBoolean(flag.defaultValue()));
        }
    }

    public long revision() {
        return revision.get();
    }

    public boolean enabled(Flag flag) {
        AtomicBoolean value = cached.get(flag);
        return value != null ? value.get() : flag.defaultValue();
    }

    public boolean isPublishingEnabled() {
        return cachedPublishingEnabled;
    }

    public void setPublishingEnabled(boolean enabled) {
        cachedPublishingEnabled = enabled;
        if (publishedConfig && publishingEnabledEntry != null) {
            publishingEnabledEntry.setBoolean(enabled);
        }
        revision.incrementAndGet();
    }

    public double getDefaultPeriodSeconds() {
        return cachedDefaultPeriodSeconds;
    }

    public void setDefaultPeriodSeconds(double seconds) {
        double period = sanitizePeriod(seconds);
        cachedDefaultPeriodSeconds = period;
        if (publishedConfig && defaultPeriodEntry != null) {
            defaultPeriodEntry.setDouble(period);
        }
        revision.incrementAndGet();
    }

    /**
     * Creates the config topics under {@code Athena/NetworkTableConfig}. Safe to call multiple times.
     *
     * <p>These are plain NT entries so Elastic/Glass/custom tools can toggle them at runtime.</p>
     */
    public void publishConfig() {
        publishConfig0();
    }

    private void publishConfig0() {
        if (publishedConfig) {
            return;
        }
        publishedConfig = true;

        configTable = nt.getTable(ROOT).getSubTable("NetworkTableConfig");
        flagsTable = configTable.getSubTable("Flags");

        publishingEnabledEntry = configTable.getEntry("EnablePublishing");
        defaultPeriodEntry = configTable.getEntry("DefaultPeriodSec");

        initIfAbsent(publishingEnabledEntry, true);
        initIfAbsent(defaultPeriodEntry, cachedDefaultPeriodSeconds);

        cachedPublishingEnabled = publishingEnabledEntry.getBoolean(true);
        cachedDefaultPeriodSeconds = sanitizePeriod(defaultPeriodEntry.getDouble(cachedDefaultPeriodSeconds));

        for (Flag flag : Flag.values()) {
            NetworkTableEntry entry = flagsTable.getEntry(flag.name());
            flagEntries.put(flag, entry);
            boolean defaultValue = desiredFlagDefaults.containsKey(flag) ? desiredFlagDefaults.get(flag) : flag.defaultValue();
            if (desiredFlagDefaults.containsKey(flag)) {
                // Code-defined startup defaults should win over any previously persisted value.
                entry.setBoolean(defaultValue);
            } else {
                initIfAbsent(entry, defaultValue);
            }
            cached.get(flag).set(entry.getBoolean(defaultValue));
        }

        synchronized (mechLock) {
            // Mechanism configs are published under each mechanism's own NetworkTables path.
        }
    }

    public void refresh() {
        refresh0();
    }

    /**
     * Advances the publish cycle and periodically prunes stale topic caches/publishers.
     */
    public void beginPublishCycle() {
        long cycle = publishCycle.incrementAndGet();
        pruneStaleTopics(cycle);
    }

    private void refresh0() {
        if (!publishedConfig) {
            return;
        }
        double nowSeconds = RobotTime.nowSeconds();
        if (!Double.isFinite(nowSeconds)) {
            nowSeconds = Timer.getFPGATimestamp();
        }
        if (Double.isFinite(nowSeconds) && Double.isFinite(lastConfigRefreshSeconds)) {
            double elapsed = nowSeconds - lastConfigRefreshSeconds;
            if (elapsed >= 0.0 && elapsed < CONFIG_REFRESH_PERIOD_SECONDS) {
                return;
            }
        }
        if (Double.isFinite(nowSeconds)) {
            lastConfigRefreshSeconds = nowSeconds;
        }
        boolean changed = false;

        boolean enabled = publishingEnabledEntry.getBoolean(cachedPublishingEnabled);
        if (enabled != cachedPublishingEnabled) {
            cachedPublishingEnabled = enabled;
            changed = true;
        }

        double period = sanitizePeriod(defaultPeriodEntry.getDouble(cachedDefaultPeriodSeconds));
        if (period != cachedDefaultPeriodSeconds) {
            cachedDefaultPeriodSeconds = period;
            changed = true;
        }

        for (Map.Entry<Flag, NetworkTableEntry> entry : flagEntries.entrySet()) {
            Flag flag = entry.getKey();
            NetworkTableEntry e = entry.getValue();
            if (flag == null || e == null) {
                continue;
            }
            boolean value = e.getBoolean(flag.defaultValue());
            AtomicBoolean target = cached.get(flag);
            if (target != null && value != target.get()) {
                target.set(value);
                changed = true;
            }
        }

        synchronized (mechLock) {
            for (MechanismToggles t : mechanismToggles.values()) {
                if (t == null) {
                    continue;
                }
                boolean details = readToggleEntry(t.detailsEntry, t.details.get());
                if (details != t.details.get()) {
                    t.details.set(details);
                    changed = true;
                }
                boolean motors = readToggleEntry(t.motorsEntry, t.motors.get());
                if (motors != t.motors.get()) {
                    t.motors.set(motors);
                    changed = true;
                }
                boolean enc = readToggleEntry(t.encoderEntry, t.encoder.get());
                if (enc != t.encoder.get()) {
                    t.encoder.set(enc);
                    changed = true;
                }
                boolean constraints = readToggleEntry(t.constraintsEntry, t.constraints.get());
                if (constraints != t.constraints.get()) {
                    t.constraints.set(constraints);
                    changed = true;
                }
                boolean sensors = readToggleEntry(t.sensorsEntry, t.sensors.get());
                if (sensors != t.sensors.get()) {
                    t.sensors.set(sensors);
                    changed = true;
                }
                boolean control = readToggleEntry(t.controlEntry, t.control.get());
                if (control != t.control.get()) {
                    t.control.set(control);
                    changed = true;
                }
                boolean inputs = readToggleEntry(t.inputsEntry, t.inputs.get());
                if (inputs != t.inputs.get()) {
                    t.inputs.set(inputs);
                    changed = true;
                }
                boolean sim = readToggleEntry(t.simulationEntry, t.simulation.get());
                if (sim != t.simulation.get()) {
                    t.simulation.set(sim);
                    changed = true;
                }
                boolean sysid = readToggleEntry(t.sysIdEntry, t.sysId.get());
                if (sysid != t.sysId.get()) {
                    t.sysId.set(sysid);
                    changed = true;
                }
            }
        }

        if (changed) {
            revision.incrementAndGet();
        }
    }

    public RobotNetworkTables setFlag(Flag flag, boolean enabled) {
        Objects.requireNonNull(flag, "flag");
        desiredFlagDefaults.put(flag, enabled);
        if (publishedConfig) {
            NetworkTableEntry entry = flagEntries.get(flag);
            if (entry != null) {
                entry.setBoolean(enabled);
            }
        }
        cached.get(flag).set(enabled);
        revision.incrementAndGet();
        return this;
    }

    public RobotNetworkTables enable(Flag flag) {
        return setFlag(flag, true);
    }

    public RobotNetworkTables disable(Flag flag) {
        return setFlag(flag, false);
    }

    /**
     * Per-mechanism publishing toggles located under the mechanism's own NT tree:
     * {@code Athena/Mechanisms/.../<MechName>/NetworkTableConfig/...}.
     */
    public MechanismToggles mechanismConfig(Node mechanismNode) {
        Objects.requireNonNull(mechanismNode, "mechanismNode");
        ensureTables();
        String id = mechanismNode.path();
        synchronized (mechLock) {
            MechanismToggles t = mechanismToggles.get(id);
            if (t != null) {
                return t;
            }
            MechanismToggles created = createMechanismToggles(mechanismNode);
            mechanismToggles.put(id, created);
            return created;
        }
    }

    /**
     * Superstructure-scoped publishing toggles under:
     * {@code Athena/Mechanisms/.../<SuperstructureName>/NetworkTableConfig/...}.
     *
     * <p>Superstructures currently consume only {@code Details}, {@code Control}, and {@code Inputs},
     * so only those keys are created for cleaner dashboards.</p>
     */
    public MechanismToggles superstructureConfig(Node superstructureNode) {
        Objects.requireNonNull(superstructureNode, "superstructureNode");
        ensureTables();
        String id = superstructureNode.path();
        synchronized (mechLock) {
            MechanismToggles t = mechanismToggles.get(id);
            if (t != null) {
                return t;
            }
            MechanismToggles created = createSuperstructureToggles(superstructureNode);
            mechanismToggles.put(id, created);
            return created;
        }
    }

    public final class MechanismToggles {
        private final String name;
        private final AtomicBoolean details;
        private final AtomicBoolean motors;
        private final AtomicBoolean encoder;
        private final AtomicBoolean constraints;
        private final AtomicBoolean sensors;
        private final AtomicBoolean control;
        private final AtomicBoolean inputs;
        private final AtomicBoolean simulation;
        private final AtomicBoolean sysId;
        private final NetworkTableEntry detailsEntry;
        private final NetworkTableEntry motorsEntry;
        private final NetworkTableEntry encoderEntry;
        private final NetworkTableEntry constraintsEntry;
        private final NetworkTableEntry sensorsEntry;
        private final NetworkTableEntry controlEntry;
        private final NetworkTableEntry inputsEntry;
        private final NetworkTableEntry simulationEntry;
        private final NetworkTableEntry sysIdEntry;

        private MechanismToggles(String name,
                                boolean detailsDefault,
                                boolean motorsDefault,
                                boolean encoderDefault,
                                boolean constraintsDefault,
                                boolean sensorsDefault,
                                boolean controlDefault,
                                boolean inputsDefault,
                                boolean simulationDefault,
                                boolean sysIdDefault,
                                NetworkTableEntry detailsEntry,
                                NetworkTableEntry motorsEntry,
                                NetworkTableEntry encoderEntry,
                                NetworkTableEntry constraintsEntry,
                                NetworkTableEntry sensorsEntry,
                                NetworkTableEntry controlEntry,
                                NetworkTableEntry inputsEntry,
                                NetworkTableEntry simulationEntry,
                                NetworkTableEntry sysIdEntry) {
            this.name = name;
            this.details = new AtomicBoolean(detailsDefault);
            this.motors = new AtomicBoolean(motorsDefault);
            this.encoder = new AtomicBoolean(encoderDefault);
            this.constraints = new AtomicBoolean(constraintsDefault);
            this.sensors = new AtomicBoolean(sensorsDefault);
            this.control = new AtomicBoolean(controlDefault);
            this.inputs = new AtomicBoolean(inputsDefault);
            this.simulation = new AtomicBoolean(simulationDefault);
            this.sysId = new AtomicBoolean(sysIdDefault);
            this.detailsEntry = detailsEntry;
            this.motorsEntry = motorsEntry;
            this.encoderEntry = encoderEntry;
            this.constraintsEntry = constraintsEntry;
            this.sensorsEntry = sensorsEntry;
            this.controlEntry = controlEntry;
            this.inputsEntry = inputsEntry;
            this.simulationEntry = simulationEntry;
            this.sysIdEntry = sysIdEntry;
        }

        public boolean detailsEnabled() {
            return details.get();
        }

        public boolean motorsEnabled() {
            return motors.get();
        }

        public boolean encoderEnabled() {
            return encoder.get();
        }

        public boolean constraintsEnabled() {
            return constraints.get();
        }

        public boolean sensorsEnabled() {
            return sensors.get();
        }

        public boolean controlEnabled() {
            return control.get();
        }

        public boolean inputsEnabled() {
            return inputs.get();
        }

        public boolean simulationEnabled() {
            return simulation.get();
        }

        public boolean sysIdEnabled() {
            return sysId.get();
        }

        public MechanismToggles details(boolean enabled) {
            if (detailsEntry != null) {
                detailsEntry.setBoolean(enabled);
            }
            details.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles motors(boolean enabled) {
            if (motorsEntry != null) {
                motorsEntry.setBoolean(enabled);
            }
            motors.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles encoder(boolean enabled) {
            if (encoderEntry != null) {
                encoderEntry.setBoolean(enabled);
            }
            encoder.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles constraints(boolean enabled) {
            if (constraintsEntry != null) {
                constraintsEntry.setBoolean(enabled);
            }
            constraints.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles sensors(boolean enabled) {
            if (sensorsEntry != null) {
                sensorsEntry.setBoolean(enabled);
            }
            sensors.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles control(boolean enabled) {
            if (controlEntry != null) {
                controlEntry.setBoolean(enabled);
            }
            control.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles inputs(boolean enabled) {
            if (inputsEntry != null) {
                inputsEntry.setBoolean(enabled);
            }
            inputs.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles simulation(boolean enabled) {
            if (simulationEntry != null) {
                simulationEntry.setBoolean(enabled);
            }
            simulation.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles sysId(boolean enabled) {
            if (sysIdEntry != null) {
                sysIdEntry.setBoolean(enabled);
            }
            sysId.set(enabled);
            revision.incrementAndGet();
            return this;
        }

    }

    private MechanismToggles createMechanismToggles(Node mechanismNode) {
        ensureTables();

        // Defaults are intentionally conservative: "details" is off until explicitly requested,
        // but the internal sections default on so when details is enabled you get the useful stuff.
        boolean detailsDefault = false;
        boolean motorsDefault = true;
        boolean encoderDefault = true;
        boolean constraintsDefault = true;
        boolean sensorsDefault = true;
        boolean controlDefault = true;
        boolean inputsDefault = false;
        boolean simulationDefault = false;
        boolean sysIdDefault = false;

        Node cfg = mechanismNode.child("NetworkTableConfig");
        NetworkTableEntry detailsEntry = cfg.entry("Details");
        NetworkTableEntry motorsEntry = cfg.entry("Motors");
        NetworkTableEntry encoderEntry = cfg.entry("Encoder");
        NetworkTableEntry constraintsEntry = cfg.entry("Constraints");
        NetworkTableEntry sensorsEntry = cfg.entry("Sensors");
        NetworkTableEntry controlEntry = cfg.entry("Control");
        NetworkTableEntry inputsEntry = cfg.entry("Inputs");
        NetworkTableEntry simulationEntry = cfg.entry("Simulation");
        NetworkTableEntry sysIdEntry = cfg.entry("SysId");

        initIfAbsent(detailsEntry, detailsDefault);
        initIfAbsent(motorsEntry, motorsDefault);
        initIfAbsent(encoderEntry, encoderDefault);
        initIfAbsent(constraintsEntry, constraintsDefault);
        initIfAbsent(sensorsEntry, sensorsDefault);
        initIfAbsent(controlEntry, controlDefault);
        initIfAbsent(inputsEntry, inputsDefault);
        initIfAbsent(simulationEntry, simulationDefault);
        initIfAbsent(sysIdEntry, sysIdDefault);

        return new MechanismToggles(mechanismNode.path(),
                detailsEntry.getBoolean(detailsDefault),
                motorsEntry.getBoolean(motorsDefault),
                encoderEntry.getBoolean(encoderDefault),
                constraintsEntry.getBoolean(constraintsDefault),
                sensorsEntry.getBoolean(sensorsDefault),
                controlEntry.getBoolean(controlDefault),
                inputsEntry.getBoolean(inputsDefault),
                simulationEntry.getBoolean(simulationDefault),
                sysIdEntry.getBoolean(sysIdDefault),
                detailsEntry,
                motorsEntry,
                encoderEntry,
                constraintsEntry,
                sensorsEntry,
                controlEntry,
                inputsEntry,
                simulationEntry,
                sysIdEntry);
    }

    private MechanismToggles createSuperstructureToggles(Node superstructureNode) {
        ensureTables();

        boolean detailsDefault = false;
        boolean controlDefault = true;
        boolean inputsDefault = false;

        Node cfg = superstructureNode.child("NetworkTableConfig");
        NetworkTableEntry detailsEntry = cfg.entry("Details");
        NetworkTableEntry controlEntry = cfg.entry("Control");
        NetworkTableEntry inputsEntry = cfg.entry("Inputs");

        initIfAbsent(detailsEntry, detailsDefault);
        initIfAbsent(controlEntry, controlDefault);
        initIfAbsent(inputsEntry, inputsDefault);

        return new MechanismToggles(superstructureNode.path(),
                detailsEntry.getBoolean(detailsDefault),
                false,
                false,
                false,
                false,
                controlEntry.getBoolean(controlDefault),
                inputsEntry.getBoolean(inputsDefault),
                false,
                false,
                detailsEntry,
                null,
                null,
                null,
                null,
                controlEntry,
                inputsEntry,
                null,
                null);
    }

    private void ensureTables() {
        if (!publishedConfig) {
            publishConfig0();
        }
    }

    private static double sanitizePeriod(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0.0) {
            return 1.0;
        }
        return seconds;
    }

    private static void initIfAbsent(NetworkTableEntry entry, boolean defaultValue) {
        if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
            entry.setBoolean(defaultValue);
        }
    }

    private static void initIfAbsent(NetworkTableEntry entry, double defaultValue) {
        if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
            entry.setDouble(defaultValue);
        }
    }

    private static boolean readToggleEntry(NetworkTableEntry entry, boolean fallback) {
        if (entry == null) {
            return fallback;
        }
        return entry.getBoolean(fallback);
    }

    /**
     * Root node for Athena publishing (default: {@code /Athena}).
     */
    public Node root() {
        return rootNode;
    }

    public int topicCacheSize() {
        return topicLastSeenCycle.size();
    }

    public int booleanPublisherCount() {
        return booleanPublishers.size();
    }

    public int doublePublisherCount() {
        return doublePublishers.size();
    }

    public int stringPublisherCount() {
        return stringPublishers.size();
    }

    public int mechanismToggleCount() {
        synchronized (mechLock) {
            return mechanismToggles.size();
        }
    }

    public final class Node {
        private final String path; // absolute, no trailing slash
        private final ConcurrentHashMap<String, Node> childNodes = new ConcurrentHashMap<>();
        private final ConcurrentHashMap<String, String> relativeResolvedKeys = new ConcurrentHashMap<>();
        private volatile NetworkTable cachedTable;

        private Node(String path) {
            this.path = path;
        }

        public String path() {
            return path;
        }

        public RobotNetworkTables robot() {
            return RobotNetworkTables.this;
        }

        public Node child(String childPath) {
            if (childPath == null || childPath.isBlank()) {
                return this;
            }
            String normalized = normalizeChildPath(childPath);
            if (normalized.isEmpty()) {
                return this;
            }
            Node cached = childNodes.get(normalized);
            if (cached != null) {
                return cached;
            }
            Node created = new Node(path + "/" + normalized);
            Node raced = childNodes.putIfAbsent(normalized, created);
            return raced != null ? raced : created;
        }

        public void putBoolean(String key, boolean value) {
            String full = resolveKey(key);
            markTopicSeen(full);
            if (!cachedPublishingEnabled) {
                return;
            }
            Boolean previous = lastBooleanValues.put(full, value);
            if (previous != null && previous.booleanValue() == value) {
                return;
            }
            BooleanPublisher pub = booleanPublishers.computeIfAbsent(full, p -> {
                BooleanTopic topic = nt.getBooleanTopic(p);
                return topic.publish();
            });
            pub.set(value);
        }

        public void putDouble(String key, double value) {
            String full = resolveKey(key);
            markTopicSeen(full);
            if (!cachedPublishingEnabled) {
                return;
            }
            long bits = Double.doubleToLongBits(value);
            Long previousBits = lastDoubleBits.put(full, bits);
            if (previousBits != null && previousBits.longValue() == bits) {
                return;
            }
            DoublePublisher pub = doublePublishers.computeIfAbsent(full, p -> {
                DoubleTopic topic = nt.getDoubleTopic(p);
                return topic.publish();
            });
            pub.set(value);
        }

        public void putString(String key, String value) {
            String full = resolveKey(key);
            markTopicSeen(full);
            if (!cachedPublishingEnabled) {
                return;
            }
            String safeValue = value != null ? value : "";
            String previous = lastStringValues.put(full, safeValue);
            if (safeValue.equals(previous)) {
                return;
            }
            StringPublisher pub = stringPublishers.computeIfAbsent(full, p -> {
                StringTopic topic = nt.getStringTopic(p);
                return topic.publish();
            });
            pub.set(safeValue);
        }

        public NetworkTable table() {
            NetworkTable table = cachedTable;
            if (table != null) {
                return table;
            }
            // Strip leading slash for NetworkTableInstance.getTable
            String rel = path.startsWith("/") ? path.substring(1) : path;
            table = nt.getTable(rel);
            cachedTable = table;
            return table;
        }

        public NetworkTableEntry entry(String key) {
            return table().getEntry(normalizeEntryKey(key));
        }

        private String resolveKey(String key) {
            String normalized = normalizeEntryKey(key);
            if (normalized.startsWith("/")) {
                return normalized;
            }
            String cached = relativeResolvedKeys.get(normalized);
            if (cached != null) {
                return cached;
            }
            String resolved = path + "/" + normalized;
            String raced = relativeResolvedKeys.putIfAbsent(normalized, resolved);
            return raced != null ? raced : resolved;
        }

        private static String normalizeChildPath(String childPath) {
            String raw = childPath != null ? childPath.trim() : "";
            if (raw.isEmpty()) {
                return "";
            }
            int start = 0;
            int end = raw.length();
            while (start < end) {
                char c = raw.charAt(start);
                if (c == '/' || c == '\\') {
                    start++;
                    continue;
                }
                break;
            }
            while (end > start) {
                char c = raw.charAt(end - 1);
                if (c == '/' || c == '\\') {
                    end--;
                    continue;
                }
                break;
            }
            if (start >= end) {
                return "";
            }
            StringBuilder normalizedBuilder = new StringBuilder(end - start);
            boolean lastSlash = false;
            for (int i = start; i < end; i++) {
                char c = raw.charAt(i);
                if (c == '/' || c == '\\') {
                    if (lastSlash) {
                        continue;
                    }
                    normalizedBuilder.append('/');
                    lastSlash = true;
                } else {
                    normalizedBuilder.append(c);
                    lastSlash = false;
                }
            }
            return normalizedBuilder.toString();
        }

        private static String normalizeEntryKey(String key) {
            if (key == null) {
                throw new IllegalArgumentException("key must not be blank");
            }
            int start = 0;
            int end = key.length();
            while (start < end && Character.isWhitespace(key.charAt(start))) {
                start++;
            }
            while (end > start && Character.isWhitespace(key.charAt(end - 1))) {
                end--;
            }
            if (start >= end) {
                throw new IllegalArgumentException("key must not be blank");
            }
            if (start == 0 && end == key.length()) {
                return key;
            }
            return key.substring(start, end);
        }
    }

    private void markTopicSeen(String key) {
        if (key == null || key.isBlank()) {
            return;
        }
        topicLastSeenCycle.put(key, publishCycle.get());
    }

    private void pruneStaleTopics(long currentCycle) {
        long cutoff = currentCycle - TOPIC_STALE_CYCLES;
        if (cutoff <= 0) {
            return;
        }
        Iterator<Map.Entry<String, Long>> iterator = topicPruneIterator;
        if (iterator == null || !iterator.hasNext()) {
            iterator = topicLastSeenCycle.entrySet().iterator();
            topicPruneIterator = iterator;
        }

        int processed = 0;
        while (iterator != null && iterator.hasNext() && processed < TOPIC_PRUNE_SCAN_BUDGET_PER_CYCLE) {
            Map.Entry<String, Long> entry = iterator.next();
            processed++;
            if (entry == null) {
                continue;
            }
            String key = entry.getKey();
            if (key == null) {
                continue;
            }
            Long seen = entry.getValue();
            if (seen != null && seen.longValue() >= cutoff) {
                continue;
            }
            if (seen != null && !topicLastSeenCycle.remove(key, seen)) {
                continue;
            }
            if (seen == null) {
                topicLastSeenCycle.remove(key);
            }
            lastBooleanValues.remove(key);
            lastDoubleBits.remove(key);
            lastStringValues.remove(key);

            BooleanPublisher booleanPublisher = booleanPublishers.remove(key);
            if (booleanPublisher != null) {
                booleanPublisher.close();
            }
            DoublePublisher doublePublisher = doublePublishers.remove(key);
            if (doublePublisher != null) {
                doublePublisher.close();
            }
            StringPublisher stringPublisher = stringPublishers.remove(key);
            if (stringPublisher != null) {
                stringPublisher.close();
            }
        }

        if (iterator == null || !iterator.hasNext()) {
            topicPruneIterator = null;
        }
    }
}
