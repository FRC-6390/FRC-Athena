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
import java.util.EnumMap;
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

    private void refresh0() {
        if (!publishedConfig) {
            return;
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
                boolean details = t.detailsEntry.getBoolean(t.details.get());
                if (details != t.details.get()) {
                    t.details.set(details);
                    changed = true;
                }
                boolean adv = t.advancedEntry.getBoolean(t.advanced.get());
                if (adv != t.advanced.get()) {
                    t.advanced.set(adv);
                    changed = true;
                }
                boolean motors = t.motorsEntry.getBoolean(t.motors.get());
                if (motors != t.motors.get()) {
                    t.motors.set(motors);
                    changed = true;
                }
                boolean enc = t.encodersEntry.getBoolean(t.encoders.get());
                if (enc != t.encoders.get()) {
                    t.encoders.set(enc);
                    changed = true;
                }
                boolean status = t.statusEntry.getBoolean(t.status.get());
                if (status != t.status.get()) {
                    t.status.set(status);
                    changed = true;
                }
                boolean setpoints = t.setpointsEntry.getBoolean(t.setpoints.get());
                if (setpoints != t.setpoints.get()) {
                    t.setpoints.set(setpoints);
                    changed = true;
                }
                boolean outputs = t.outputsEntry.getBoolean(t.outputs.get());
                if (outputs != t.outputs.get()) {
                    t.outputs.set(outputs);
                    changed = true;
                }
                boolean sim = t.simulationEntry.getBoolean(t.simulation.get());
                if (sim != t.simulation.get()) {
                    t.simulation.set(sim);
                    changed = true;
                }
                boolean sysid = t.sysIdEntry.getBoolean(t.sysId.get());
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

    public final class MechanismToggles {
        private final String name;
        private final AtomicBoolean details;
        private final AtomicBoolean advanced;
        private final AtomicBoolean motors;
        private final AtomicBoolean encoders;
        private final AtomicBoolean status;
        private final AtomicBoolean setpoints;
        private final AtomicBoolean outputs;
        private final AtomicBoolean simulation;
        private final AtomicBoolean sysId;
        private final NetworkTableEntry detailsEntry;
        private final NetworkTableEntry advancedEntry;
        private final NetworkTableEntry motorsEntry;
        private final NetworkTableEntry encodersEntry;
        private final NetworkTableEntry statusEntry;
        private final NetworkTableEntry setpointsEntry;
        private final NetworkTableEntry outputsEntry;
        private final NetworkTableEntry simulationEntry;
        private final NetworkTableEntry sysIdEntry;

        private MechanismToggles(String name,
                                boolean detailsDefault,
                                boolean advancedDefault,
                                boolean motorsDefault,
                                boolean encodersDefault,
                                boolean statusDefault,
                                boolean setpointsDefault,
                                boolean outputsDefault,
                                boolean simulationDefault,
                                boolean sysIdDefault,
                                NetworkTableEntry detailsEntry,
                                NetworkTableEntry advancedEntry,
                                NetworkTableEntry motorsEntry,
                                NetworkTableEntry encodersEntry,
                                NetworkTableEntry statusEntry,
                                NetworkTableEntry setpointsEntry,
                                NetworkTableEntry outputsEntry,
                                NetworkTableEntry simulationEntry,
                                NetworkTableEntry sysIdEntry) {
            this.name = name;
            this.details = new AtomicBoolean(detailsDefault);
            this.advanced = new AtomicBoolean(advancedDefault);
            this.motors = new AtomicBoolean(motorsDefault);
            this.encoders = new AtomicBoolean(encodersDefault);
            this.status = new AtomicBoolean(statusDefault);
            this.setpoints = new AtomicBoolean(setpointsDefault);
            this.outputs = new AtomicBoolean(outputsDefault);
            this.simulation = new AtomicBoolean(simulationDefault);
            this.sysId = new AtomicBoolean(sysIdDefault);
            this.detailsEntry = detailsEntry;
            this.advancedEntry = advancedEntry;
            this.motorsEntry = motorsEntry;
            this.encodersEntry = encodersEntry;
            this.statusEntry = statusEntry;
            this.setpointsEntry = setpointsEntry;
            this.outputsEntry = outputsEntry;
            this.simulationEntry = simulationEntry;
            this.sysIdEntry = sysIdEntry;
        }

        public boolean detailsEnabled() {
            return details.get();
        }

        public boolean advancedEnabled() {
            return advanced.get();
        }

        public boolean motorsEnabled() {
            return motors.get();
        }

        public boolean encodersEnabled() {
            return encoders.get();
        }

        public boolean statusEnabled() {
            return status.get();
        }

        public boolean setpointsEnabled() {
            return setpoints.get();
        }

        public boolean outputsEnabled() {
            return outputs.get();
        }

        public boolean simulationEnabled() {
            return simulation.get();
        }

        public boolean sysIdEnabled() {
            return sysId.get();
        }

        public MechanismToggles details(boolean enabled) {
            detailsEntry.setBoolean(enabled);
            details.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles advanced(boolean enabled) {
            advancedEntry.setBoolean(enabled);
            advanced.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles motors(boolean enabled) {
            motorsEntry.setBoolean(enabled);
            motors.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles encoders(boolean enabled) {
            encodersEntry.setBoolean(enabled);
            encoders.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles status(boolean enabled) {
            statusEntry.setBoolean(enabled);
            status.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles setpoints(boolean enabled) {
            setpointsEntry.setBoolean(enabled);
            setpoints.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles outputs(boolean enabled) {
            outputsEntry.setBoolean(enabled);
            outputs.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles simulation(boolean enabled) {
            simulationEntry.setBoolean(enabled);
            simulation.set(enabled);
            revision.incrementAndGet();
            return this;
        }

        public MechanismToggles sysId(boolean enabled) {
            sysIdEntry.setBoolean(enabled);
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
        boolean advancedDefault = false;
        boolean motorsDefault = true;
        boolean encodersDefault = true;
        boolean statusDefault = true;
        boolean setpointsDefault = true;
        boolean outputsDefault = true;
        boolean simulationDefault = false;
        boolean sysIdDefault = false;

        Node cfg = mechanismNode.child("NetworkTableConfig");
        NetworkTableEntry detailsEntry = cfg.entry("Details");
        NetworkTableEntry advancedEntry = cfg.entry("Advanced");
        NetworkTableEntry motorsEntry = cfg.entry("Motors");
        NetworkTableEntry encodersEntry = cfg.entry("Encoders");
        NetworkTableEntry statusEntry = cfg.entry("Status");
        NetworkTableEntry setpointsEntry = cfg.entry("Setpoints");
        NetworkTableEntry outputsEntry = cfg.entry("Outputs");
        NetworkTableEntry simulationEntry = cfg.entry("Simulation");
        NetworkTableEntry sysIdEntry = cfg.entry("SysId");

        initIfAbsent(detailsEntry, detailsDefault);
        initIfAbsent(advancedEntry, advancedDefault);
        initIfAbsent(motorsEntry, motorsDefault);
        initIfAbsent(encodersEntry, encodersDefault);
        initIfAbsent(statusEntry, statusDefault);
        initIfAbsent(setpointsEntry, setpointsDefault);
        initIfAbsent(outputsEntry, outputsDefault);
        initIfAbsent(simulationEntry, simulationDefault);
        initIfAbsent(sysIdEntry, sysIdDefault);

        return new MechanismToggles(mechanismNode.path(),
                detailsEntry.getBoolean(detailsDefault),
                advancedEntry.getBoolean(advancedDefault),
                motorsEntry.getBoolean(motorsDefault),
                encodersEntry.getBoolean(encodersDefault),
                statusEntry.getBoolean(statusDefault),
                setpointsEntry.getBoolean(setpointsDefault),
                outputsEntry.getBoolean(outputsDefault),
                simulationEntry.getBoolean(simulationDefault),
                sysIdEntry.getBoolean(sysIdDefault),
                detailsEntry,
                advancedEntry,
                motorsEntry,
                encodersEntry,
                statusEntry,
                setpointsEntry,
                outputsEntry,
                simulationEntry,
                sysIdEntry);
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

    /**
     * Root node for Athena publishing (default: {@code /Athena}).
     */
    public Node root() {
        return new Node("/" + ROOT);
    }

    public final class Node {
        private final String path; // absolute, no trailing slash

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
            String normalized = childPath.trim().replace('\\', '/').replaceAll("/+", "/");
            while (normalized.startsWith("/")) {
                normalized = normalized.substring(1);
            }
            while (normalized.endsWith("/")) {
                normalized = normalized.substring(0, normalized.length() - 1);
            }
            if (normalized.isEmpty()) {
                return this;
            }
            return new Node(path + "/" + normalized);
        }

        public void putBoolean(String key, boolean value) {
            if (!cachedPublishingEnabled) {
                return;
            }
            String full = resolveKey(key);
            BooleanPublisher pub = booleanPublishers.computeIfAbsent(full, p -> {
                BooleanTopic topic = nt.getBooleanTopic(p);
                return topic.publish();
            });
            pub.set(value);
        }

        public void putDouble(String key, double value) {
            if (!cachedPublishingEnabled) {
                return;
            }
            String full = resolveKey(key);
            DoublePublisher pub = doublePublishers.computeIfAbsent(full, p -> {
                DoubleTopic topic = nt.getDoubleTopic(p);
                return topic.publish();
            });
            pub.set(value);
        }

        public void putString(String key, String value) {
            if (!cachedPublishingEnabled) {
                return;
            }
            String full = resolveKey(key);
            StringPublisher pub = stringPublishers.computeIfAbsent(full, p -> {
                StringTopic topic = nt.getStringTopic(p);
                return topic.publish();
            });
            pub.set(value != null ? value : "");
        }

        public NetworkTable table() {
            // Strip leading slash for NetworkTableInstance.getTable
            String rel = path.startsWith("/") ? path.substring(1) : path;
            return nt.getTable(rel);
        }

        public NetworkTableEntry entry(String key) {
            String k = key != null ? key.trim() : "";
            if (k.isEmpty()) {
                throw new IllegalArgumentException("key must not be blank");
            }
            return table().getEntry(k);
        }

        private String resolveKey(String key) {
            String k = key != null ? key.trim() : "";
            if (k.isEmpty()) {
                throw new IllegalArgumentException("key must not be blank");
            }
            if (k.startsWith("/")) {
                return k;
            }
            return path + "/" + k;
        }
    }
}
