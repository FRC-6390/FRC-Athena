package ca.frc6390.athena.dashboard;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import ca.frc6390.athena.core.RobotSendableSystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Central, standardized Shuffleboard publishing controls.
 *
 * <p>Publish one "Athena/Config" tab, then gate expensive sections everywhere else using these flags.
 * This is intentionally simple: values live in NetworkTables via the Shuffleboard widgets, but are
 * cached locally and refreshed at a low rate to keep per-loop overhead negligible.</p>
 */
public final class ShuffleboardControls {
    private ShuffleboardControls() {}

    public enum Flag {
        // Auto publishing (RobotCore)
        AUTO_PUBLISH_CORE(true),
        AUTO_PUBLISH_MECHANISMS(true),

        // Mechanism sections
        MECHANISM_MOTORS(true),
        MECHANISM_ENCODERS(true),
        MECHANISM_STATUS(true),
        MECHANISM_SETPOINTS(true),
        MECHANISM_OUTPUTS(true),
        MECHANISM_VISUALIZATION(false),
        MECHANISM_SIMULATION(false),
        MECHANISM_CONFIG(false),
        MECHANISM_LIMIT_SWITCHES(false),
        MECHANISM_COMMANDS(false),
        MECHANISM_SYSID(false),
        MECHANISM_CONTROLLERS(false),

        // Hardware detail fans (very expensive at scale)
        MOTOR_GROUP_PER_MOTOR(false),
        ENCODER_GROUP_PER_ENCODER(false),

        // Other heavy debug sections
        STATE_MACHINE_STATES_LIST(false),
        SWERVE_MODULE_DEBUG(false),

        // Localization / vision
        LOCALIZATION_HEALTH_WIDGETS(true),
        LOCALIZATION_BACKEND_WIDGETS(false),
        LOCALIZATION_POSE_TOPICS(true),
        VISION_CAMERA_WIDGETS(false),
        VISION_FIELD_WIDGET(true),

        // Telemetry widgets on Shuffleboard (NetworkTables telemetry can still be enabled separately)
        TELEMETRY_SHUFFLEBOARD_WIDGETS(false);

        private final boolean defaultValue;

        Flag(boolean defaultValue) {
            this.defaultValue = defaultValue;
        }

        public boolean defaultValue() {
            return defaultValue;
        }
    }

    private static final EnumMap<Flag, AtomicBoolean> cached = new EnumMap<>(Flag.class);
    private static final EnumMap<Flag, GenericEntry> entries = new EnumMap<>(Flag.class);
    private static volatile boolean published = false;
    private static volatile ShuffleboardTab configTab;
    private static double lastRefreshSeconds = Double.NaN;
    private static final double REFRESH_PERIOD_SECONDS = 0.25;
    private static GenericEntry shuffleboardEnabledEntry;
    private static GenericEntry defaultPeriodSecondsEntry;
    private static GenericEntry debugEnabledEntry;
    private static volatile boolean cachedShuffleboardEnabled = true;
    private static volatile double cachedDefaultPeriodSeconds = Double.NaN;
    private static volatile boolean cachedDebugEnabled = false;

    private static final Object MECH_LOCK = new Object();
    private static final LinkedHashMap<String, MechanismToggles> mechanismToggles = new LinkedHashMap<>();
    private static final Set<String> pendingMechanisms = new LinkedHashSet<>();
    private static final int MECH_TOGGLE_START_X = 0;
    private static final int MECH_TOGGLE_DEBUG_X = 3;
    private static final int MECH_TOGGLE_START_Y = 6;

    private static final class MechanismToggles {
        final AtomicBoolean publishEnabled;
        final AtomicBoolean debugEnabled;
        GenericEntry publishEntry;
        GenericEntry debugEntry;
        final int index;

        private MechanismToggles(boolean publishDefault, boolean debugDefault, int index) {
            this.publishEnabled = new AtomicBoolean(publishDefault);
            this.debugEnabled = new AtomicBoolean(debugDefault);
            this.index = index;
        }
    }

    static {
        for (Flag flag : Flag.values()) {
            cached.put(flag, new AtomicBoolean(flag.defaultValue()));
        }
    }

    public static boolean enabled(Flag flag) {
        AtomicBoolean value = cached.get(flag);
        return value != null ? value.get() : flag.defaultValue();
    }

    /**
     * Master dashboard gate. When false, Athena will avoid publishing Shuffleboard widgets and will
     * also suppress "dashboard-oriented" NetworkTables topics (ex: pose struct topics).
     */
    public static boolean dashboardEnabled() {
        return cachedShuffleboardEnabled;
    }

    /**
     * Registers a mechanism name so Athena can expose dedicated per-mechanism Shuffleboard toggles.
     *
     * <p>Defaults are intentionally OFF to avoid "publish everything" crushing the roboRIO.</p>
     */
    public static void registerMechanism(String mechanismName) {
        if (mechanismName == null) {
            return;
        }
        String name = mechanismName.trim();
        if (name.isEmpty()) {
            return;
        }

        boolean publishNow = false;
        synchronized (MECH_LOCK) {
            if (mechanismToggles.containsKey(name)) {
                return;
            }
            mechanismToggles.put(name, new MechanismToggles(false, false, mechanismToggles.size()));
            if (published && configTab != null) {
                publishNow = true;
            } else {
                pendingMechanisms.add(name);
            }
        }
        if (publishNow) {
            publishMechanismToggles(name);
        }
    }

    public static boolean mechanismEnabled(String mechanismName) {
        if (mechanismName == null) {
            return false;
        }
        synchronized (MECH_LOCK) {
            MechanismToggles t = mechanismToggles.get(mechanismName.trim());
            return t != null && t.publishEnabled.get();
        }
    }

    public static boolean mechanismDebugEnabled(String mechanismName) {
        if (mechanismName == null) {
            return false;
        }
        synchronized (MECH_LOCK) {
            MechanismToggles t = mechanismToggles.get(mechanismName.trim());
            return t != null && t.debugEnabled.get();
        }
    }

    public static boolean mechanismAllowed(String mechanismName, RobotSendableSystem.SendableLevel level) {
        if (!dashboardEnabled()) {
            return false;
        }
        String name = mechanismName != null ? mechanismName.trim() : "";
        MechanismToggles t = null;
        if (!name.isEmpty()) {
            synchronized (MECH_LOCK) {
                t = mechanismToggles.get(name);
            }
        }

        // Strict policy: if a mechanism isn't registered, it is not allowed to publish anything.
        if (t == null) {
            return false;
        }

        if (level == RobotSendableSystem.SendableLevel.DEBUG) {
            return cachedDebugEnabled && t.debugEnabled.get();
        }
        return t.publishEnabled.get();
    }

    /**
     * Publishes a single standardized config tab. Safe to call multiple times.
     *
     * <p>Call once from robot init (preferred: {@link ca.frc6390.athena.core.RobotCore#robotInit()}).</p>
     */
    public static void publishConfigTab() {
        if (published) {
            return;
        }
        published = true;

        ShuffleboardTab tab = Shuffleboard.getTab(RobotSendableSystem.SHUFFLEBOARD_ROOT + "/Config");
        configTab = tab;

        // Place widgets directly on the tab so users don't have to drag out nested layouts.
        final int xSystem = 0;
        final int xMechanisms = 4;
        final int xHardware = 10;
        final int xLocalization = 14;
        final int xVision = 18;
        final int xTelemetry = 22;

        shuffleboardEnabledEntry = tab.add("Enable Shuffleboard Publishing", RobotSendableSystem.isShuffleboardEnabled())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(xSystem, 0)
                .withSize(3, 1)
                .getEntry();
        debugEnabledEntry = tab.add("Enable DEBUG Publishing", RobotSendableSystem.isDebugShuffleboardEnabled())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(xSystem, 1)
                .withSize(3, 1)
                .getEntry();
        defaultPeriodSecondsEntry = tab.add("Default Period (s)", RobotSendableSystem.getDefaultShuffleboardPeriodSeconds())
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(xSystem, 2)
                .withSize(3, 1)
                .getEntry();

        addToggle(tab, "Auto Publish: Core", Flag.AUTO_PUBLISH_CORE, xSystem, 3);
        addToggle(tab, "Auto Publish: Mechanisms", Flag.AUTO_PUBLISH_MECHANISMS, xSystem, 4);
        cachedShuffleboardEnabled = RobotSendableSystem.isShuffleboardEnabled();
        cachedDefaultPeriodSeconds = RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();
        cachedDebugEnabled = RobotSendableSystem.isDebugShuffleboardEnabled();

        // Mechanisms: 2 columns x 6 rows.
        addToggle(tab, "Mech: Motors", Flag.MECHANISM_MOTORS, xMechanisms, 0);
        addToggle(tab, "Mech: Encoders", Flag.MECHANISM_ENCODERS, xMechanisms, 1);
        addToggle(tab, "Mech: Status", Flag.MECHANISM_STATUS, xMechanisms, 2);
        addToggle(tab, "Mech: Setpoints", Flag.MECHANISM_SETPOINTS, xMechanisms, 3);
        addToggle(tab, "Mech: Outputs", Flag.MECHANISM_OUTPUTS, xMechanisms, 4);
        addToggle(tab, "Mech: Visualization", Flag.MECHANISM_VISUALIZATION, xMechanisms, 5);

        addToggle(tab, "Mech: Simulation", Flag.MECHANISM_SIMULATION, xMechanisms + 3, 0);
        addToggle(tab, "Mech: Config", Flag.MECHANISM_CONFIG, xMechanisms + 3, 1);
        addToggle(tab, "Mech: Limit Switches", Flag.MECHANISM_LIMIT_SWITCHES, xMechanisms + 3, 2);
        addToggle(tab, "Mech: Commands", Flag.MECHANISM_COMMANDS, xMechanisms + 3, 3);
        addToggle(tab, "Mech: SysId", Flag.MECHANISM_SYSID, xMechanisms + 3, 4);
        addToggle(tab, "Mech: Controllers", Flag.MECHANISM_CONTROLLERS, xMechanisms + 3, 5);

        // Hardware + other heavy debug.
        addToggle(tab, "HW: Per-Motor Widgets", Flag.MOTOR_GROUP_PER_MOTOR, xHardware, 0);
        addToggle(tab, "HW: Per-Encoder Widgets", Flag.ENCODER_GROUP_PER_ENCODER, xHardware, 1);
        addToggle(tab, "Other: State List (SM)", Flag.STATE_MACHINE_STATES_LIST, xHardware, 3);
        addToggle(tab, "Other: Swerve Debug", Flag.SWERVE_MODULE_DEBUG, xHardware, 4);

        // Localization / vision.
        addToggle(tab, "Loc: Health Widgets", Flag.LOCALIZATION_HEALTH_WIDGETS, xLocalization, 0);
        addToggle(tab, "Loc: Backend Overrides", Flag.LOCALIZATION_BACKEND_WIDGETS, xLocalization, 1);
        addToggle(tab, "Loc: Pose Topics", Flag.LOCALIZATION_POSE_TOPICS, xLocalization, 2);

        addToggle(tab, "Vision: Field Widget", Flag.VISION_FIELD_WIDGET, xVision, 0);
        addToggle(tab, "Vision: Camera Widgets", Flag.VISION_CAMERA_WIDGETS, xVision, 1);

        // Telemetry.
        addToggle(tab, "Telemetry: Shuffleboard Widgets", Flag.TELEMETRY_SHUFFLEBOARD_WIDGETS, xTelemetry, 0);

        publishPendingMechanismToggles();
    }

    /**
     * Refresh cached values from NetworkTables at a low rate.
     *
     * <p>Call once per robot loop (preferred: early in robotPeriodic).</p>
     */
    public static void refresh() {
        if (!published) {
            return;
        }
        double nowSeconds = Timer.getFPGATimestamp();
        if (Double.isFinite(lastRefreshSeconds) && (nowSeconds - lastRefreshSeconds) < REFRESH_PERIOD_SECONDS) {
            return;
        }
        lastRefreshSeconds = nowSeconds;

        if (shuffleboardEnabledEntry != null) {
            boolean enabled = shuffleboardEnabledEntry.getBoolean(cachedShuffleboardEnabled);
            cachedShuffleboardEnabled = enabled;
            RobotSendableSystem.setShuffleboardEnabled(enabled);
            if (!enabled) {
                // Keep debug publishing coherent with master disable.
                cachedDebugEnabled = false;
                RobotSendableSystem.setDebugShuffleboardEnabled(false);
            }
        }
        if (defaultPeriodSecondsEntry != null) {
            double candidate = defaultPeriodSecondsEntry.getDouble(cachedDefaultPeriodSeconds);
            if (Double.isFinite(candidate) && candidate > 0.0) {
                cachedDefaultPeriodSeconds = candidate;
                RobotSendableSystem.setDefaultShuffleboardPeriodSeconds(candidate);
            }
        }
        if (debugEnabledEntry != null) {
            boolean enabled = debugEnabledEntry.getBoolean(cachedDebugEnabled);
            cachedDebugEnabled = enabled && cachedShuffleboardEnabled;
            RobotSendableSystem.setDebugShuffleboardEnabled(cachedDebugEnabled);
        }

        for (Map.Entry<Flag, GenericEntry> entry : entries.entrySet()) {
            Flag flag = entry.getKey();
            GenericEntry widget = entry.getValue();
            if (flag == null || widget == null) {
                continue;
            }
            boolean value = widget.getBoolean(flag.defaultValue());
            AtomicBoolean target = cached.get(flag);
            if (target != null) {
                target.set(value);
            }
        }

        synchronized (MECH_LOCK) {
            for (MechanismToggles t : mechanismToggles.values()) {
                if (t == null) {
                    continue;
                }
                if (t.publishEntry != null) {
                    t.publishEnabled.set(t.publishEntry.getBoolean(t.publishEnabled.get()));
                }
                if (t.debugEntry != null) {
                    t.debugEnabled.set(t.debugEntry.getBoolean(t.debugEnabled.get()));
                }
            }
        }
    }

    private static void addToggle(ShuffleboardTab tab, String name, Flag flag, int x, int y) {
        GenericEntry entry = tab.add(name, flag.defaultValue())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(x, y)
                .withSize(3, 1)
                .getEntry();
        entries.put(flag, entry);
        cached.get(flag).set(flag.defaultValue());
        entry.setBoolean(flag.defaultValue());
    }

    private static void publishPendingMechanismToggles() {
        Set<String> toPublish = null;
        synchronized (MECH_LOCK) {
            if (!pendingMechanisms.isEmpty()) {
                toPublish = new LinkedHashSet<>(pendingMechanisms);
                pendingMechanisms.clear();
            }
        }
        if (toPublish == null) {
            return;
        }
        for (String name : toPublish) {
            publishMechanismToggles(name);
        }
    }

    private static void publishMechanismToggles(String mechanismName) {
        ShuffleboardTab tab = configTab;
        if (tab == null || mechanismName == null) {
            return;
        }

        MechanismToggles t;
        synchronized (MECH_LOCK) {
            t = mechanismToggles.get(mechanismName);
        }
        if (t == null) {
            return;
        }

        int y = MECH_TOGGLE_START_Y + t.index;
        if (t.publishEntry == null) {
            t.publishEntry = tab.add("Mech Publish: " + mechanismName, t.publishEnabled.get())
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .withPosition(MECH_TOGGLE_START_X, y)
                    .withSize(3, 1)
                    .getEntry();
        }
        if (t.debugEntry == null) {
            t.debugEntry = tab.add("Mech Debug: " + mechanismName, t.debugEnabled.get())
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .withPosition(MECH_TOGGLE_DEBUG_X, y)
                    .withSize(3, 1)
                    .getEntry();
        }
    }
}
