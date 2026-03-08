package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.nio.file.Path;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.core.hooks.LifecycleHooksSectionBase;
import ca.frc6390.athena.core.input.TypedInputRegistration;
import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.ChineseRemainderEncoder;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderAdapter;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.encoder.SupplierEncoder;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.hardware.motor.MotorRegistry;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.FlywheelMechanism;
import ca.frc6390.athena.mechanisms.statespec.StateSpecAccess;
import ca.frc6390.athena.mechanisms.FlywheelMechanism.StatefulFlywheelMechanism;
	import ca.frc6390.athena.mechanisms.TurretMechanism;
	import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;
	import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
	import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.BlockDirection;
	import ca.frc6390.athena.mechanisms.sim.MechanismSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationModel;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSensorSimulationConfig;
import ca.frc6390.athena.mechanisms.config.MechanismConfigApplier;
import ca.frc6390.athena.mechanisms.config.MechanismConfigFile;
import ca.frc6390.athena.mechanisms.config.MechanismConfigLoader;
import ca.frc6390.athena.mechanisms.config.MechanismEncoderConfig;
import ca.frc6390.athena.mechanisms.config.MechanismEncoderCrtInputConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Fluent builder that captures the hardware and control configuration for a {@link Mechanism}.
 * Teams populate this object with motor, sensor, and control metadata before calling
 * {@link #build()} to construct the runtime mechanism instance (and optional simulation models).
 *
 * @param <T> concrete mechanism type that will be created from this configuration
 */
public class MechanismConfig<T extends Mechanism> {

    private MechanismConfigRecord data = MechanismConfigRecord.defaults();
    private String mechanismName;
    private boolean disabled;
    /**
     * When true, turret factory helpers will auto-enable continuous PID input for unbounded turrets
     * using the configured encoder conversion as the wrap span (e.g. 360 degrees, 2*pi radians).
     *
     * This is intentionally only applied when bounds are not configured because bounded turrets
     * typically must not "wrap" across hard stops.
     */
    private boolean autoContinuousPidForUnboundedTurret;
    /** Factory used to instantiate the final mechanism once configuration is complete. */
    private Function<MechanismConfig<T>, T> factory = null;
    /** Optional per-state callbacks that run when the mechanism state machine enters the state. */
    private final Map<Enum<?>, Function<T, Boolean>> stateActions = new HashMap<>();
    /** Optional hooks that run once when entering a state. */
    private final Map<Enum<?>, List<MechanismBinding<T, ?>>> enterStateHooks = new HashMap<>();
    /** Optional state hooks that run every loop while a state is active. */
    private final Map<Enum<?>, List<MechanismBinding<T, ?>>> stateHooks = new HashMap<>();
    /** Optional hooks that run once when leaving a state. */
    private final Map<Enum<?>, List<MechanismBinding<T, ?>>> exitStateHooks = new HashMap<>();
    /** Optional hooks that run once for specific state transitions (from -> to). */
    private final List<TransitionHookBinding<T>> transitionHooks = new ArrayList<>();
    /** Optional hooks that run once whenever any state is exited. */
    private final List<MechanismBinding<T, ?>> exitAlwaysHooks = new ArrayList<>();
    /** Optional hooks that run every loop regardless of the active state. */
    private final List<MechanismBinding<T, ?>> alwaysHooks = new ArrayList<>();
    /** Optional hooks that run once during robot init after all mechanisms are registered. */
    private final List<MechanismBinding<T, ?>> initBindings = new ArrayList<>();
    /** Optional lifecycle hooks keyed by robot phase (RobotCore parity). */
    private final Map<RobotCoreHooks.Phase, List<LifecycleHookBinding<T>>> lifecycleBindings = new HashMap<>();
    /** Optional hooks that run whenever a robot mode exits (before mode-specific exit hooks). */
    private final List<LifecycleHookBinding<T>> lifecycleExitBindings = new ArrayList<>();
    /** Optional state triggers that can enqueue states when a predicate becomes true. */
    private final List<StateTriggerBinding<T>> stateTriggerBindings = new ArrayList<>();
    /** Optional hooks that run every periodic loop. */
    private final List<Consumer<T>> periodicHooks = new ArrayList<>();
    /** Optional hooks that run on a fixed cadence rather than every loop. */
    private final List<PeriodicHookBinding<T>> periodicHookBindings = new ArrayList<>();
    /** Optional custom control loops that return output contributions. */
    private final List<ControlLoopBinding<T>> controlLoops = new ArrayList<>();
    /** Optional named PID profiles for control-loop usage. */
    private final Map<String, PidProfile> controlLoopPidProfiles = new HashMap<>();
    /** Optional named bang-bang profiles for control-loop usage. */
    private final Map<String, BangBangProfile> controlLoopBangBangProfiles = new HashMap<>();
    /** Optional named feedforward profiles for control-loop and mechanism usage. */
    private final Map<String, FeedforwardProfile> controlLoopFeedforwardProfiles = new HashMap<>();
    /** Optional boolean inputs exposed to state hooks. */
    private final Map<String, BooleanSupplier> inputs = new HashMap<>();
    /** Optional mutable boolean inputs (defaults) that other systems can set at runtime. */
    private final Map<String, Boolean> mutableBoolInputDefaults = new HashMap<>();
    /** Optional double inputs exposed to state hooks. */
    private final Map<String, DoubleSupplier> doubleInputs = new HashMap<>();
    /** Optional mutable double inputs (defaults) that other systems can set at runtime. */
    private final Map<String, Double> mutableDoubleInputDefaults = new HashMap<>();
    /** Optional int inputs exposed to hooks/loops. */
    private final Map<String, IntSupplier> intInputs = new HashMap<>();
    /** Optional mutable int inputs (defaults) that other systems can set at runtime. */
    private final Map<String, Integer> mutableIntInputDefaults = new HashMap<>();
    /** Optional string inputs exposed to hooks/loops. */
    private final Map<String, Supplier<String>> stringInputs = new HashMap<>();
    /** Optional mutable string inputs (defaults) that other systems can set at runtime. */
    private final Map<String, String> mutableStringInputDefaults = new HashMap<>();
    /** Optional Pose2d inputs exposed to hooks/loops. */
    private final Map<String, Supplier<Pose2d>> pose2dInputs = new HashMap<>();
    /** Optional mutable Pose2d inputs (defaults) that other systems can set at runtime. */
    private final Map<String, Pose2d> mutablePose2dInputDefaults = new HashMap<>();
    /** Optional Pose3d inputs exposed to hooks/loops. */
    private final Map<String, Supplier<Pose3d>> pose3dInputs = new HashMap<>();
    /** Optional mutable Pose3d inputs (defaults) that other systems can set at runtime. */
    private final Map<String, Pose3d> mutablePose3dInputDefaults = new HashMap<>();
    /** Optional object inputs exposed to state hooks. */
    private final Map<String, Supplier<?>> objectInputs = new HashMap<>();
    /** Optional transition graph that defines required intermediate states and guards. */
    private StateGraph<?> stateGraph = null;
    /** Simulation model description used when running in simulation environments. */
    private MechanismSimulationConfig simulationConfig = null;
    /** Cached elevator-specific simulation hints provided through {@link #setSimulationElevator}. */
    private ElevatorSimulationParameters elevatorSimulationParameters = null;
    /** Cached arm-specific simulation hints provided through {@link #setSimulationArm}. */
    private ArmSimulationParameters armSimulationParameters = null;
    /** Cached simple-motor simulation hints provided through {@link #setSimulationSimpleMotor}. */
    private SimpleMotorSimulationParameters simpleMotorSimulationParameters = null;
    /** Optional visualization metadata consumed by the mechanism visualizer. */
    private MechanismVisualizationConfig visualizationConfig = null;
    /** Optional field-heading visualization config for turret mechanisms. */
    private Supplier<TurretMechanism.FieldHeadingVisualization> turretHeadingVisualization = null;
    /** Optional sensor simulation configuration used to generate virtual readings. */
    private MechanismSensorSimulationConfig sensorSimulationConfig = null;
    private final LinkedHashMap<String, EncoderSourceSpec> encoderSourceSpecs = new LinkedHashMap<>();
    private String controlPositionSourceName;
    private String controlVelocitySourceName;
    private String controlAbsoluteSourceName;
    /**
     * Loads a JSON/TOML deploy-file mechanism config into this builder and applies it using
     * {@link MechanismConfigApplier}. This is intended for teams to keep hardware/constants in deploy
     * files while retaining Java-only hooks and loops.
     */
    public MechanismConfig<T> loadFrom(Path path) {
        MechanismConfigFile file = MechanismConfigLoader.load(path);
        MechanismConfigApplier.apply(this, file);
        return this;
    }

    /**
     * Loads a base JSON/TOML config and applies one or more overlays (deep merge, overlay wins).
     */
    public MechanismConfig<T> loadFrom(Path base, Path... overlays) {
        MechanismConfigFile file = MechanismConfigLoader.loadMerged(base, overlays);
        MechanismConfigApplier.apply(this, file);
        return this;
    }

    /**
     * Sectioned fluent API: motors.
     */
    public MechanismConfig<T> motors(Consumer<MotorsSection<T>> section) {
        if (section != null) {
            section.accept(new MotorsSection<>(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API: named encoder sources.
     */
    public MechanismConfig<T> encoders(Consumer<EncodersSection<T>> section) {
        if (section != null) {
            section.accept(new EncodersSection<>(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API: constraints (formerly bounds/limits).
     */
    public MechanismConfig<T> constraints(Consumer<ConstraintsSection<T>> section) {
        if (section != null) {
            section.accept(new ConstraintsSection<>(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API: sensors.
     */
    public MechanismConfig<T> sensors(Consumer<SensorsSection<T>> section) {
        if (section != null) {
            section.accept(new SensorsSection<>(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API: inputs (typed external values usable by hooks and control loops).
     */
    public MechanismConfig<T> inputs(Consumer<InputsSection<T>> section) {
        if (section != null) {
            section.accept(new InputsSection<>(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API: control.
     */
    public MechanismConfig<T> control(Consumer<ControlSection<T>> section) {
        if (section != null) {
            section.accept(new ControlSection<>(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API: simulation.
     */
    public MechanismConfig<T> sim(Consumer<SimSection<T>> section) {
        if (section != null) {
            section.accept(new SimSection<>(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API: hooks (state hooks, exit hooks, periodic hooks, etc).
     */
    public MechanismConfig<T> hooks(Consumer<HooksSection<T>> section) {
        if (section != null) {
            section.accept(new HooksSection<>(this));
        }
        return this;
    }

    public static final class MotorsSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private MotorsSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public MotorsSection<T> add(AthenaMotor motor, int id) {
            owner.data.motors().add(MotorControllerConfig.create(motor.resolveController(), id));
            return this;
        }

        /**
         * Adds a motor controller using a {@link MotorControllerType} registry key (data-only / deploy-friendly).
         */
        public MotorsSection<T> add(MotorControllerType type, int id) {
            owner.data.motors().add(MotorControllerConfig.create(type, id));
            return this;
        }

        public MotorsSection<T> neutralMode(MotorNeutralMode mode) {
            owner.updateData(builder -> builder.motorNeutralMode(mode));
            return this;
        }

        public MotorsSection<T> currentLimit(double amps) {
            owner.updateData(builder -> builder.motorCurrentLimit(amps));
            return this;
        }

        public MotorsSection<T> canbus(String canbus) {
            owner.updateData(builder -> builder.canbus(canbus));
            return this;
        }
    }

    public static final class EncodersSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private EncodersSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public EncodersSection<T> add(String name, Consumer<EncoderSourceBuilder> section) {
            String key = normalizeSourceName(name);
            if (owner.encoderSourceSpecs.containsKey(key)) {
                throw new IllegalArgumentException("encoder source already registered: " + key);
            }
            EncoderSourceBuilder builder = new EncoderSourceBuilder(owner, key);
            if (section != null) {
                section.accept(builder);
            }
            owner.encoderSourceSpecs.put(key, builder.build());
            return this;
        }

        private static String normalizeSourceName(String name) {
            if (name == null || name.isBlank()) {
                throw new IllegalArgumentException("encoder source name cannot be blank");
            }
            return name.trim();
        }
    }

    public static final class EncoderSourceBuilder {
        private final MechanismConfig<?> owner;
        private final String name;
        private EncoderSourceKind kind;
        private AthenaEncoder athenaType;
        private EncoderType type;
        private int id;
        private String canbus;
        private double gearRatio = 1.0;
        private double conversion = 1.0;
        private double offset = 0.0;
        private double wrapsEvery = Double.NaN;
        private MechanismEncoderUnit unit;
        private DoubleSupplier positionSupplier;
        private DoubleSupplier velocitySupplier;
        private DoubleSupplier absoluteSupplier;
        private final List<CrtInputSpec> crtInputs = new ArrayList<>();
        private double validMin = Double.NaN;
        private double validMax = Double.NaN;

        private EncoderSourceBuilder(MechanismConfig<?> owner, String name) {
            this.owner = owner;
            this.name = name;
        }

        public EncoderSourceBuilder config(EncoderConfig config) {
            EncoderConfig resolved = Objects.requireNonNull(config, "config");
            EncoderType resolvedType = resolved.type();
            if (resolvedType == null) {
                throw new IllegalArgumentException("encoder config must declare a type");
            }
            int resolvedId = resolved.id();
            if (resolved.inverted() && resolvedId > 0) {
                resolvedId = -resolvedId;
            }
            this.kind = EncoderSourceKind.HARDWARE;
            this.type = resolvedType;
            this.athenaType = null;
            this.id = resolvedId;
            this.canbus = resolved.canbus();
            this.gearRatio = resolved.gearRatio();
            this.conversion = resolved.conversion();
            this.offset = resolved.conversionOffset();
            return this;
        }

        public EncoderSourceBuilder encoder(AthenaEncoder type, int id) {
            this.kind = EncoderSourceKind.HARDWARE;
            this.athenaType = Objects.requireNonNull(type, "type");
            this.type = null;
            this.id = id;
            return this;
        }

        public EncoderSourceBuilder encoder(EncoderType type, int id) {
            this.kind = EncoderSourceKind.HARDWARE;
            this.type = Objects.requireNonNull(type, "type");
            this.athenaType = null;
            this.id = id;
            return this;
        }

        public EncoderSourceBuilder virtual(DoubleSupplier positionSupplier) {
            this.kind = EncoderSourceKind.VIRTUAL;
            this.positionSupplier = Objects.requireNonNull(positionSupplier, "positionSupplier");
            return this;
        }

        public EncoderSourceBuilder virtual(Consumer<VirtualSourceBuilder> section) {
            this.kind = EncoderSourceKind.VIRTUAL;
            VirtualSourceBuilder builder = new VirtualSourceBuilder();
            if (section != null) {
                section.accept(builder);
            }
            this.positionSupplier = builder.positionSupplier;
            this.velocitySupplier = builder.velocitySupplier;
            this.absoluteSupplier = builder.absoluteSupplier;
            return this;
        }

        public EncoderSourceBuilder crt(Consumer<CrtSourceBuilder> section) {
            this.kind = EncoderSourceKind.CRT;
            CrtSourceBuilder builder = new CrtSourceBuilder(owner);
            if (section != null) {
                section.accept(builder);
            }
            this.crtInputs.clear();
            this.crtInputs.addAll(builder.inputs);
            this.validMin = builder.validMin;
            this.validMax = builder.validMax;
            return this;
        }

        public EncoderSourceBuilder canbus(String canbus) {
            this.canbus = canbus;
            return this;
        }

        public EncoderSourceBuilder gearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
            return this;
        }

        public EncoderSourceBuilder conversion(double conversion) {
            this.conversion = conversion;
            return this;
        }

        public EncoderSourceBuilder offset(double offset) {
            this.offset = offset;
            return this;
        }

        public EncoderSourceBuilder unit(MechanismEncoderUnit unit) {
            this.unit = unit;
            return this;
        }

        public EncoderSourceBuilder wrapsEvery(double wrapsEvery) {
            this.wrapsEvery = wrapsEvery;
            return this;
        }

        private EncoderSourceSpec build() {
            if (kind == null) {
                throw new IllegalStateException("encoder source '" + name + "' is missing a source definition.");
            }
            if (kind == EncoderSourceKind.CRT && crtInputs.isEmpty()) {
                throw new IllegalStateException("CRT encoder source '" + name + "' must declare at least one input.");
            }
            return new EncoderSourceSpec(
                    name,
                    kind,
                    athenaType,
                    type,
                    id,
                    canbus,
                    gearRatio,
                    conversion,
                    offset,
                    unit,
                    wrapsEvery,
                    positionSupplier,
                    velocitySupplier,
                    absoluteSupplier,
                    List.copyOf(crtInputs),
                    validMin,
                    validMax);
        }
    }

    public static final class VirtualSourceBuilder {
        private DoubleSupplier positionSupplier;
        private DoubleSupplier velocitySupplier;
        private DoubleSupplier absoluteSupplier;

        public VirtualSourceBuilder position(DoubleSupplier positionSupplier) {
            this.positionSupplier = positionSupplier;
            return this;
        }

        public VirtualSourceBuilder velocity(DoubleSupplier velocitySupplier) {
            this.velocitySupplier = velocitySupplier;
            return this;
        }

        public VirtualSourceBuilder absolute(DoubleSupplier absoluteSupplier) {
            this.absoluteSupplier = absoluteSupplier;
            return this;
        }
    }

    public static final class CrtSourceBuilder {
        private final MechanismConfig<?> owner;
        private final List<CrtInputSpec> inputs = new ArrayList<>();
        private double validMin = Double.NaN;
        private double validMax = Double.NaN;

        private CrtSourceBuilder(MechanismConfig<?> owner) {
            this.owner = owner;
        }

        public CrtSourceBuilder input(String sourceName, int modulus) {
            String key = EncodersSection.normalizeSourceName(sourceName);
            if (!owner.encoderSourceSpecs.containsKey(key)) {
                throw new IllegalArgumentException("CRT input source is not defined yet: " + key);
            }
            inputs.add(new CrtInputSpec(key, modulus));
            return this;
        }

        public CrtSourceBuilder validRange(double min, double max) {
            this.validMin = min;
            this.validMax = max;
            return this;
        }
    }

    private enum EncoderSourceKind {
        HARDWARE,
        VIRTUAL,
        CRT
    }

    private record CrtInputSpec(String sourceName, int modulus) {
    }

    private record EncoderSourceSpec(
            String name,
            EncoderSourceKind kind,
            AthenaEncoder athenaType,
            EncoderType type,
            int id,
            String canbus,
            double gearRatio,
            double conversion,
            double offset,
            MechanismEncoderUnit unit,
            double wrapsEvery,
            DoubleSupplier positionSupplier,
            DoubleSupplier velocitySupplier,
            DoubleSupplier absoluteSupplier,
            List<CrtInputSpec> crtInputs,
            double validMin,
            double validMax) {
    }

    public static final class ConstraintsSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private ConstraintsSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public ConstraintsSection<T> bounds(double min, double max) {
            owner.updateData(builder -> builder.minBound(min).maxBound(max));
            return this;
        }

        public ConstraintsSection<T> clearBounds() {
            owner.updateData(builder -> builder.minBound(Double.NaN).maxBound(Double.NaN));
            return this;
        }

        public ConstraintsSection<T> motionLimits(MotionLimits.AxisLimits limits) {
            owner.updateData(builder -> builder.motionLimits(limits));
            return this;
        }
    }

    public static final class SensorsSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private SensorsSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public SensorsSection<T> limitSwitch(GenericLimitSwitchConfig cfg) {
            owner.data.limitSwitches().add(cfg);
            return this;
        }

        /**
         * Builder-style limit switch config (preferred).
         *
         * <p>Example:
         * <pre>
         * .sensors(s -> s.limitSwitch("TurretLimit", 5, sw -> sw
         *     .inverted(true)
         *     .position(0.0)
         *     .hardstop(false, BlockDirection.PositiveInput)
         *     .delaySeconds(0.0)))
         * </pre>
         * </p>
         */
        public SensorsSection<T> limitSwitch(String name, int dioPort, Consumer<MechanismLimitSwitchConfig> section) {
            MechanismLimitSwitchConfig cfg = new MechanismLimitSwitchConfig().dio(dioPort).name(name);
            if (section != null) {
                section.accept(cfg);
            }
            owner.data.limitSwitches().add(cfg.build());
            return this;
        }

        /**
         * Builder-style limit switch config (name and DIO port must be set inside the builder).
         */
        public SensorsSection<T> limitSwitch(Consumer<MechanismLimitSwitchConfig> section) {
            if (section == null) {
                throw new IllegalArgumentException("limit switch builder cannot be null");
            }
            MechanismLimitSwitchConfig cfg = new MechanismLimitSwitchConfig();
            section.accept(cfg);
            owner.data.limitSwitches().add(cfg.build());
            return this;
        }

        public SensorsSection<T> simulation(MechanismSensorSimulationConfig cfg) {
            owner.sensorSimulationConfig = cfg;
            return this;
        }

        /**
         * Sets how often this mechanism refreshes hardware devices (encoder + motor controller updates).
         * Use 20ms for full-rate updates, or a larger value to decimate slow devices.
         */
        public SensorsSection<T> hardwareUpdatePeriodSeconds(double periodSeconds) {
            owner.updateData(builder -> builder.hardwareUpdatePeriodSeconds(periodSeconds));
            return this;
        }

        /**
         * Millisecond variant of {@link #hardwareUpdatePeriodSeconds(double)}.
         */
        public SensorsSection<T> hardwareUpdatePeriodMs(double periodMs) {
            return hardwareUpdatePeriodSeconds(periodMs / 1000.0);
        }
    }

    public static final class InputsSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private InputsSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public InputsSection<T> boolVal(String key, BooleanSupplier supplier) {
            String k = Objects.requireNonNull(key, "key");
            Objects.requireNonNull(supplier, "supplier");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.mutableBoolInputDefaults.containsKey(k)) {
                throw new IllegalArgumentException("mutable bool input already registered: " + k);
            }
            TypedInputRegistration.put(owner.inputs, k, supplier);
            return this;
        }

        /**
         * Declares a mutable boolean input with a default value.
         * Other mechanisms/superstructures can set it via {@link Mechanism#input()} and {@code bool(...)}.
         */
        public InputsSection<T> boolVal(String key, boolean defaultValue) {
            String k = Objects.requireNonNull(key, "key");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.inputs.containsKey(k)) {
                throw new IllegalArgumentException("bool input already registered as supplier: " + k);
            }
            owner.mutableBoolInputDefaults.put(k, defaultValue);
            return this;
        }

        public InputsSection<T> doubleVal(String key, DoubleSupplier supplier) {
            String k = Objects.requireNonNull(key, "key");
            Objects.requireNonNull(supplier, "supplier");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.mutableDoubleInputDefaults.containsKey(k)) {
                throw new IllegalArgumentException("mutable double input already registered: " + k);
            }
            TypedInputRegistration.put(owner.doubleInputs, k, supplier);
            return this;
        }

        public InputsSection<T> doubleVal(String key, double defaultValue) {
            String k = Objects.requireNonNull(key, "key");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.doubleInputs.containsKey(k)) {
                throw new IllegalArgumentException("double input already registered as supplier: " + k);
            }
            owner.mutableDoubleInputDefaults.put(k, defaultValue);
            return this;
        }

        public InputsSection<T> intVal(String key, IntSupplier supplier) {
            String k = Objects.requireNonNull(key, "key");
            Objects.requireNonNull(supplier, "supplier");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.mutableIntInputDefaults.containsKey(k)) {
                throw new IllegalArgumentException("mutable int input already registered: " + k);
            }
            TypedInputRegistration.put(owner.intInputs, k, supplier);
            return this;
        }

        public InputsSection<T> intVal(String key, int defaultValue) {
            String k = Objects.requireNonNull(key, "key");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.intInputs.containsKey(k)) {
                throw new IllegalArgumentException("int input already registered as supplier: " + k);
            }
            owner.mutableIntInputDefaults.put(k, defaultValue);
            return this;
        }

        public InputsSection<T> stringVal(String key, Supplier<String> supplier) {
            String k = Objects.requireNonNull(key, "key");
            Objects.requireNonNull(supplier, "supplier");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.mutableStringInputDefaults.containsKey(k)) {
                throw new IllegalArgumentException("mutable string input already registered: " + k);
            }
            TypedInputRegistration.put(owner.stringInputs, k, supplier);
            return this;
        }

        public InputsSection<T> stringVal(String key, String defaultValue) {
            String k = Objects.requireNonNull(key, "key");
            String v = Objects.requireNonNull(defaultValue, "defaultValue");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.stringInputs.containsKey(k)) {
                throw new IllegalArgumentException("string input already registered as supplier: " + k);
            }
            owner.mutableStringInputDefaults.put(k, v);
            return this;
        }

        public InputsSection<T> pose2dVal(String key, Supplier<Pose2d> supplier) {
            String k = Objects.requireNonNull(key, "key");
            Objects.requireNonNull(supplier, "supplier");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.mutablePose2dInputDefaults.containsKey(k)) {
                throw new IllegalArgumentException("mutable Pose2d input already registered: " + k);
            }
            TypedInputRegistration.put(owner.pose2dInputs, k, supplier);
            return this;
        }

        public InputsSection<T> pose2dVal(String key, Pose2d defaultValue) {
            String k = Objects.requireNonNull(key, "key");
            Pose2d v = Objects.requireNonNull(defaultValue, "defaultValue");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.pose2dInputs.containsKey(k)) {
                throw new IllegalArgumentException("Pose2d input already registered as supplier: " + k);
            }
            owner.mutablePose2dInputDefaults.put(k, v);
            return this;
        }

        public InputsSection<T> pose3dVal(String key, Supplier<Pose3d> supplier) {
            String k = Objects.requireNonNull(key, "key");
            Objects.requireNonNull(supplier, "supplier");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.mutablePose3dInputDefaults.containsKey(k)) {
                throw new IllegalArgumentException("mutable Pose3d input already registered: " + k);
            }
            TypedInputRegistration.put(owner.pose3dInputs, k, supplier);
            return this;
        }

        public InputsSection<T> pose3dVal(String key, Pose3d defaultValue) {
            String k = Objects.requireNonNull(key, "key");
            Pose3d v = Objects.requireNonNull(defaultValue, "defaultValue");
            if (k.isBlank()) {
                throw new IllegalArgumentException("input key cannot be blank");
            }
            if (owner.pose3dInputs.containsKey(k)) {
                throw new IllegalArgumentException("Pose3d input already registered as supplier: " + k);
            }
            owner.mutablePose3dInputDefaults.put(k, v);
            return this;
        }

        public InputsSection<T> objVal(String key, Supplier<?> supplier) {
            TypedInputRegistration.put(owner.objectInputs, key, supplier);
            return this;
        }
    }

    public enum InputSource {
        Position,
        Velocity,
        Absolute
    }

    public static final class ControlSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private ControlSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public ControlSection<T> output(OutputType outputType) {
            owner.updateData(builder -> builder.outputType(outputType));
            return this;
        }

        public ControlSection<T> pidPeriod(double periodSeconds) {
            owner.updateData(builder -> builder.pidPeriod(periodSeconds));
            return this;
        }

        public ControlSection<T> pidContinuous(double min, double max) {
            owner.updateData(builder -> builder.pidContinous(true).continousMin(min).continousMax(max));
            return this;
        }

        public ControlSection<T> pidContinuousDisabled() {
            owner.updateData(builder -> builder.pidContinous(false).continousMin(0.0).continousMax(0.0));
            return this;
        }

        public ControlSection<T> setpointAsOutput(boolean enabled) {
            owner.updateData(builder -> builder.useSetpointAsOutput(enabled));
            return this;
        }

        public ControlSection<T> positionSource(String sourceName) {
            owner.controlPositionSourceName = EncodersSection.normalizeSourceName(sourceName);
            return this;
        }

        public ControlSection<T> velocitySource(String sourceName) {
            owner.controlVelocitySourceName = EncodersSection.normalizeSourceName(sourceName);
            return this;
        }

        public ControlSection<T> absoluteSource(String sourceName) {
            owner.controlAbsoluteSourceName = EncodersSection.normalizeSourceName(sourceName);
            return this;
        }

        public ControlSection<T> customPidCycle(boolean enabled, double periodSeconds) {
            owner.updateData(builder -> builder.customPIDCycle(enabled).pidPeriod(periodSeconds));
            return this;
        }

        public ControlSection<T> controlLoop(String name, double periodMs, MechanismControlLoop<T> loop) {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(loop, "loop");
            if (name.isBlank()) {
                throw new IllegalArgumentException("control loop name cannot be blank");
            }
            if (!Double.isFinite(periodMs) || periodMs <= 0.0) {
                throw new IllegalArgumentException("control loop period must be finite and > 0");
            }
            for (ControlLoopBinding<T> binding : owner.controlLoops) {
                if (binding != null && name.equals(binding.name())) {
                    throw new IllegalArgumentException("control loop name already registered: " + name);
                }
            }
            owner.controlLoops.add(new ControlLoopBinding<>(name, periodMs / 1000.0, loop));
            return this;
        }

        public ControlSection<T> controlLoopSeconds(String name, double periodSeconds, MechanismControlLoop<T> loop) {
            return controlLoop(name, periodSeconds * 1000.0, loop);
        }

        /**
         * Registers a periodic loop bound to the uniquely named controller entry.
         */
        public ControlSection<T> periodic(String name) {
            return registerPeriodic(name, periodMillisFromConfig(), name);
        }

        /**
         * Registers a periodic loop with explicit period (milliseconds) bound to the uniquely named
         * controller entry.
         */
        public ControlSection<T> periodic(String name, double periodMs) {
            return registerPeriodic(name, periodMs, name);
        }

        private ControlSection<T> registerPeriodic(
                String loopName,
                double periodMs,
                String controllerName) {
            String normalizedName = normalizeName(controllerName);
            if (normalizedName == null) {
                throw new IllegalArgumentException("periodic controller name cannot be blank");
            }
            boolean hasPid = owner.controlLoopPidProfiles.containsKey(normalizedName);
            boolean hasBangBang = owner.controlLoopBangBangProfiles.containsKey(normalizedName);
            FeedforwardProfile ffProfile = owner.controlLoopFeedforwardProfiles.get(normalizedName);
            if (ffProfile != null && ffProfile.type() != FeedforwardType.SIMPLE) {
                throw new IllegalArgumentException(
                        "Periodic loop '" + normalizedName
                                + "' references a non-simple feedforward profile. Use a custom control loop for ARM/ELEVATOR feedforward.");
            }
            boolean hasFf = ffProfile != null;
            int matches = (hasPid ? 1 : 0) + (hasBangBang ? 1 : 0) + (hasFf ? 1 : 0);
            if (matches == 0) {
                throw new IllegalArgumentException("No controller registered with name: " + normalizedName);
            }
            if (matches > 1) {
                throw new IllegalStateException(
                        "Controller name '" + normalizedName + "' collides across control registries");
            }
            PidProfile pidProfile = hasPid ? owner.controlLoopPidProfiles.get(normalizedName) : null;
            BangBangProfile bangBangProfile = hasBangBang ? owner.controlLoopBangBangProfiles.get(normalizedName) : null;
            return controlLoop(loopName, periodMs, ctx -> {
                double output = 0.0;
                if (hasPid) {
                    double measurement = resolveInputSource(
                            ctx,
                            pidProfile != null ? pidProfile.inputSource() : MechanismInputSource.Position);
                    double setpoint = resolveSetpointSource(
                            ctx,
                            pidProfile != null ? pidProfile.setpointSource() : MechanismSetpointSource.Setpoint);
                    output += ctx.pidOut(normalizedName, measurement, setpoint);
                }
                if (hasBangBang) {
                    double measurement = resolveInputSource(
                            ctx,
                            bangBangProfile != null ? bangBangProfile.inputSource() : MechanismInputSource.Position);
                    double setpoint = resolveSetpointSource(
                            ctx,
                            bangBangProfile != null ? bangBangProfile.setpointSource() : MechanismSetpointSource.Setpoint);
                    output += ctx.bangBangOut(normalizedName, measurement, setpoint);
                }
                if (hasFf) {
                    double velocitySetpoint = resolveSetpointSource(
                            ctx,
                            ffProfile != null ? ffProfile.setpointSource() : MechanismSetpointSource.Setpoint);
                    output += ctx.feedforwardOut(normalizedName, velocitySetpoint);
                }
                return output;
            });
        }

        private static double resolveInputSource(
                MechanismControlContext<?> ctx,
                MechanismInputSource source) {
            MechanismInputSource resolved = source != null ? source : MechanismInputSource.Position;
            return switch (resolved.kind()) {
                case POSITION -> ctx.mechanism().position(resolved.encoderId());
                case VELOCITY -> ctx.mechanism().velocity(resolved.encoderId());
                case ABSOLUTE -> ctx.mechanism().absolutePosition(resolved.encoderId());
                case INPUT -> ctx.doubleInput(resolved.inputKey());
            };
        }

        private static double resolveSetpointSource(
                MechanismControlContext<?> ctx,
                MechanismSetpointSource source) {
            MechanismSetpointSource resolved = source != null ? source : MechanismSetpointSource.Setpoint;
            return switch (resolved.kind()) {
                case SETPOINT -> ctx.mechanism().setpoint() + ctx.mechanism().nudge();
                case INPUT -> ctx.doubleInput(resolved.inputKey());
            };
        }

        private static String normalizeName(String name) {
            if (name == null) {
                return null;
            }
            String trimmed = name.trim();
            return trimmed.isEmpty() ? null : trimmed;
        }

        private double periodMillisFromConfig() {
            double configured = owner.data.pidPeriod();
            double seconds = (Double.isFinite(configured) && configured > 0.0) ? configured : 0.02;
            return seconds * 1000.0;
        }

        private static MechanismInputSource resolveInputSourceSelection(InputSource source, String encoderId) {
            InputSource resolved = source != null ? source : InputSource.Position;
            if (encoderId == null || encoderId.isBlank()) {
                return switch (resolved) {
                    case Position -> MechanismInputSource.Position;
                    case Velocity -> MechanismInputSource.Velocity;
                    case Absolute -> MechanismInputSource.Absolute;
                };
            }
            return switch (resolved) {
                case Position -> MechanismInputSource.position(encoderId);
                case Velocity -> MechanismInputSource.velocity(encoderId);
                case Absolute -> MechanismInputSource.absolute(encoderId);
            };
        }

        public ControlSection<T> pid(String name, Consumer<PidBuilder> builder) {
            Objects.requireNonNull(name, "name");
            PidBuilder spec = new PidBuilder();
            if (builder != null) {
                builder.accept(spec);
            }
            return registerPid(
                    name,
                    spec.outputType,
                    spec.kP,
                    spec.kI,
                    spec.kD,
                    spec.iZone,
                    spec.tolerance,
                    spec.maxVelocity,
                    spec.maxAcceleration,
                    spec.autotuner,
                    spec.inputSource,
                    spec.setpointSource);
        }

        public ControlSection<T> bangBang(String name, Consumer<BangBangBuilder> builder) {
            Objects.requireNonNull(name, "name");
            BangBangBuilder spec = new BangBangBuilder();
            if (builder != null) {
                builder.accept(spec);
            }
            double high = spec.highOutputSet ? spec.highOutput : spec.outputLevel;
            double low;
            if (spec.lowOutputSet) {
                low = spec.lowOutput;
            } else if (spec.highOutputSet) {
                low = -high;
            } else {
                low = -spec.outputLevel;
            }
            return registerBangBang(
                    name,
                    spec.outputType,
                    high,
                    low,
                    spec.tolerance,
                    spec.inputSource,
                    spec.setpointSource);
        }

        public ControlSection<T> ff(String name, Consumer<FeedforwardBuilder> builder) {
            Objects.requireNonNull(name, "name");
            FeedforwardBuilder spec = new FeedforwardBuilder();
            if (builder != null) {
                builder.accept(spec);
            }
            return registerFeedforward(
                    name,
                    spec.outputType,
                    spec.type,
                    spec.kS,
                    spec.kG,
                    spec.kV,
                    spec.kA,
                    spec.tolerance,
                    spec.setpointSource);
        }

        private ControlSection<T> registerPid(
                String name,
                OutputType outputType,
                double kP,
                double kI,
                double kD,
                double iZone,
                double tolerance,
                double maxVelocity,
                double maxAcceleration,
                PidAutotunerConfig pidAutotuner,
                MechanismInputSource inputSource,
                MechanismSetpointSource setpointSource) {
            Objects.requireNonNull(name, "name");
            String normalized = normalizeName(name);
            if (normalized == null) {
                throw new IllegalArgumentException("PID name cannot be blank");
            }
            assertControllerNameAvailable(normalized, "PID");
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.PERCENT;
            if (resolvedOutput != OutputType.PERCENT && resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("PID output type must be PERCENT or VOLTAGE");
            }
            boolean hasProfiledVelocity = Double.isFinite(maxVelocity) && maxVelocity > 0.0;
            boolean hasProfiledAcceleration = Double.isFinite(maxAcceleration) && maxAcceleration > 0.0;
            if (hasProfiledVelocity != hasProfiledAcceleration) {
                throw new IllegalArgumentException(
                        "PID constraints require both maxVelocity and maxAcceleration (both > 0)");
            }
            if ((Double.isFinite(maxVelocity) && maxVelocity <= 0.0)
                    || (Double.isFinite(maxAcceleration) && maxAcceleration <= 0.0)) {
                throw new IllegalArgumentException("PID constraints must be > 0 when provided");
            }
            owner.controlLoopPidProfiles.put(
                    normalized,
                    new PidProfile(
                            resolvedOutput,
                            kP,
                            kI,
                            kD,
                            iZone,
                            tolerance,
                            maxVelocity,
                            maxAcceleration,
                            pidAutotuner,
                            inputSource,
                            setpointSource));
            return this;
        }

        private ControlSection<T> registerBangBang(
                String name,
                OutputType outputType,
                double highOutput,
                double lowOutput,
                double tolerance,
                MechanismInputSource inputSource,
                MechanismSetpointSource setpointSource) {
            Objects.requireNonNull(name, "name");
            String normalized = normalizeName(name);
            if (normalized == null) {
                throw new IllegalArgumentException("bang-bang name cannot be blank");
            }
            assertControllerNameAvailable(normalized, "bang-bang");
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.PERCENT;
            if (resolvedOutput != OutputType.PERCENT && resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("bang-bang output type must be PERCENT or VOLTAGE");
            }
            owner.controlLoopBangBangProfiles.put(
                    normalized,
                    new BangBangProfile(
                            resolvedOutput,
                            highOutput,
                            lowOutput,
                            tolerance,
                            inputSource,
                            setpointSource));
            return this;
        }

        private ControlSection<T> registerFeedforward(
                String name,
                OutputType outputType,
                FeedforwardType type,
                double kS,
                double kG,
                double kV,
                double kA,
                double tolerance,
                MechanismSetpointSource setpointSource) {
            Objects.requireNonNull(name, "name");
            String normalized = normalizeName(name);
            if (normalized == null) {
                throw new IllegalArgumentException("feedforward name cannot be blank");
            }
            assertControllerNameAvailable(normalized, "feedforward");
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.VOLTAGE;
            if (resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("feedforward output type must be VOLTAGE");
            }
            FeedforwardType resolvedType = type != null ? type : FeedforwardType.SIMPLE;
            if (Double.isFinite(tolerance) && tolerance < 0.0) {
                throw new IllegalArgumentException("feedforward tolerance must be >= 0 when provided");
            }
            owner.controlLoopFeedforwardProfiles.put(
                    normalized,
                    new FeedforwardProfile(
                            resolvedOutput,
                            resolvedType,
                            kS,
                            kG,
                            kV,
                            kA,
                            tolerance,
                            setpointSource));
            return this;
        }

        private void assertControllerNameAvailable(String name, String kind) {
            String normalized = normalizeName(name);
            if (normalized == null) {
                throw new IllegalArgumentException(kind + " name cannot be blank");
            }
            if (owner.controlLoopPidProfiles.containsKey(normalized)
                    || owner.controlLoopBangBangProfiles.containsKey(normalized)
                    || owner.controlLoopFeedforwardProfiles.containsKey(normalized)) {
                throw new IllegalArgumentException(
                        "Controller name already registered: " + normalized + " (must be unique across all controllers)");
            }
        }

        public static final class PidBuilder {
            private OutputType outputType = OutputType.PERCENT;
            private double kP = 0.0;
            private double kI = 0.0;
            private double kD = 0.0;
            private double iZone = Double.NaN;
            private double tolerance = Double.NaN;
            private double maxVelocity = Double.NaN;
            private double maxAcceleration = Double.NaN;
            private PidAutotunerConfig autotuner = PidAutotunerConfig.defaults();
            private MechanismInputSource inputSource = MechanismInputSource.Position;
            private MechanismSetpointSource setpointSource = MechanismSetpointSource.Setpoint;

            public PidBuilder output(OutputType outputType) {
                this.outputType = outputType != null ? outputType : OutputType.PERCENT;
                return this;
            }

            public PidBuilder kp(double kP) {
                this.kP = kP;
                return this;
            }

            public PidBuilder ki(double kI) {
                this.kI = kI;
                return this;
            }

            public PidBuilder kd(double kD) {
                this.kD = kD;
                return this;
            }

            public PidBuilder iZone(double iZone) {
                this.iZone = iZone;
                return this;
            }

            public PidBuilder tolerance(double tolerance) {
                this.tolerance = tolerance;
                return this;
            }

            /**
             * Enables a profiled PID controller for this PID entry.
             */
            public PidBuilder constraints(double maxVelocity, double maxAcceleration) {
                this.maxVelocity = maxVelocity;
                this.maxAcceleration = maxAcceleration;
                return this;
            }

            /**
             * Enables profiled PID constraints for this PID entry.
             */
            public PidBuilder profiled(double maxVelocity, double maxAcceleration) {
                return constraints(maxVelocity, maxAcceleration);
            }

            public PidBuilder inputSource(MechanismInputSource inputSource) {
                this.inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
                return this;
            }

            public PidBuilder inputSource(InputSource inputSource) {
                return inputSource(resolveInputSourceSelection(inputSource, null));
            }

            public PidBuilder inputSource(InputSource inputSource, String encoderId) {
                return inputSource(resolveInputSourceSelection(inputSource, encoderId));
            }

            public PidBuilder inputInput(String key) {
                return inputSource(MechanismInputSource.input(key));
            }

            public PidBuilder setpointSource(MechanismSetpointSource setpointSource) {
                this.setpointSource = setpointSource != null
                        ? setpointSource
                        : MechanismSetpointSource.Setpoint;
                return this;
            }

            public PidBuilder setpointInput(String key) {
                return setpointSource(MechanismSetpointSource.input(key));
            }

            public PidBuilder autotuner() {
                autotuner = autotuner.withEnabled(true);
                return this;
            }

            public PidBuilder autotuner(MechanismPidAutotunerProgram program) {
                autotuner = autotuner.withEnabled(true).withProgram(program);
                return this;
            }

            public PidBuilder autotunerConfig(Consumer<PidAutotunerSection> section) {
                PidAutotunerSection builder = PidAutotunerSection.from(autotuner);
                if (section != null) {
                    section.accept(builder);
                }
                autotuner = builder.build();
                return this;
            }
        }

        public static final class PidAutotunerSection {
            private boolean enabled;
            private String dashboardPath;
            private MechanismPidAutotunerProgram program;

            private PidAutotunerSection(
                    boolean enabled,
                    String dashboardPath,
                    MechanismPidAutotunerProgram program) {
                this.enabled = enabled;
                this.dashboardPath = dashboardPath;
                this.program = program;
            }

            static PidAutotunerSection from(PidAutotunerConfig config) {
                PidAutotunerConfig resolved = config != null
                        ? config
                        : PidAutotunerConfig.defaults();
                return new PidAutotunerSection(
                        resolved.enabled(),
                        resolved.dashboardPath(),
                        resolved.program());
            }

            public PidAutotunerSection enabled(boolean enabled) {
                this.enabled = enabled;
                return this;
            }

            public PidAutotunerSection dashboardPath(String dashboardPath) {
                this.dashboardPath = dashboardPath;
                return this;
            }

            public PidAutotunerSection program(MechanismPidAutotunerProgram program) {
                this.program = program;
                return this;
            }

            PidAutotunerConfig build() {
                return new PidAutotunerConfig(enabled, dashboardPath, program);
            }
        }

        public static final class FeedforwardBuilder {
            private FeedforwardType type = FeedforwardType.SIMPLE;
            private OutputType outputType = OutputType.VOLTAGE;
            private double kS = 0.0;
            private double kG = 0.0;
            private double kV = 0.0;
            private double kA = 0.0;
            private double tolerance = Double.NaN;
            private MechanismSetpointSource setpointSource = MechanismSetpointSource.Setpoint;

            public FeedforwardBuilder simple() {
                this.type = FeedforwardType.SIMPLE;
                return this;
            }

            public FeedforwardBuilder simple(double kS, double kV, double kA) {
                return simple().ks(kS).kv(kV).ka(kA);
            }

            public FeedforwardBuilder arm() {
                this.type = FeedforwardType.ARM;
                return this;
            }

            public FeedforwardBuilder arm(double kS, double kG, double kV, double kA) {
                return arm().ks(kS).kg(kG).kv(kV).ka(kA);
            }

            public FeedforwardBuilder elevator() {
                this.type = FeedforwardType.ELEVATOR;
                return this;
            }

            public FeedforwardBuilder elevator(double kS, double kG, double kV, double kA) {
                return elevator().ks(kS).kg(kG).kv(kV).ka(kA);
            }

            public FeedforwardBuilder output(OutputType outputType) {
                this.outputType = outputType != null ? outputType : OutputType.VOLTAGE;
                return this;
            }

            public FeedforwardBuilder ks(double kS) {
                this.kS = kS;
                return this;
            }

            public FeedforwardBuilder kv(double kV) {
                this.kV = kV;
                return this;
            }

            public FeedforwardBuilder kg(double kG) {
                this.kG = kG;
                return this;
            }

            public FeedforwardBuilder ka(double kA) {
                this.kA = kA;
                return this;
            }

            public FeedforwardBuilder tolerance(double tolerance) {
                this.tolerance = tolerance;
                return this;
            }

            public FeedforwardBuilder setpointSource(MechanismSetpointSource setpointSource) {
                this.setpointSource = setpointSource != null
                        ? setpointSource
                        : MechanismSetpointSource.Setpoint;
                return this;
            }

            public FeedforwardBuilder setpointInput(String key) {
                return setpointSource(MechanismSetpointSource.input(key));
            }
        }

        public static final class BangBangBuilder {
            private OutputType outputType = OutputType.PERCENT;
            private double outputLevel = 0.0;
            private double highOutput = 0.0;
            private double lowOutput = 0.0;
            private boolean highOutputSet;
            private boolean lowOutputSet;
            private double tolerance = 0.0;
            private MechanismInputSource inputSource = MechanismInputSource.Position;
            private MechanismSetpointSource setpointSource = MechanismSetpointSource.Setpoint;

            public BangBangBuilder output(OutputType outputType) {
                this.outputType = outputType != null ? outputType : OutputType.PERCENT;
                return this;
            }

            public BangBangBuilder level(double outputLevel) {
                this.outputLevel = outputLevel;
                return this;
            }

            public BangBangBuilder high(double highOutput) {
                this.highOutput = highOutput;
                this.highOutputSet = true;
                return this;
            }

            public BangBangBuilder low(double lowOutput) {
                this.lowOutput = lowOutput;
                this.lowOutputSet = true;
                return this;
            }

            public BangBangBuilder tolerance(double tolerance) {
                this.tolerance = tolerance;
                return this;
            }

            public BangBangBuilder inputSource(MechanismInputSource inputSource) {
                this.inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
                return this;
            }

            public BangBangBuilder inputSource(InputSource inputSource) {
                return inputSource(resolveInputSourceSelection(inputSource, null));
            }

            public BangBangBuilder inputSource(InputSource inputSource, String encoderId) {
                return inputSource(resolveInputSourceSelection(inputSource, encoderId));
            }

            public BangBangBuilder inputInput(String key) {
                return inputSource(MechanismInputSource.input(key));
            }

            public BangBangBuilder setpointSource(MechanismSetpointSource setpointSource) {
                this.setpointSource = setpointSource != null
                        ? setpointSource
                        : MechanismSetpointSource.Setpoint;
                return this;
            }

            public BangBangBuilder setpointInput(String key) {
                return setpointSource(MechanismSetpointSource.input(key));
            }
        }

    }

    public static final class SimSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private SimSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public SimSection<T> config(MechanismSimulationConfig cfg) {
            owner.simulationConfig = cfg;
            return this;
        }

        public SimSection<T> factory(Function<T, MechanismSimulationModel> simulationFactory) {
            Objects.requireNonNull(simulationFactory, "simulationFactory");
            owner.simulationConfig = MechanismSimulationConfig.builder()
                    .withFactory(mechanism -> simulationFactory.apply((T) mechanism))
                    .build();
            return this;
        }

        public SimSection<T> simpleMotor(SimpleMotorSimulationParameters params) {
            owner.simpleMotorSimulationParameters = Objects.requireNonNull(params, "params");
            return this;
        }

        public SimSection<T> arm(ArmSimulationParameters params) {
            owner.armSimulationParameters = Objects.requireNonNull(params, "params");
            return this;
        }

        public SimSection<T> elevator(ElevatorSimulationParameters params) {
            owner.elevatorSimulationParameters = Objects.requireNonNull(params, "params");
            return this;
        }
    }

    public static final class HooksSection<T extends Mechanism>
            extends LifecycleHooksSectionBase<HooksSection<T>, MechanismBinding<T, ?>, Enum<?>> {
        private final MechanismConfig<T> owner;

        private HooksSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        @Override
        protected HooksSection<T> self() {
            return this;
        }

        @Override
        protected void addPhaseBinding(RobotCoreHooks.Phase phase, MechanismBinding<T, ?> binding, List<Enum<?>> states) {
            owner.addLifecycleBinding(phase, binding, states);
        }

        @Override
        protected void addPhaseExitBinding(MechanismBinding<T, ?> binding, List<Enum<?>> states) {
            owner.addLifecycleExitBinding(binding, states);
        }

        @SafeVarargs
        public final <E extends Enum<E>> HooksSection<T> onStateEnter(
                MechanismBinding<T, E> binding,
                E... states) {
            Objects.requireNonNull(binding, "binding");
            Objects.requireNonNull(states, "states");
            if (states.length == 0) {
                throw new IllegalArgumentException("states must contain at least one state");
            }
            for (E state : states) {
                if (state == null) {
                    continue;
                }
                owner.enterStateHooks.computeIfAbsent(state, unused -> new ArrayList<>()).add(binding);
            }
            return this;
        }

        @SafeVarargs
        public final <E extends Enum<E>> HooksSection<T> onStatePeriodic(
                MechanismBinding<T, E> binding,
                E... states) {
            Objects.requireNonNull(binding, "binding");
            Objects.requireNonNull(states, "states");
            if (states.length == 0) {
                throw new IllegalArgumentException("states must contain at least one state; use always(...) for always-on hooks");
            }
            for (E state : states) {
                if (state == null) {
                    continue;
                }
                owner.stateHooks.computeIfAbsent(state, unused -> new ArrayList<>()).add(binding);
            }
            return this;
        }

        @SafeVarargs
        public final <E extends Enum<E>> HooksSection<T> onStateExit(
                MechanismBinding<T, E> binding,
                E... states) {
            Objects.requireNonNull(binding, "binding");
            Objects.requireNonNull(states, "states");
            if (states.length == 0) {
                throw new IllegalArgumentException("states must contain at least one state; use onAnyStateExit(...) for any-state exit hooks");
            }
            for (E state : states) {
                if (state == null) {
                    continue;
                }
                owner.exitStateHooks.computeIfAbsent(state, unused -> new ArrayList<>()).add(binding);
            }
            return this;
        }

        public <E extends Enum<E>> HooksSection<T> always(MechanismBinding<T, E> binding) {
            Objects.requireNonNull(binding, "binding");
            owner.alwaysHooks.add(binding);
            return this;
        }

        /**
         * Registers a state trigger that will enqueue the provided state when the predicate becomes true.
         *
         * <p>Semantics: the trigger fires on the rising edge (false -> true) to avoid queue spam.
         * If the state is already the goal state or already present in the state queue, it will not be
         * enqueued again.</p>
         */
        public <E extends Enum<E>> HooksSection<T> stateTrigger(
                E state,
                StateTrigger<T, E> trigger) {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(trigger, "trigger");
            owner.stateTriggerBindings.add(new StateTriggerBinding<>(state, (StateTrigger<T, ?>) trigger));
            return this;
        }

        public <E extends Enum<E>> HooksSection<T> onAnyStateExit(MechanismBinding<T, E> binding) {
            Objects.requireNonNull(binding, "binding");
            owner.exitAlwaysHooks.add(binding);
            return this;
        }

        public HooksSection<T> onRobotPeriodic(Consumer<T> hook) {
            Objects.requireNonNull(hook, "hook");
            owner.periodicHooks.add(hook);
            return this;
        }

        /**
         * Registers a hook that runs once when any motor enters a stalled condition.
         *
         * <p>Semantics: fires on the rising edge (not stalled -> stalled) to avoid repeated callbacks
         * while the mechanism remains stalled.</p>
         */
        public HooksSection<T> onStall(Consumer<T> hook) {
            owner.periodicHooks.add(stallEdgeHook(hook));
            return this;
        }

        /**
         * Registers a hook that runs once when the mechanism exits a stalled condition.
         *
         * <p>Semantics: fires on the falling edge (stalled -> not stalled) to avoid repeated callbacks
         * while the mechanism remains not stalled.</p>
         */
        public HooksSection<T> onStallExit(Consumer<T> hook) {
            owner.periodicHooks.add(stallExitHook(hook));
            return this;
        }

        public <E extends Enum<E>> HooksSection<T> onInit(MechanismBinding<T, E> binding) {
            owner.addLifecycleBinding(RobotCoreHooks.Phase.ROBOT_INIT, (MechanismBinding<T, ?>) binding, List.of());
            return this;
        }

        public HooksSection<T> onRobotPeriodic(Consumer<T> hook, double periodMs) {
            Objects.requireNonNull(hook, "hook");
            owner.periodicHookBindings.add(new PeriodicHookBinding<>(hook, periodMs));
            return this;
        }

        /**
         * Registers a stall hook sampled on a fixed cadence.
         *
         * <p>Semantics: fires on the rising edge (not stalled -> stalled).</p>
         */
        public HooksSection<T> onStall(Consumer<T> hook, double periodMs) {
            owner.periodicHookBindings.add(new PeriodicHookBinding<>(stallEdgeHook(hook), periodMs));
            return this;
        }

        /**
         * Registers a stall-exit hook sampled on a fixed cadence.
         *
         * <p>Semantics: fires on the falling edge (stalled -> not stalled).</p>
         */
        public HooksSection<T> onStallExit(Consumer<T> hook, double periodMs) {
            owner.periodicHookBindings.add(new PeriodicHookBinding<>(stallExitHook(hook), periodMs));
            return this;
        }

        public <E extends Enum<E>> HooksSection<T> onStateTransition(
                MechanismTransitionBinding<T, E> binding,
                E from,
                E to) {
            Objects.requireNonNull(binding, "binding");
            Objects.requireNonNull(from, "from");
            Objects.requireNonNull(to, "to");
            owner.transitionHooks.add(new TransitionHookBinding<>(from, to, (MechanismTransitionBinding<T, ?>) binding));
            return this;
        }

        @SafeVarargs
        public final <E extends Enum<E>> HooksSection<T> onStateTransition(
                MechanismTransitionBinding<T, E> binding,
                StateTransitionPair<E>... pairs) {
            Objects.requireNonNull(binding, "binding");
            Objects.requireNonNull(pairs, "pairs");
            for (StateTransitionPair<E> pair : pairs) {
                if (pair == null || pair.from == null || pair.to == null) {
                    continue;
                }
                owner.transitionHooks.add(new TransitionHookBinding<>(pair.from, pair.to, (MechanismTransitionBinding<T, ?>) binding));
            }
            return this;
        }

        public HooksSection<T> stateAction(Function<T, Boolean> action, Enum<?>... states) {
            Objects.requireNonNull(action, "action");
            Objects.requireNonNull(states, "states");
            Arrays.stream(states).forEach(state -> owner.stateActions.put(state, action));
            return this;
        }

        public HooksSection<T> stateAction(Consumer<T> action, Enum<?>... states) {
            Objects.requireNonNull(action, "action");
            return stateAction(mech -> {
                action.accept(mech);
                return false;
            }, states);
        }

        public HooksSection<T> stateActionSuppressMotors(Consumer<T> action, Enum<?>... states) {
            Objects.requireNonNull(action, "action");
            return stateAction(mech -> {
                action.accept(mech);
                return true;
            }, states);
        }

        private Consumer<T> stallEdgeHook(Consumer<T> hook) {
            Objects.requireNonNull(hook, "hook");
            final boolean[] wasStalled = {false};
            return mechanism -> {
                boolean stalled = mechanism.motors().device().isStalling();
                if (stalled && !wasStalled[0]) {
                    hook.accept(mechanism);
                }
                wasStalled[0] = stalled;
            };
        }

        private Consumer<T> stallExitHook(Consumer<T> hook) {
            Objects.requireNonNull(hook, "hook");
            final boolean[] wasStalled = {false};
            return mechanism -> {
                boolean stalled = mechanism.motors().device().isStalling();
                if (!stalled && wasStalled[0]) {
                    hook.accept(mechanism);
                }
                wasStalled[0] = stalled;
            };
        }
    }

    @FunctionalInterface
    public interface StateTrigger<T extends Mechanism, E extends Enum<E>> {
        boolean shouldQueue(MechanismContext<T, E> ctx);
    }

    public record StateTriggerBinding<T extends Mechanism>(Enum<?> state, StateTrigger<T, ?> trigger) {
        public StateTriggerBinding {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(trigger, "trigger");
        }
    }

    private void addLifecycleBinding(RobotCoreHooks.Phase phase, MechanismBinding<T, ?> binding, List<Enum<?>> states) {
        Objects.requireNonNull(phase, "phase");
        Objects.requireNonNull(binding, "binding");
        lifecycleBindings
                .computeIfAbsent(phase, unused -> new ArrayList<>())
                .add(new LifecycleHookBinding<>(binding, sanitizeStates(states)));
    }

    private void addLifecycleExitBinding(MechanismBinding<T, ?> binding, List<Enum<?>> states) {
        Objects.requireNonNull(binding, "binding");
        lifecycleExitBindings.add(new LifecycleHookBinding<>(binding, sanitizeStates(states)));
    }

    private static List<Enum<?>> sanitizeStates(List<Enum<?>> states) {
        if (states == null || states.isEmpty()) {
            return List.of();
        }
        List<Enum<?>> copied = new ArrayList<>(states.size());
        for (Enum<?> state : states) {
            if (state != null) {
                copied.add(state);
            }
        }
        return copied;
    }

    private static boolean isExitPhase(RobotCoreHooks.Phase phase) {
        return phase == RobotCoreHooks.Phase.DISABLED_EXIT
                || phase == RobotCoreHooks.Phase.TELEOP_EXIT
                || phase == RobotCoreHooks.Phase.AUTONOMOUS_EXIT
                || phase == RobotCoreHooks.Phase.TEST_EXIT;
    }

    private static Enum<?> resolveActiveState(Mechanism mechanism) {
        if (!(mechanism instanceof StatefulLike<?> stateful)) {
            return null;
        }
        return stateful.stateMachine().goal();
    }

    /**
     * Runs init hooks with a mechanism-context payload after registration.
     */
    public void runInitHooks(Mechanism mechanism) {
        if (mechanism == null) {
            return;
        }
        runLegacyInitHooks(mechanism);
        runPhaseHooks(mechanism, RobotCoreHooks.Phase.ROBOT_INIT);
    }

    /**
     * Runs phase hooks with optional state filtering.
     */
    @SuppressWarnings("unchecked")
    public void runPhaseHooks(Mechanism mechanism, RobotCoreHooks.Phase phase) {
        if (mechanism == null || phase == null) {
            return;
        }
        T typedMechanism = (T) mechanism;
        Enum<?> activeState = resolveActiveState(mechanism);
        if (isExitPhase(phase)) {
            runLifecycleBindings(typedMechanism, lifecycleExitBindings, activeState);
        }
        runLifecycleBindings(typedMechanism, lifecycleBindings.get(phase), activeState);
    }

    @SuppressWarnings("unchecked")
    private void runLegacyInitHooks(Mechanism mechanism) {
        if (initBindings == null || initBindings.isEmpty()) {
            return;
        }
        T typedMechanism = (T) mechanism;
        Enum<?> activeState = resolveActiveState(mechanism);
        for (MechanismBinding<T, ?> binding : initBindings) {
            if (binding == null) {
                continue;
            }
            applyLifecycleBindingRaw(typedMechanism, binding, activeState);
        }
    }

    private void runLifecycleBindings(
            T mechanism,
            List<LifecycleHookBinding<T>> bindings,
            Enum<?> activeState) {
        if (bindings == null || bindings.isEmpty()) {
            return;
        }
        for (LifecycleHookBinding<T> hook : bindings) {
            if (hook == null || hook.binding() == null || !hook.appliesTo(activeState)) {
                continue;
            }
            applyLifecycleBindingRaw(mechanism, hook.binding(), activeState);
        }
    }

    @SuppressWarnings({ "rawtypes", "unchecked" })
    private void applyLifecycleBindingRaw(
            T mechanism,
            MechanismBinding<T, ?> binding,
            Enum<?> activeState) {
        MechanismBinding rawBinding = (MechanismBinding) binding;
        rawBinding.apply(new LifecycleMechanismContext<>(mechanism, activeState));
    }

        private final class LifecycleMechanismContext<E extends Enum<E>> implements MechanismContext<T, E> {
            private final T mechanism;
            private final E state;
            private final double setpoint;
        private final TypedInputResolver inputsView;

        @SuppressWarnings("unchecked")
        private LifecycleMechanismContext(T mechanism, Enum<?> activeState) {
            this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
            Enum<?> stateCandidate = activeState != null ? activeState : resolveActiveState(mechanism);
            E resolvedState = null;
            if (stateCandidate != null) {
                try {
                    resolvedState = (E) stateCandidate;
                } catch (ClassCastException ignored) {
                    resolvedState = null;
                }
            }
            this.state = resolvedState;
            Double resolvedSetpoint = null;
            if (resolvedState != null) {
                try {
                    resolvedSetpoint = StateSpecAccess.setpoint(resolvedState);
                } catch (NullPointerException ignored) {
                    // Some state providers intentionally leave setpoint unset.
                    resolvedSetpoint = null;
                }
                if (resolvedSetpoint == null) {
                    ca.frc6390.athena.mechanisms.statespec.StateSeed<E> seed = StateSpecAccess.seed(resolvedState);
                    resolvedSetpoint = ca.frc6390.athena.mechanisms.statespec.StateSeedRuntime.doubleSetpoint(seed);
                }
            }
            if (resolvedSetpoint == null) {
                double mechanismSetpoint = mechanism.setpoint();
                if (Double.isFinite(mechanismSetpoint)) {
                    resolvedSetpoint = mechanismSetpoint;
                }
            }
            this.setpoint = resolvedSetpoint != null ? resolvedSetpoint : Double.NaN;
            this.inputsView = new TypedInputResolver(
                    "LifecycleMechanismContext",
                    TypedInputResolver.ValueMode.STRICT,
                    new TypedInputResolver.MutableInputs() {
                        @Override
                        public boolean hasBool(String key) {
                            return LifecycleMechanismContext.this.mechanism.hasMutableBoolValKey(key);
                        }

                        @Override
                        public boolean bool(String key) {
                            return LifecycleMechanismContext.this.mechanism.mutableBoolVal(key);
                        }

                        @Override
                        public boolean hasDouble(String key) {
                            return LifecycleMechanismContext.this.mechanism.hasMutableDblValKey(key);
                        }

                        @Override
                        public double dbl(String key) {
                            return LifecycleMechanismContext.this.mechanism.mutableDblVal(key);
                        }

                        @Override
                        public boolean hasInt(String key) {
                            return LifecycleMechanismContext.this.mechanism.hasMutableIntValKey(key);
                        }

                        @Override
                        public int intVal(String key) {
                            return LifecycleMechanismContext.this.mechanism.mutableIntVal(key);
                        }

                        @Override
                        public boolean hasString(String key) {
                            return LifecycleMechanismContext.this.mechanism.hasMutableStrValKey(key);
                        }

                        @Override
                        public String str(String key) {
                            return LifecycleMechanismContext.this.mechanism.mutableStrVal(key);
                        }

                        @Override
                        public boolean hasPose2d(String key) {
                            return LifecycleMechanismContext.this.mechanism.hasMutablePose2dValKey(key);
                        }

                        @Override
                        public Pose2d pose2d(String key) {
                            return LifecycleMechanismContext.this.mechanism.mutablePose2dVal(key);
                        }

                        @Override
                        public boolean hasPose3d(String key) {
                            return LifecycleMechanismContext.this.mechanism.hasMutablePose3dValKey(key);
                        }

                        @Override
                        public Pose3d pose3d(String key) {
                            return LifecycleMechanismContext.this.mechanism.mutablePose3dVal(key);
                        }
                    },
                    inputs,
                    doubleInputs,
                    intInputs,
                    stringInputs,
                    pose2dInputs,
                    pose3dInputs,
                    objectInputs);
        }

        @Override
        public T mechanism() {
            return mechanism;
        }

        @Override
        public E state() {
            return state;
        }

        @Override
        public double setpoint() {
            return setpoint;
        }

        @Override
        public boolean input(String key) {
            return inputsView.boolVal(key);
        }

        @Override
        public BooleanSupplier inputSupplier(String key) {
            return inputsView.boolSupplier(key);
        }

        @Override
        public double doubleInput(String key) {
            return inputsView.doubleVal(key);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return inputsView.doubleSupplier(key);
        }

        @Override
        public int intVal(String key) {
            return inputsView.intVal(key);
        }

        @Override
        public IntSupplier intValSupplier(String key) {
            return inputsView.intSupplier(key);
        }

        @Override
        public String stringVal(String key) {
            return inputsView.stringVal(key);
        }

        @Override
        public Supplier<String> stringValSupplier(String key) {
            return inputsView.stringSupplier(key);
        }

        @Override
        public Pose2d pose2dVal(String key) {
            return inputsView.pose2dVal(key);
        }

        @Override
        public Supplier<Pose2d> pose2dValSupplier(String key) {
            return inputsView.pose2dSupplier(key);
        }

        @Override
        public Pose3d pose3dVal(String key) {
            return inputsView.pose3dVal(key);
        }

        @Override
        public Supplier<Pose3d> pose3dValSupplier(String key) {
            return inputsView.pose3dSupplier(key);
        }

        @Override
        public <V> V objectInput(String key, Class<V> type) {
            return inputsView.objectVal(key, type);
        }

        @Override
        public <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
            return inputsView.objectSupplier(key, type);
        }
    }


    public MechanismConfigRecord data() {
        synchronizeEncoderData();
        return data;
    }

    public MechanismConfig<T> data(MechanismConfigRecord data) {
        this.data = data != null ? data : MechanismConfigRecord.defaults();
        loadEncoderData(this.data);
        return this;
    }

    /**
     * Assigns the {@link edu.wpi.first.wpilibj2.command.Subsystem} name of the mechanism created by
     * this config. This is strongly recommended so RobotCore can enforce uniqueness and so teams can
     * retrieve mechanisms by name without relying on Java class names.
     */
    public MechanismConfig<T> named(String name) {
        this.mechanismName = name;
        return this;
    }

    public String name() {
        return mechanismName;
    }

    /**
     * Enables/disables this mechanism config.
     * When disabled, {@link #build()} still returns a mechanism instance so callers can keep stable
     * references, but it is forced into an inert no-op runtime mode.
     */
    public MechanismConfig<T> disabled(boolean disabled) {
        this.disabled = disabled;
        return this;
    }

    public boolean disabled() {
        return disabled;
    }

    public Map<Enum<?>, Function<T, Boolean>> stateActions() {
        return Map.copyOf(stateActions);
    }

    public Map<Enum<?>, List<MechanismBinding<T, ?>>> enterStateHooks() {
        return immutableMapOfLists(enterStateHooks);
    }

    public Map<Enum<?>, List<MechanismBinding<T, ?>>> stateHooks() {
        return immutableMapOfLists(stateHooks);
    }

    public Map<Enum<?>, List<MechanismBinding<T, ?>>> exitStateHooks() {
        return immutableMapOfLists(exitStateHooks);
    }

    public List<TransitionHookBinding<T>> transitionHooks() {
        return List.copyOf(transitionHooks);
    }

    public List<MechanismBinding<T, ?>> alwaysHooks() {
        return List.copyOf(alwaysHooks);
    }

    public List<MechanismBinding<T, ?>> exitAlwaysHooks() {
        return List.copyOf(exitAlwaysHooks);
    }

    public List<StateTriggerBinding<T>> stateTriggerBindings() {
        return List.copyOf(stateTriggerBindings);
    }

    public Map<String, BooleanSupplier> inputs() {
        return Map.copyOf(inputs);
    }

    public Map<String, DoubleSupplier> doubleInputs() {
        return Map.copyOf(doubleInputs);
    }

    public Map<String, IntSupplier> intInputs() {
        return Map.copyOf(intInputs);
    }

    public Map<String, Supplier<String>> stringInputs() {
        return Map.copyOf(stringInputs);
    }

    public Map<String, Supplier<Pose2d>> pose2dInputs() {
        return Map.copyOf(pose2dInputs);
    }

    public Map<String, Supplier<Pose3d>> pose3dInputs() {
        return Map.copyOf(pose3dInputs);
    }

    public Map<String, Supplier<?>> objectInputs() {
        return Map.copyOf(objectInputs);
    }

    public MechanismSimulationConfig simulationConfig() {
        return simulationConfig;
    }

    public ElevatorSimulationParameters elevatorSimulationParameters() {
        return elevatorSimulationParameters;
    }

    public ArmSimulationParameters armSimulationParameters() {
        return armSimulationParameters;
    }

    public SimpleMotorSimulationParameters simpleMotorSimulationParameters() {
        return simpleMotorSimulationParameters;
    }

    public Supplier<TurretMechanism.FieldHeadingVisualization> turretHeadingVisualization() {
        return turretHeadingVisualization;
    }

    public MechanismSensorSimulationConfig sensorSimulationConfig() {
        return sensorSimulationConfig;
    }

    public List<Consumer<T>> periodicHooks() {
        return List.copyOf(periodicHooks);
    }

    public List<PeriodicHookBinding<T>> periodicHookBindings() {
        return List.copyOf(periodicHookBindings);
    }

    public Map<String, MechanismEncoderSource> resolveEncoderSources(MotorControllerGroup motors) {
        LinkedHashMap<String, MechanismEncoderSource> resolved = new LinkedHashMap<>();
        for (EncoderSourceSpec spec : encoderSourceSpecs.values()) {
            resolved.put(spec.name(), resolveEncoderSource(spec, resolved, motors));
        }
        return Map.copyOf(resolved);
    }

    public String resolvePositionSourceName(Map<String, MechanismEncoderSource> encoderSources) {
        return resolveSelectedEncoderSourceName(controlPositionSourceName, encoderSources, "position");
    }

    public String resolveVelocitySourceName(Map<String, MechanismEncoderSource> encoderSources) {
        return resolveSelectedEncoderSourceName(controlVelocitySourceName, encoderSources, "velocity");
    }

    public String resolveAbsoluteSourceName(Map<String, MechanismEncoderSource> encoderSources) {
        return resolveSelectedEncoderSourceName(controlAbsoluteSourceName, encoderSources, "absolute");
    }

    public Map<String, Boolean> mutableBoolInputDefaults() {
        return Map.copyOf(mutableBoolInputDefaults);
    }

    public Map<String, Double> mutableDoubleInputDefaults() {
        return Map.copyOf(mutableDoubleInputDefaults);
    }

    public Map<String, Integer> mutableIntInputDefaults() {
        return Map.copyOf(mutableIntInputDefaults);
    }

    public Map<String, String> mutableStringInputDefaults() {
        return Map.copyOf(mutableStringInputDefaults);
    }

    public Map<String, Pose2d> mutablePose2dInputDefaults() {
        return Map.copyOf(mutablePose2dInputDefaults);
    }

    public Map<String, Pose3d> mutablePose3dInputDefaults() {
        return Map.copyOf(mutablePose3dInputDefaults);
    }

    public Map<String, PidProfile> controlLoopPidProfiles() {
        return Map.copyOf(controlLoopPidProfiles);
    }

    public Map<String, BangBangProfile> controlLoopBangBangProfiles() {
        return Map.copyOf(controlLoopBangBangProfiles);
    }

    public Map<String, FeedforwardProfile> controlLoopFeedforwardProfiles() {
        return Map.copyOf(controlLoopFeedforwardProfiles);
    }

    public FeedforwardProfile controlLoopFeedforwardProfile(String name) {
        if (name == null || name.isBlank()) {
            return null;
        }
        return controlLoopFeedforwardProfiles.get(name.trim());
    }

    public FeedforwardProfile mechanismFeedforwardProfile(FeedforwardType type) {
        FeedforwardType requested = type != null ? type : FeedforwardType.SIMPLE;
        String[] preferred = {"ff", "feedforward", "default", "main"};
        for (String key : preferred) {
            for (Map.Entry<String, FeedforwardProfile> entry : controlLoopFeedforwardProfiles.entrySet()) {
                if (entry == null || entry.getKey() == null || entry.getValue() == null) {
                    continue;
                }
                if (!key.equalsIgnoreCase(entry.getKey().trim())) {
                    continue;
                }
                FeedforwardProfile profile = entry.getValue();
                if (profile.type() == requested) {
                    return profile;
                }
            }
        }
        return null;
    }

    public List<ControlLoopBinding<T>> controlLoops() {
        return List.copyOf(controlLoops);
    }

    private static <K, V> Map<K, List<V>> immutableMapOfLists(Map<K, List<V>> source) {
        Map<K, List<V>> copy = new HashMap<>();
        for (Map.Entry<K, List<V>> entry : source.entrySet()) {
            copy.put(entry.getKey(), List.copyOf(entry.getValue()));
        }
        return Map.copyOf(copy);
    }

    private MechanismEncoderSource resolveEncoderSource(
            EncoderSourceSpec spec,
            Map<String, MechanismEncoderSource> resolved,
            MotorControllerGroup motors) {
        return switch (spec.kind()) {
            case HARDWARE -> resolveHardwareEncoderSource(spec, motors);
            case VIRTUAL -> resolveVirtualEncoderSource(spec);
            case CRT -> resolveCrtEncoderSource(spec, resolved);
        };
    }

    private MechanismEncoderSource resolveHardwareEncoderSource(
            EncoderSourceSpec spec,
            MotorControllerGroup motors) {
        Encoder device;
        if (spec.athenaType() != null && spec.athenaType().isInternal()) {
            String resolvedCanbus = resolveEncoderCanbus(spec.canbus());
            Encoder rawDevice = resolveInternalEncoder(spec.id(), resolvedCanbus, motors);
            EncoderConfig config = EncoderConfig.create();
            config.hardware()
                    .id(spec.id())
                    .canbus(resolvedCanbus)
                    .inverted(spec.id() < 0);
            config.measurement()
                    .gearRatio(spec.gearRatio())
                    .conversion(spec.conversion())
                    .conversionOffset(spec.offset())
                    .offset(0.0);
            device = rawDevice != null ? new EncoderAdapter(rawDevice, config) : null;
            if (device == null) {
                throw new IllegalStateException(
                        "Encoder source '" + spec.name() + "' uses INTERNAL, but no matching motor encoder exists for id "
                                + Math.abs(spec.id()) + " on canbus '" + resolvedCanbus + "'.");
            }
        } else {
            EncoderType resolvedType = spec.type();
            if (resolvedType == null && spec.athenaType() != null) {
                resolvedType = spec.athenaType().resolve();
            }
            if (resolvedType == null) {
                throw new IllegalStateException(
                        "Encoder source '" + spec.name() + "' is missing an encoder type.");
            }
            EncoderConfig config = EncoderConfig.create(resolvedType, spec.id());
            config.hardware().canbus(resolveEncoderCanbus(spec.canbus()));
            config.measurement()
                    .gearRatio(spec.gearRatio())
                    .conversion(spec.conversion())
                    .conversionOffset(spec.offset())
                    .offset(0.0);
            device = HardwareFactories.encoder(config);
        }
        MechanismEncoderUnit unit = spec.unit() != null ? spec.unit() : MechanismEncoderUnit.ROTATIONS;
        return new MechanismEncoderSource(spec.name(), device, unit, spec.wrapsEvery());
    }

    private MechanismEncoderSource resolveVirtualEncoderSource(EncoderSourceSpec spec) {
        DoubleSupplier position = spec.positionSupplier();
        DoubleSupplier absolute = spec.absoluteSupplier() != null ? spec.absoluteSupplier() : position;
        Encoder device = new SupplierEncoder(
                EncoderConfig.create(),
                position,
                spec.velocitySupplier(),
                absolute);
        MechanismEncoderUnit unit = spec.unit() != null ? spec.unit() : MechanismEncoderUnit.ROTATIONS;
        return new MechanismEncoderSource(spec.name(), device, unit, spec.wrapsEvery());
    }

    private MechanismEncoderSource resolveCrtEncoderSource(
            EncoderSourceSpec spec,
            Map<String, MechanismEncoderSource> resolved) {
        if (spec.crtInputs().isEmpty()) {
            throw new IllegalStateException("CRT encoder source '" + spec.name() + "' has no inputs.");
        }
        List<ChineseRemainderEncoder.Input> inputs = new ArrayList<>();
        MechanismEncoderUnit unit = spec.unit();
        double commonSpan = Double.NaN;
        for (CrtInputSpec inputSpec : spec.crtInputs()) {
            MechanismEncoderSource source = resolved.get(inputSpec.sourceName());
            if (source == null) {
                throw new IllegalStateException(
                        "CRT encoder source '" + spec.name() + "' references unknown source '" + inputSpec.sourceName() + "'.");
            }
            if (!Double.isFinite(source.wrapsEvery()) || source.wrapsEvery() <= 0.0) {
                throw new IllegalStateException(
                        "CRT encoder source '" + spec.name() + "' requires wrapsEvery(...) on input '" + source.name() + "'.");
            }
            if (unit == null || unit == MechanismEncoderUnit.ENCODER_UNITS) {
                unit = source.unit();
            } else if (source.unit() != unit) {
                throw new IllegalStateException(
                        "CRT encoder source '" + spec.name() + "' mixes units between inputs.");
            }
            if (inputSpec.modulus() <= 0) {
                throw new IllegalStateException(
                        "CRT encoder source '" + spec.name() + "' has non-positive modulus for input '" + source.name() + "'.");
            }
            double span = source.wrapsEvery() * inputSpec.modulus();
            if (!Double.isFinite(commonSpan)) {
                commonSpan = span;
            } else if (Math.abs(commonSpan - span) > 1e-6) {
                throw new IllegalStateException(
                        "CRT encoder source '" + spec.name() + "' has incompatible gearing between inputs.");
            }
            inputs.add(new ChineseRemainderEncoder.Input(source, inputSpec.modulus()));
        }
        validateCrtModuli(spec.name(), spec.crtInputs(), commonSpan, spec.validMin(), spec.validMax());
        Encoder device = new ChineseRemainderEncoder(
                EncoderConfig.create(),
                inputs,
                spec.validMin(),
                spec.validMax());
        return new MechanismEncoderSource(spec.name(), device, unit, commonSpan);
    }

    private void validateCrtModuli(
            String name,
            List<CrtInputSpec> inputs,
            double commonSpan,
            double validMin,
            double validMax) {
        for (int i = 0; i < inputs.size(); i++) {
            for (int j = i + 1; j < inputs.size(); j++) {
                int a = inputs.get(i).modulus();
                int b = inputs.get(j).modulus();
                if (greatestCommonDivisor(a, b) != 1) {
                    throw new IllegalStateException(
                            "CRT encoder source '" + name + "' requires pairwise-coprime moduli, but got "
                                    + a + " and " + b + ".");
                }
            }
        }
        if (Double.isFinite(validMin) && Double.isFinite(validMax) && validMax > validMin
                && Double.isFinite(commonSpan) && (validMax - validMin) > (commonSpan + 1e-6)) {
            throw new IllegalStateException(
                    "CRT encoder source '" + name + "' validRange exceeds the unique span implied by the gearing.");
        }
    }

    private static int greatestCommonDivisor(int a, int b) {
        int left = Math.abs(a);
        int right = Math.abs(b);
        while (right != 0) {
            int next = left % right;
            left = right;
            right = next;
        }
        return Math.max(left, 1);
    }

    private String resolveEncoderCanbus(String explicitCanbus) {
        if (explicitCanbus != null && !explicitCanbus.isBlank()) {
            return explicitCanbus;
        }
        return data.canbus();
    }

    private static String resolveSelectedEncoderSourceName(
            String configuredName,
            Map<String, MechanismEncoderSource> encoderSources,
            String role) {
        if (encoderSources == null || encoderSources.isEmpty()) {
            return null;
        }
        if (configuredName == null || configuredName.isBlank()) {
            return encoderSources.keySet().iterator().next();
        }
        if (!encoderSources.containsKey(configuredName)) {
            throw new IllegalStateException(
                    "Control " + role + " source '" + configuredName + "' is not a configured encoder source.");
        }
        return configuredName;
    }

    private static Encoder resolveInternalEncoder(int id, String canbus, MotorControllerGroup motors) {
        if (motors == null) {
            return null;
        }
        int targetId = Math.abs(id);
        Encoder match = null;
        for (MotorController controller : motors.getControllers()) {
            if (controller == null || controller.getId() != targetId) {
                continue;
            }
            if (canbus != null && !canbus.isBlank()
                    && !canbus.equalsIgnoreCase(controller.getCanbus())) {
                continue;
            }
            if (match != null) {
                throw new IllegalStateException(
                        "Multiple motors matched INTERNAL encoder id=" + targetId
                                + (canbus != null && !canbus.isBlank() ? " canbus=" + canbus : "")
                                + ". Specify canbus explicitly.");
            }
            match = controller.getEncoder();
        }
        return match;
    }

    private void loadEncoderData(MechanismConfigRecord record) {
        encoderSourceSpecs.clear();
        controlPositionSourceName = null;
        controlVelocitySourceName = null;
        controlAbsoluteSourceName = null;
        if (record == null) {
            return;
        }
        List<MechanismEncoderConfig> encoders = record.encoders();
        if (encoders != null) {
            for (MechanismEncoderConfig cfg : encoders) {
                if (cfg == null) {
                    continue;
                }
                String name = EncodersSection.normalizeSourceName(cfg.name());
                if (encoderSourceSpecs.containsKey(name)) {
                    throw new IllegalArgumentException("duplicate encoder source in config record: " + name);
                }
                EncoderSourceBuilder builder = new EncoderSourceBuilder(this, name);
                applyEncoderData(builder, cfg);
                encoderSourceSpecs.put(name, builder.build());
            }
        }
        controlPositionSourceName = normalizeOptionalSourceName(record.positionSource());
        controlVelocitySourceName = normalizeOptionalSourceName(record.velocitySource());
        controlAbsoluteSourceName = normalizeOptionalSourceName(record.absoluteSource());
    }

    private static String normalizeOptionalSourceName(String sourceName) {
        if (sourceName == null || sourceName.isBlank()) {
            return null;
        }
        return EncodersSection.normalizeSourceName(sourceName);
    }

    private void applyEncoderData(EncoderSourceBuilder builder, MechanismEncoderConfig cfg) {
        String source = cfg.source() != null ? cfg.source().trim() : "";
        if (source.isBlank()) {
            throw new IllegalArgumentException("encoder source '" + cfg.name() + "' is missing a source type");
        }
        String normalizedSource = source.toLowerCase(Locale.ROOT);
        if ("virtual".equals(normalizedSource)) {
            throw new IllegalArgumentException(
                    "encoder source '" + cfg.name() + "' uses source=virtual, which is code-only and cannot be loaded from data");
        } else if ("crt".equals(normalizedSource)) {
            builder.crt(crt -> {
                if (cfg.crtInputs() == null || cfg.crtInputs().isEmpty()) {
                    throw new IllegalArgumentException(
                            "encoder source '" + cfg.name() + "' requires crtInputs when source=crt");
                }
                for (MechanismEncoderCrtInputConfig input : cfg.crtInputs()) {
                    if (input == null) {
                        continue;
                    }
                    crt.input(input.source(), input.modulus() != null ? input.modulus() : 0);
                }
                if (cfg.validMin() != null && cfg.validMax() != null) {
                    crt.validRange(cfg.validMin(), cfg.validMax());
                }
            });
        } else {
            int id = cfg.id() != null ? Math.abs(cfg.id()) : 0;
            if (cfg.id() == null) {
                throw new IllegalArgumentException("encoder source '" + cfg.name() + "' requires an id");
            }
            int signedId = Boolean.TRUE.equals(cfg.inverted()) ? -id : id;
            if ("internal".equals(normalizedSource)) {
                builder.encoder(AthenaEncoder.INTERNAL, signedId);
            } else {
                boolean matchedAthena = false;
                for (AthenaEncoder athenaEncoder : AthenaEncoder.values()) {
                    if (athenaEncoder.name().equalsIgnoreCase(source) && !athenaEncoder.isInternal()) {
                        builder.encoder(athenaEncoder, signedId);
                        matchedAthena = true;
                        break;
                    }
                }
                if (!matchedAthena) {
                    builder.encoder(ca.frc6390.athena.hardware.encoder.EncoderRegistry.get().encoder(source), signedId);
                }
            }
        }
        if (cfg.canbus() != null && !cfg.canbus().isBlank()) {
            builder.canbus(cfg.canbus());
        }
        if (cfg.gearRatio() != null) {
            builder.gearRatio(cfg.gearRatio());
        }
        if (cfg.conversion() != null) {
            builder.conversion(cfg.conversion());
        }
        if (cfg.offset() != null) {
            builder.offset(cfg.offset());
        }
        if (cfg.unit() != null && !cfg.unit().isBlank()) {
            builder.unit(parseEncoderUnit(cfg.unit()));
        }
        if (cfg.wrapsEvery() != null) {
            builder.wrapsEvery(cfg.wrapsEvery());
        }
    }

    private static MechanismEncoderUnit parseEncoderUnit(String raw) {
        if (raw == null || raw.isBlank()) {
            return null;
        }
        return switch (raw.trim().toLowerCase(Locale.ROOT)) {
            case "rotations" -> MechanismEncoderUnit.ROTATIONS;
            case "radians" -> MechanismEncoderUnit.RADIANS;
            case "degrees" -> MechanismEncoderUnit.DEGREES;
            case "encoder_units", "encoder-units", "encoderunits" -> MechanismEncoderUnit.ENCODER_UNITS;
            default -> throw new IllegalArgumentException("unknown encoder unit: " + raw);
        };
    }

    private void updateData(Consumer<MechanismConfigRecord.Builder> mutator) {
        MechanismConfigRecord.Builder builder = data.toBuilder();
        mutator.accept(builder);
        data = builder.build();
    }

    private void synchronizeEncoderData() {
        refreshEncoderSummaryData();
        MechanismConfigRecord.Builder builder = data.toBuilder()
                .encoders(exportEncoderConfigs())
                .positionSource(resolveConfiguredSourceName(controlPositionSourceName))
                .velocitySource(resolveConfiguredSourceName(controlVelocitySourceName))
                .absoluteSource(resolveConfiguredSourceName(controlAbsoluteSourceName));
        data = builder.build();
    }

    private String resolveConfiguredSourceName(String configuredName) {
        if (encoderSourceSpecs.isEmpty()) {
            return null;
        }
        if (configuredName == null || configuredName.isBlank()) {
            return encoderSourceSpecs.keySet().iterator().next();
        }
        if (!encoderSourceSpecs.containsKey(configuredName)) {
            throw new IllegalStateException("configured encoder source '" + configuredName + "' is not defined");
        }
        return configuredName;
    }

    private List<MechanismEncoderConfig> exportEncoderConfigs() {
        List<MechanismEncoderConfig> configs = new ArrayList<>();
        for (EncoderSourceSpec spec : encoderSourceSpecs.values()) {
            configs.add(exportEncoderConfig(spec));
        }
        return configs;
    }

    private static MechanismEncoderConfig exportEncoderConfig(EncoderSourceSpec spec) {
        String source;
        Integer id = null;
        Boolean inverted = null;
        List<MechanismEncoderCrtInputConfig> crtInputs = null;
        switch (spec.kind()) {
            case CRT -> {
                source = "crt";
                crtInputs = new ArrayList<>();
                for (CrtInputSpec input : spec.crtInputs()) {
                    crtInputs.add(new MechanismEncoderCrtInputConfig(input.sourceName(), input.modulus()));
                }
            }
            case HARDWARE -> {
                if (spec.athenaType() != null) {
                    source = spec.athenaType().name().toLowerCase(Locale.ROOT);
                } else if (spec.type() != null) {
                    source = spec.type().getKey();
                } else {
                    source = null;
                }
                id = Math.abs(spec.id());
                inverted = spec.id() < 0;
            }
            case VIRTUAL -> source = "virtual";
            default -> throw new IllegalStateException("unknown encoder source kind: " + spec.kind());
        }
        return new MechanismEncoderConfig(
                spec.name(),
                source,
                id,
                spec.canbus(),
                inverted,
                spec.gearRatio(),
                spec.conversion(),
                spec.offset(),
                exportEncoderUnit(spec.unit()),
                Double.isFinite(spec.wrapsEvery()) ? spec.wrapsEvery() : null,
                Double.isFinite(spec.validMin()) ? spec.validMin() : null,
                Double.isFinite(spec.validMax()) ? spec.validMax() : null,
                crtInputs != null && !crtInputs.isEmpty() ? List.copyOf(crtInputs) : null);
    }

    private static String exportEncoderUnit(MechanismEncoderUnit unit) {
        if (unit == null) {
            return null;
        }
        return switch (unit) {
            case ROTATIONS -> "rotations";
            case RADIANS -> "radians";
            case DEGREES -> "degrees";
            case ENCODER_UNITS -> "encoder_units";
        };
    }

    private void refreshEncoderSummaryData() {
        if (encoderSourceSpecs.isEmpty()) {
            return;
        }
        EncoderSourceSpec summary = null;
        if (controlPositionSourceName != null) {
            summary = encoderSourceSpecs.get(controlPositionSourceName);
        }
        if (summary == null) {
            summary = encoderSourceSpecs.values().iterator().next();
        }
        if (summary == null) {
            return;
        }
        EncoderSourceSpec resolved = summary;
        updateData(builder -> builder
                .encoderGearRatio(resolved.gearRatio())
                .encoderConversion(resolved.conversion())
                .encoderConversionOffset(resolved.offset())
                .encoderOffset(0.0));
    }

    /**
     * Creates a configuration builder that instantiates a plain {@link Mechanism} with no additional
     * behaviors. Useful for simple rollers or mocked subsystems.
     */
    public static MechanismConfig<Mechanism> generic(){
        return custom(Mechanism::new);
    }

    /**
     * Named variant of {@link #generic()}.
     */
    public static MechanismConfig<Mechanism> generic(String name) {
        return generic().named(name);
    }

    /**
     * Creates a state-aware mechanism configuration where the mechanism itself owns a state machine
     * enum that produces setpoints.
     *
     * @param initialState starting state when the mechanism is constructed
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulMechanism<E>> stateMachineGeneric(E initialState){
        return custom(config -> new StatefulMechanism<>(config, initialState));
    }

    /**
     * Named variant of {@link #stateMachineGeneric(Enum)}.
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulMechanism<E>> stateMachineGeneric(String name, E initialState) {
        return stateMachineGeneric(initialState).named(name);
    }

    /**
     * Creates a state-aware mechanism configuration using a caller-supplied factory.
     *
     * @param factory builder that receives this config and the initial state
     * @param initialState starting state when the mechanism is constructed
     * @param <E> state enum type that provides setpoints
     * @param <T> concrete mechanism type returned from the factory
     */
    public static <E extends Enum<E>, T extends StatefulMechanism<E>> MechanismConfig<T> stateMachine(BiFunction<MechanismConfig<T>, E, T> factory, E initialState) {
        return custom(config -> factory.apply(config, initialState));
    }

    /**
     * Creates a configuration that will build an {@link ElevatorMechanism} with optional feedforward control.
     *
     * <p>Control behavior is defined via explicit periodic/custom control loops.</p>
     */
    public static MechanismConfig<ElevatorMechanism> elevator() {
        return custom(config -> new ElevatorMechanism(config));
    }

    /**
     * Named variant of {@link #elevator()}.
     */
    public static MechanismConfig<ElevatorMechanism> elevator(String name) {
        return elevator().named(name);
    }

    /**
     * Creates a configuration that wraps a caller-supplied elevator mechanism factory.
     */
    public static <T extends ElevatorMechanism> MechanismConfig<T> elevator(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful elevator configuration with an initial state.
     *
     * @param initialState state machine starting point
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulElevatorMechanism<E>> statefulElevator(E initialState) {
        return custom(config -> new StatefulElevatorMechanism<>(config, initialState));
    }

    /**
     * Named variant of {@link #statefulElevator(Enum)}.
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulElevatorMechanism<E>> statefulElevator(
            String name,
            E initialState) {
        return statefulElevator(initialState).named(name);
    }

    /**
     * Builds a stateful elevator configuration backed by a caller-supplied factory.
     */
    public static <E extends Enum<E>, T extends StatefulElevatorMechanism<E>> MechanismConfig<T> statefulElevator(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful arm configuration with an initial state.
     *
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulArmMechanism<E>> statefulArm(E initialState) {
        return custom(config -> new StatefulArmMechanism<>(config, initialState));
    }

    /**
     * Named variant of {@link #statefulArm(Enum)}.
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulArmMechanism<E>> statefulArm(
            String name,
            E initialState) {
        return statefulArm(initialState).named(name);
    }

    /**
     * Builds a stateful arm configuration backed by a caller-supplied factory.
     */
    public static <E extends Enum<E>, T extends StatefulArmMechanism<E>> MechanismConfig<T> statefulArm(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful turret configuration (simple motor with continuous rotation).
     *
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulTurretMechanism<E>> statefulTurret(E initialState) {
        MechanismConfig<StatefulTurretMechanism<E>> cfg =
                MechanismConfig.<StatefulTurretMechanism<E>>custom(
                        config -> new StatefulTurretMechanism<>(config, initialState));
        cfg.autoContinuousPidForUnboundedTurret = true;
        return cfg;
    }

    /**
     * Named variant of {@link #statefulTurret(Enum)}.
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulTurretMechanism<E>> statefulTurret(
            String name,
            E initialState) {
        return statefulTurret(initialState).named(name);
    }

    /**
     * Builds a stateful turret configuration backed by a caller-supplied factory.
     *
     * @param factory constructor logic for the concrete mechanism
     * @param <E> state enum type that provides setpoints
     * @param <T> concrete mechanism type created by the factory
     */
    public static <E extends Enum<E>, T extends StatefulTurretMechanism<E>> MechanismConfig<T> statefulTurret(Function<MechanismConfig<T>, T> factory) {
        MechanismConfig<T> cfg = MechanismConfig.<T>custom(factory);
        cfg.autoContinuousPidForUnboundedTurret = true;
        return cfg;
    }

    /**
     * Builds a stateful flywheel configuration with an initial state.
     *
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulFlywheelMechanism<E>> statefulFlywheel(E initialState) {
        return custom(config -> new StatefulFlywheelMechanism<>(config, initialState));
    }

    /**
     * Named variant of {@link #statefulFlywheel(Enum)}.
     */
    public static <E extends Enum<E>> MechanismConfig<StatefulFlywheelMechanism<E>> statefulFlywheel(
            String name,
            E initialState) {
        return statefulFlywheel(initialState).named(name);
    }

    /**
     * Builds a stateful flywheel configuration backed by a caller-supplied factory.
     */
    public static <E extends Enum<E>, T extends StatefulFlywheelMechanism<E>> MechanismConfig<T> statefulFlywheel(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Creates a turret configuration that yields a {@link TurretMechanism}.
     */
    public static MechanismConfig<TurretMechanism> turret() {
        MechanismConfig<TurretMechanism> cfg =
                MechanismConfig.<TurretMechanism>custom(config -> new TurretMechanism(config));
        cfg.autoContinuousPidForUnboundedTurret = true;
        return cfg;
    }

    /**
     * Named variant of {@link #turret()}.
     */
    public static MechanismConfig<TurretMechanism> turret(String name) {
        return turret().named(name);
    }

    /**
     * Creates a turret configuration backed by a caller-supplied factory.
     *
     * @param factory constructor logic for the concrete mechanism
     * @param <T> concrete mechanism type created by the factory
     */
    public static <T extends TurretMechanism> MechanismConfig<T> turret(Function<MechanismConfig<T>, T> factory) {
        MechanismConfig<T> cfg = MechanismConfig.<T>custom(factory);
        cfg.autoContinuousPidForUnboundedTurret = true;
        return cfg;
    }

    static MechanismConfigRecord applyAutoContinuousPidForUnboundedTurret(MechanismConfigRecord cfg) {
        if (cfg == null || cfg.pidContinous()) {
            return cfg;
        }

        // Only apply defaults when bounds are unset (unbounded turret).
        if (Double.isFinite(cfg.minBound()) && Double.isFinite(cfg.maxBound()) && cfg.maxBound() > cfg.minBound()) {
            return cfg;
        }

        double conversion = cfg.encoderConversion();
        if (!Double.isFinite(conversion) || conversion <= 0.0) {
            return cfg;
        }

        double min = -conversion / 2.0;
        double max = conversion / 2.0;
        return cfg.toBuilder()
                .pidContinous(true)
                .continousMin(min)
                .continousMax(max)
                .build();
    }

    /**
     * Creates a flywheel configuration that yields a {@link FlywheelMechanism}.
     */
    public static MechanismConfig<FlywheelMechanism> flywheel() {
        return custom(config -> new FlywheelMechanism(config));
    }

    /**
     * Named variant of {@link #flywheel()}.
     */
    public static MechanismConfig<FlywheelMechanism> flywheel(String name) {
        return flywheel().named(name);
    }

    /**
     * Creates a flywheel configuration backed by a caller-supplied factory.
     */
    public static <T extends FlywheelMechanism> MechanismConfig<T> flywheel(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Creates a configuration that builds a classic {@link ArmMechanism} with optional feedforward control.
     *
     * <p>Control behavior is defined via explicit periodic/custom control loops.</p>
     */
    public static MechanismConfig<ArmMechanism> arm() {
        return custom(config -> new ArmMechanism(config));
    }

    /**
     * Named variant of {@link #arm()}.
     */
    public static MechanismConfig<ArmMechanism> arm(String name) {
        return arm().named(name);
    }

    /**
     * Creates an arm configuration backed by a caller-supplied factory.
     */
    public static <T extends ArmMechanism> MechanismConfig<T> arm(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Creates a configuration builder using a custom mechanism factory.
     *
     * @param factory constructor logic that consumes the populated configuration
     * @param <T> mechanism type created by the factory
     */
    public static <T extends Mechanism> MechanismConfig<T> custom(Function<MechanismConfig<T>, T> factory){
        MechanismConfig<T> cfg = new MechanismConfig<>();
        cfg.factory = factory;
        return cfg;
    }

    /**
     * Named variant of {@link #custom(Function)}. Prefer this overload so every mechanism config has
     * a stable, unique name that RobotCore can validate during registration.
     */
    public static <T extends Mechanism> MechanismConfig<T> custom(String name, Function<MechanismConfig<T>, T> factory) {
        return custom(factory).named(name);
    }

    /**
     * Sets a debounce delay between state-machine transitions.
     *
     * @param delay minimum delay (seconds) between transitions
     * @return this config for chaining
     */
    public MechanismConfig<T> stateMachineDelay(double delay){
        updateData(builder -> builder.stateMachineDelay(delay));
        return this;
    }

    /**
     * Clamps all setpoints to the provided bounds.
     *
     * @param min minimum setpoint value
     * @param max maximum setpoint value
     * @return this config for chaining
     */
    private MechanismConfig<T> setBounds(double min, double max) {
        updateData(builder -> builder.minBound(min).maxBound(max));
        return this;
    }

    /**
     * Clears any configured setpoint bounds.
     *
     * @return this config for chaining
     */
    private MechanismConfig<T> clearBounds() {
        updateData(builder -> builder.minBound(Double.NaN).maxBound(Double.NaN));
        return this;
    }

    /**
     * Attaches a {@link StateGraph} that enumerates allowed transitions and guards between states.
     * Only applies when the built mechanism extends {@link StatefulMechanism}.
     *
     * @param stateGraph transition graph shared by the mechanism's state machine
     * @param <E> enum backing the state machine
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public <E extends Enum<E>> MechanismConfig<T> stateGraph(StateGraph<E> stateGraph){
        this.stateGraph = Objects.requireNonNull(stateGraph, "stateGraph");
        return this;
    }

    @FunctionalInterface
    public interface MechanismBinding<M extends Mechanism, E extends Enum<E>>
            extends Consumer<MechanismContext<M, E>> {
        void apply(MechanismContext<M, E> context);

        @Override
        default void accept(MechanismContext<M, E> context) {
            apply(context);
        }
    }

    @FunctionalInterface
    public interface MechanismTransitionBinding<M extends Mechanism, E extends Enum<E>> {
        void apply(MechanismContext<M, E> context, E from, E to);
    }

    @FunctionalInterface
    public interface MechanismControlLoop<M extends Mechanism>
            extends ToDoubleFunction<MechanismControlContext<M>> {
        double calculate(MechanismControlContext<M> context);

        @Override
        default double applyAsDouble(MechanismControlContext<M> context) {
            return calculate(context);
        }
    }

    public record ControlLoopBinding<M extends Mechanism>(
            String name,
            double periodSeconds,
            MechanismControlLoop<M> loop) { }

    public record StateTransitionPair<E extends Enum<E>>(E from, E to) { }

    public record TransitionHookBinding<M extends Mechanism>(
            Enum<?> from,
            Enum<?> to,
            MechanismTransitionBinding<M, ?> binding) {
        public TransitionHookBinding {
            Objects.requireNonNull(from, "from");
            Objects.requireNonNull(to, "to");
            Objects.requireNonNull(binding, "binding");
        }
    }

    public record PidAutotunerConfig(
            boolean enabled,
            String dashboardPath,
            MechanismPidAutotunerProgram program) {
        public static PidAutotunerConfig defaults() {
            return new PidAutotunerConfig(false, null, null);
        }

        public PidAutotunerConfig withEnabled(boolean enabled) {
            return new PidAutotunerConfig(enabled, dashboardPath, program);
        }

        public PidAutotunerConfig withDashboardPath(String dashboardPath) {
            return new PidAutotunerConfig(enabled, dashboardPath, program);
        }

        public PidAutotunerConfig withProgram(MechanismPidAutotunerProgram program) {
            return new PidAutotunerConfig(enabled, dashboardPath, program);
        }
    }

    public record PidProfile(
            OutputType outputType,
            double kP,
            double kI,
            double kD,
            double iZone,
            double tolerance,
            double maxVelocity,
            double maxAcceleration,
            PidAutotunerConfig autotuner,
            MechanismInputSource inputSource,
            MechanismSetpointSource setpointSource) {
        public PidProfile {
            autotuner = autotuner != null ? autotuner : PidAutotunerConfig.defaults();
            inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
            setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        }

        public PidProfile(
                OutputType outputType,
                double kP,
                double kI,
                double kD,
                double iZone,
                double tolerance,
                double maxVelocity,
                double maxAcceleration,
                PidAutotunerConfig autotuner) {
            this(outputType, kP, kI, kD, iZone, tolerance, maxVelocity, maxAcceleration, autotuner, null, null);
        }

        public PidProfile(
                OutputType outputType,
                double kP,
                double kI,
                double kD,
                double iZone,
                double tolerance,
                double maxVelocity,
                double maxAcceleration,
                PidAutotunerConfig autotuner,
                MechanismInputSource inputSource) {
            this(outputType, kP, kI, kD, iZone, tolerance, maxVelocity, maxAcceleration, autotuner, inputSource, null);
        }

    }

    public record BangBangProfile(
            OutputType outputType,
            double highOutput,
            double lowOutput,
            double tolerance,
            MechanismInputSource inputSource,
            MechanismSetpointSource setpointSource) {
        public BangBangProfile {
            inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
            setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        }

        public BangBangProfile(
                OutputType outputType,
                double highOutput,
                double lowOutput,
                double tolerance) {
            this(outputType, highOutput, lowOutput, tolerance, null, null);
        }

        public BangBangProfile(
                OutputType outputType,
                double highOutput,
                double lowOutput,
                double tolerance,
                MechanismInputSource inputSource) {
            this(outputType, highOutput, lowOutput, tolerance, inputSource, null);
        }

    }

    public enum FeedforwardType {
        SIMPLE("simple"),
        ARM("arm"),
        ELEVATOR("elevator");

        private final String configKey;

        FeedforwardType(String configKey) {
            this.configKey = configKey;
        }

        public String configKey() {
            return configKey;
        }

        public static FeedforwardType fromConfig(String rawType) {
            if (rawType == null || rawType.isBlank()) {
                return SIMPLE;
            }
            String normalized = rawType.trim().toLowerCase(java.util.Locale.ROOT);
            return switch (normalized) {
                case "simple" -> SIMPLE;
                case "arm" -> ARM;
                case "elevator" -> ELEVATOR;
                default -> SIMPLE;
            };
        }
    }

    public record FeedforwardProfile(
            OutputType outputType,
            FeedforwardType type,
            double kS,
            double kG,
            double kV,
            double kA,
            double tolerance,
            MechanismSetpointSource setpointSource) {
        public FeedforwardProfile {
            type = type != null ? type : FeedforwardType.SIMPLE;
            setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        }

        public FeedforwardProfile(
                OutputType outputType,
                FeedforwardType type,
                double kS,
                double kG,
                double kV,
                double kA,
                double tolerance) {
            this(outputType, type, kS, kG, kV, kA, tolerance, null);
        }

        public SimpleMotorFeedforward simple() {
            return new SimpleMotorFeedforward(kS, kV, kA);
        }

        public ArmFeedforward arm() {
            return new ArmFeedforward(kS, kG, kV, kA);
        }

        public ElevatorFeedforward elevator() {
            return new ElevatorFeedforward(kS, kG, kV, kA);
        }
    }

    public record LifecycleHookBinding<M extends Mechanism>(
            MechanismBinding<M, ?> binding,
            List<Enum<?>> states) {
        public LifecycleHookBinding {
            Objects.requireNonNull(binding, "binding");
            states = states == null ? List.of() : List.copyOf(states);
        }

        public boolean appliesTo(Enum<?> activeState) {
            if (states == null || states.isEmpty()) {
                return true;
            }
            if (activeState == null) {
                return false;
            }
            for (Enum<?> candidate : states) {
                if (candidate == activeState || candidate.equals(activeState)) {
                    return true;
                }
            }
            return false;
        }
    }

    public record PeriodicHookBinding<M extends Mechanism>(
            Consumer<M> hook,
            double periodMs) {
        public PeriodicHookBinding {
            Objects.requireNonNull(hook, "hook");
            if (!Double.isFinite(periodMs) || periodMs < 0.0) {
                throw new IllegalArgumentException("periodMs must be finite and >= 0");
            }
        }
    }

    /**
     * Associates visualization metadata (2D/3D nodes) with the mechanism. These nodes are rendered
     * automatically in Shuffleboard/AdvantageScope and reflect runtime simulation updates.
     *
     * @param visualizationConfig visualization metadata describing nodes and hierarchy
     * @return this config for chaining
     */
    public MechanismVisualizationConfig visualizationConfig() {
        return visualizationConfig;
    }

    public MechanismConfig<T> visualizationConfig(MechanismVisualizationConfig visualizationConfig) {
        this.visualizationConfig = Objects.requireNonNull(visualizationConfig);
        return this;
    }

    /**
     * Supplies a field-heading visualization for turret mechanisms (Field2d line for AdvantageScope).
     */
    public MechanismConfig<T> turretHeadingVisualization(Supplier<TurretMechanism.FieldHeadingVisualization> supplier) {
        this.turretHeadingVisualization = Objects.requireNonNull(supplier, "supplier");
        return this;
    }

    public static class ElevatorSimulationParameters {
        /** Optional carriage mass in kilograms. */
        public double carriageMassKg = Double.NaN;
        /** Optional drum radius in meters (distance from drum center to cable). */
        public double drumRadiusMeters = Double.NaN;
        /** Minimum vertical travel in meters. */
        public double minHeightMeters = 0.0;
        /** Maximum vertical travel in meters. */
        public double maxHeightMeters = 2.0;
        /** Starting height used when the sim initializes. */
        public double startingHeightMeters = 0.0;
        /** Whether to include gravity when simulating. */
        public boolean simulateGravity = true;
        /** Nominal battery voltage the simulator clamps against. */
        public double nominalVoltage = 12.0;
        /** Optional override that converts raw encoder units to meters. */
        public double unitsPerMeterOverride = Double.NaN;

        /**
         * Sets the simulated carriage mass in kilograms.
         *
         * @param carriageMassKg mass in kg
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters carriageMassKg(double carriageMassKg) {
            this.carriageMassKg = carriageMassKg;
            return this;
        }

        /**
         * Sets the radius of the winch drum in meters.
         *
         * @param drumRadiusMeters radius in meters
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters drumRadiusMeters(double drumRadiusMeters) {
            this.drumRadiusMeters = drumRadiusMeters;
            return this;
        }

        /**
         * Sets the minimum and maximum elevator travel in meters.
         *
         * @param minHeightMeters lower bound of travel
         * @param maxHeightMeters upper bound of travel
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters rangeMeters(double minHeightMeters, double maxHeightMeters) {
            this.minHeightMeters = minHeightMeters;
            this.maxHeightMeters = maxHeightMeters;
            return this;
        }

        /**
         * Sets the starting elevator height reported when the simulation resets.
         *
         * @param startingHeightMeters starting position in meters
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters startingHeightMeters(double startingHeightMeters) {
            this.startingHeightMeters = startingHeightMeters;
            return this;
        }

        /**
         * Enables or disables gravity effects in the elevator simulation.
         *
         * @param simulateGravity true to include gravity
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters simulateGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        /**
         * Overrides the nominal battery voltage used to clamp the simulated motor output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters nominalVoltage(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            return this;
        }

        /**
         * Supplies a manual conversion between encoder units and meters when the default inference
         * from the mechanism configuration is insufficient.
         *
         * @param unitsPerMeter encoder units per meter of travel
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters unitsPerMeter(double unitsPerMeter) {
            this.unitsPerMeterOverride = unitsPerMeter;
            return this;
        }
    }

    public static class ArmSimulationParameters {
        /** Optional motor-to-arm gear reduction (motor rotations per arm rotation). */
        public double motorReduction = Double.NaN;
        /** Optional moment of inertia around pivot, in kg·m^2. */
        public double momentOfInertia = Double.NaN;
        /** Optional link length in meters from pivot to end-effector. */
        public double armLengthMeters = Double.NaN;
        /** Lower bound on allowed angle (radians). */
        public double minAngleRadians = -Math.PI;
        /** Upper bound on allowed angle (radians). */
        public double maxAngleRadians = Math.PI;
        /** Starting angle when sim resets. */
        public double startingAngleRadians = 0.0;
        /** Whether to include gravity torque. */
        public boolean simulateGravity = true;
        /** Nominal battery voltage limit. */
        public double nominalVoltage = 12.0;
        /** Optional override to convert encoder units to radians. */
        public double unitsPerRadianOverride = Double.NaN;

        /**
         * Overrides the motor-to-arm gearing used by the sim model.
         *
         * @param motorReduction motor rotations per arm rotation
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters motorReduction(double motorReduction) {
            this.motorReduction = motorReduction;
            return this;
        }

        /**
         * Sets the simulated arm's moment of inertia about the pivot.
         *
         * @param momentOfInertia inertia in kg·m^2
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters momentOfInertia(double momentOfInertia) {
            this.momentOfInertia = momentOfInertia;
            return this;
        }

        /**
         * Sets the link length measured from the pivot to the end effector.
         *
         * @param armLengthMeters arm length in meters
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters armLengthMeters(double armLengthMeters) {
            this.armLengthMeters = armLengthMeters;
            return this;
        }

        /**
         * Sets the allowable arm travel range in radians.
         *
         * @param minAngleRadians lower bound of rotation
         * @param maxAngleRadians upper bound of rotation
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters angleRangeRadians(double minAngleRadians, double maxAngleRadians) {
            this.minAngleRadians = minAngleRadians;
            this.maxAngleRadians = maxAngleRadians;
            return this;
        }

        /**
         * Sets the starting arm angle reported when the simulation resets.
         *
         * @param startingAngleRadians starting angle in radians
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters startingAngleRadians(double startingAngleRadians) {
            this.startingAngleRadians = startingAngleRadians;
            return this;
        }

        /**
         * Enables or disables gravity torque in the arm simulation.
         *
         * @param simulateGravity true to include gravity
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters simulateGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        /**
         * Overrides the nominal battery voltage used to clamp simulated motor output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters nominalVoltage(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            return this;
        }

        /**
         * Supplies a manual conversion between encoder units and radians when the default inference
         * from the mechanism configuration is insufficient.
         *
         * @param unitsPerRadian encoder units per radian of arm rotation
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters unitsPerRadian(double unitsPerRadian) {
            this.unitsPerRadianOverride = unitsPerRadian;
            return this;
        }
    }

    public static class SimpleMotorSimulationParameters {
        /** Optional inertia of rotating system in kg·m^2. */
        public double momentOfInertia = Double.NaN;
        /** Nominal battery voltage limit. */
        public double nominalVoltage = 12.0;
        /** Optional override to convert encoder units to radians. */
        public double unitsPerRadianOverride = Double.NaN;

        /**
         * Sets the inertia of the rotating system being simulated.
         *
         * @param momentOfInertia inertia in kg·m^2
         * @return this parameter builder for chaining
         */
        public SimpleMotorSimulationParameters momentOfInertia(double momentOfInertia) {
            this.momentOfInertia = momentOfInertia;
            return this;
        }

        /**
         * Overrides the nominal battery voltage used to clamp simulated motor output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public SimpleMotorSimulationParameters nominalVoltage(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            return this;
        }

        /**
         * Supplies a manual conversion between encoder units and radians when the default inference
         * from the mechanism configuration is insufficient.
         *
         * @param unitsPerRadian encoder units per radian of rotation
         * @return this parameter builder for chaining
         */
        public SimpleMotorSimulationParameters unitsPerRadian(double unitsPerRadian) {
            this.unitsPerRadianOverride = unitsPerRadian;
            return this;
        }
    }

    /**
     * Finalizes the configuration, applies shared defaults to motors/encoders, and invokes the
     * registered factory to create the concrete mechanism instance. Simulation configurations are
     * also bound to the resulting mechanism here.
     *
     * @return constructed mechanism instance
     */
    public T build(){
        if (disabled) {
            T mechanism = buildDisabledMechanism();
            if (mechanismName != null && !mechanismName.isBlank()) {
                mechanism.setName(mechanismName);
            }
            mechanism.setConfigDisabled(true);
            if (stateGraph != null && mechanism instanceof StatefulLike<?> stateful) {
                applyStateGraph(stateful, stateGraph);
            }
            return mechanism;
        }
        synchronizeEncoderData();
        MechanismConfigRecord cfg = data;
        if (autoContinuousPidForUnboundedTurret) {
            cfg = applyAutoContinuousPidForUnboundedTurret(cfg);
            data = cfg;
        }
        for (MotorControllerConfig motor : cfg.motors()) {
            motor.hardware()
                    .neutralMode(cfg.motorNeutralMode())
                    .currentLimit(cfg.motorCurrentLimit())
                    .canbus(cfg.canbus());
        }

        // PID controller construction/config is handled by the mechanism runtime using named profiles.

        if (simulationConfig == null) {
            if (elevatorSimulationParameters != null) {
                MechanismSimulationConfig.ElevatorParameters params = MechanismSimulationConfig.ElevatorParameters.create()
                        .simulateGravity(elevatorSimulationParameters.simulateGravity)
                        .startingHeight(elevatorSimulationParameters.startingHeightMeters)
                        .heightLimits(elevatorSimulationParameters.minHeightMeters, elevatorSimulationParameters.maxHeightMeters)
                        .nominalVoltage(elevatorSimulationParameters.nominalVoltage);

                if (!Double.isNaN(elevatorSimulationParameters.carriageMassKg)) {
                    params.carriageMassKg(elevatorSimulationParameters.carriageMassKg);
                }
                if (!Double.isNaN(elevatorSimulationParameters.drumRadiusMeters)) {
                    params.drumRadiusMeters(elevatorSimulationParameters.drumRadiusMeters);
                }
                if (!Double.isNaN(elevatorSimulationParameters.unitsPerMeterOverride)) {
                    params.unitsPerMeter(elevatorSimulationParameters.unitsPerMeterOverride);
                }

                simulationConfig = MechanismSimulationConfig.elevator(params);
            } else if (armSimulationParameters != null) {
                MechanismSimulationConfig.ArmParameters params = MechanismSimulationConfig.ArmParameters.create()
                        .simulateGravity(armSimulationParameters.simulateGravity)
                        .startingAngle(armSimulationParameters.startingAngleRadians)
                        .angleLimits(armSimulationParameters.minAngleRadians, armSimulationParameters.maxAngleRadians)
                        .nominalVoltage(armSimulationParameters.nominalVoltage);

                if (!Double.isNaN(armSimulationParameters.momentOfInertia)) {
                    params.momentOfInertia(armSimulationParameters.momentOfInertia);
                }
                if (!Double.isNaN(armSimulationParameters.motorReduction)) {
                    params.gearing(armSimulationParameters.motorReduction);
                }
                if (!Double.isNaN(armSimulationParameters.armLengthMeters)) {
                    params.armLengthMeters(armSimulationParameters.armLengthMeters);
                }
                if (!Double.isNaN(armSimulationParameters.unitsPerRadianOverride)) {
                    params.unitsPerRadian(armSimulationParameters.unitsPerRadianOverride);
                }

                simulationConfig = MechanismSimulationConfig.arm(params);
            } else if (simpleMotorSimulationParameters != null) {
                MechanismSimulationConfig.SimpleMotorParameters params = MechanismSimulationConfig.SimpleMotorParameters.create()
                        .nominalVoltage(simpleMotorSimulationParameters.nominalVoltage);

                if (!Double.isNaN(simpleMotorSimulationParameters.momentOfInertia)) {
                    params.momentOfInertia(simpleMotorSimulationParameters.momentOfInertia);
                }
                if (!Double.isNaN(simpleMotorSimulationParameters.unitsPerRadianOverride)) {
                    params.unitsPerRadian(simpleMotorSimulationParameters.unitsPerRadianOverride);
                }

                simulationConfig = MechanismSimulationConfig.simpleMotor(params);
            }
        }

        if (simulationConfig != null) {
            simulationConfig = simulationConfig.bindSourceConfig(this);
        }

        validateMotorTypesForHardware();
        validateMotorTypesForSimulation();

        T mechanism = factory.apply(this);
        if (mechanismName != null && !mechanismName.isBlank()) {
            mechanism.setName(mechanismName);
        }

        if (stateGraph != null && mechanism instanceof StatefulLike<?> stateful) {
            applyStateGraph(stateful, stateGraph);
        }

        return mechanism;
    }

    private T buildDisabledMechanism() {
        MechanismConfigRecord originalData = data;
        MechanismSimulationConfig originalSimulationConfig = simulationConfig;
        ElevatorSimulationParameters originalElevatorSimulationParameters = elevatorSimulationParameters;
        ArmSimulationParameters originalArmSimulationParameters = armSimulationParameters;
        SimpleMotorSimulationParameters originalSimpleMotorSimulationParameters = simpleMotorSimulationParameters;
        MechanismSensorSimulationConfig originalSensorSimulationConfig = sensorSimulationConfig;
        LinkedHashMap<String, EncoderSourceSpec> originalEncoderSources = new LinkedHashMap<>(encoderSourceSpecs);
        String originalPositionSource = controlPositionSourceName;
        String originalVelocitySource = controlVelocitySourceName;
        String originalAbsoluteSource = controlAbsoluteSourceName;
        try {
            data = data.toBuilder()
                    .motors(new ArrayList<>())
                    .encoders(new ArrayList<>())
                    .positionSource(null)
                    .velocitySource(null)
                    .absoluteSource(null)
                    .limitSwitches(new ArrayList<>())
                    .build();
            encoderSourceSpecs.clear();
            controlPositionSourceName = null;
            controlVelocitySourceName = null;
            controlAbsoluteSourceName = null;
            simulationConfig = null;
            elevatorSimulationParameters = null;
            armSimulationParameters = null;
            simpleMotorSimulationParameters = null;
            sensorSimulationConfig = null;
            return factory.apply(this);
        } finally {
            data = originalData;
            encoderSourceSpecs.clear();
            encoderSourceSpecs.putAll(originalEncoderSources);
            controlPositionSourceName = originalPositionSource;
            controlVelocitySourceName = originalVelocitySource;
            controlAbsoluteSourceName = originalAbsoluteSource;
            simulationConfig = originalSimulationConfig;
            elevatorSimulationParameters = originalElevatorSimulationParameters;
            armSimulationParameters = originalArmSimulationParameters;
            simpleMotorSimulationParameters = originalSimpleMotorSimulationParameters;
            sensorSimulationConfig = originalSensorSimulationConfig;
        }
    }

    /**
     * Ensures each configured motor type is registered for hardware use. This surfaces missing
     * vendor modules early instead of failing silently at runtime.
     */
    private void validateMotorTypesForHardware() {
        if (data.motors() == null || data.motors().isEmpty()) {
            return;
        }
        data.motors().forEach(config -> {
            if (config.type() == null) {
                throw new IllegalStateException("Motor controller config is missing a type");
            }
            // Throws with a clear message if the vendor module is absent.
            MotorRegistry.get().motor(config.type().getKey());
        });
    }

    @SuppressWarnings("unchecked")
    private static <E extends Enum<E>> void applyStateGraph(
            StatefulLike<?> stateful,
            StateGraph<?> stateGraph) {
        StatefulLike<E> typedMechanism = (StatefulLike<E>) stateful;
        StateGraph<E> typedGraph = (StateGraph<E>) stateGraph;
        typedMechanism.stateMachine().graph(typedGraph);
    }

    /**
     * Ensures each configured motor type can be mapped to a simulation model whenever simulation
     * metadata is present. This prevents silent fallbacks to placeholder motors.
     */
    private void validateMotorTypesForSimulation() {
        boolean simEnabled = simulationConfig != null
                || elevatorSimulationParameters != null
                || armSimulationParameters != null
                || simpleMotorSimulationParameters != null;
        if (!simEnabled) {
            return;
        }
        if (data.motors() == null || data.motors().isEmpty()) {
            throw new IllegalStateException("Simulation requires at least one motor controller to be configured");
        }
        data.motors().forEach(config -> {
            if (config.type() == null) {
                throw new IllegalStateException("Motor controller config is missing a type");
            }
            MechanismSimulationConfig.requireSupportedMotorSim(config.type());
        });
    }

}
