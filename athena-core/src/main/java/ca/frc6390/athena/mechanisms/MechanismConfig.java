package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
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

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderRegistry;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.hardware.motor.MotorRegistry;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.FlywheelMechanism;
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
    // Selected "main" control profiles used by the built-in mechanism PID/FF implementations.
    // These are optional; teams can also drive output entirely via custom control loops.
    private String mainPidProfileName;
    private String mainSimpleFeedforwardProfileName;
    private String mainArmFeedforwardProfileName;
    private String mainElevatorFeedforwardProfileName;
    /**
     * When true, turret factory helpers will auto-enable continuous PID input for unbounded turrets
     * using the configured encoder conversion as the wrap span (e.g. 360 degrees, 2*pi radians).
     *
     * This is intentionally only applied when bounds are not configured because bounded turrets
     * typically must not "wrap" across hard stops.
     */
    private boolean autoContinuousPidForUnboundedTurret;
    /** Factory used to instantiate the final mechanism once configuration is complete. */
    public Function<MechanismConfig<T>, T> factory = null;
    /** Optional per-state callbacks that run when the mechanism state machine enters the state. */
    public Map<Enum<?>, Function<T, Boolean>> stateActions = new HashMap<>();
    /** Optional hooks that run once when entering a state. */
    public Map<Enum<?>, List<MechanismBinding<T, ?>>> enterStateHooks = new HashMap<>();
    /** Optional state hooks that run every loop while a state is active. */
    public Map<Enum<?>, List<MechanismBinding<T, ?>>> stateHooks = new HashMap<>();
    /** Optional hooks that run once when leaving a state. */
    public Map<Enum<?>, List<MechanismBinding<T, ?>>> exitStateHooks = new HashMap<>();
    /** Optional hooks that run once for specific state transitions (from -> to). */
    public List<TransitionHookBinding<T>> transitionHooks = new ArrayList<>();
    /** Optional hooks that run once whenever any state is exited. */
    public List<MechanismBinding<T, ?>> exitAlwaysHooks = new ArrayList<>();
    /** Optional hooks that run every loop regardless of the active state. */
    public List<MechanismBinding<T, ?>> alwaysHooks = new ArrayList<>();
    /** Optional state triggers that can enqueue states when a predicate becomes true. */
    public List<StateTriggerBinding<T>> stateTriggerBindings = new ArrayList<>();
    /** Optional hooks that run every periodic loop. */
    public List<Consumer<T>> periodicHooks = new ArrayList<>();
    /** Optional hooks that run on a fixed cadence rather than every loop. */
    public List<PeriodicHookBinding<T>> periodicHookBindings = new ArrayList<>();
    /** Optional custom control loops that return output contributions. */
    public List<ControlLoopBinding<T>> controlLoops = new ArrayList<>();
    /** Optional named PID profiles for control-loop usage. */
    public Map<String, PidProfile> controlLoopPidProfiles = new HashMap<>();
    /** Optional named simple-motor feedforward profiles for control-loop usage (and simple-motor main FF). */
    public Map<String, FeedforwardProfile> controlLoopFeedforwardProfiles = new HashMap<>();
    /** Optional named arm feedforward profiles (for ArmMechanism main FF). */
    public Map<String, ArmFeedforwardProfile> armFeedforwardProfiles = new HashMap<>();
    /** Optional named elevator feedforward profiles (for ElevatorMechanism main FF). */
    public Map<String, ElevatorFeedforwardProfile> elevatorFeedforwardProfiles = new HashMap<>();
    /** Optional boolean inputs exposed to state hooks. */
    public Map<String, BooleanSupplier> inputs = new HashMap<>();
    /** Optional mutable boolean inputs (defaults) that other systems can set at runtime. */
    public Map<String, Boolean> mutableBoolInputDefaults = new HashMap<>();
    /** Optional double inputs exposed to state hooks. */
    public Map<String, DoubleSupplier> doubleInputs = new HashMap<>();
    /** Optional mutable double inputs (defaults) that other systems can set at runtime. */
    public Map<String, Double> mutableDoubleInputDefaults = new HashMap<>();
    /** Optional int inputs exposed to hooks/loops. */
    public Map<String, IntSupplier> intInputs = new HashMap<>();
    /** Optional mutable int inputs (defaults) that other systems can set at runtime. */
    public Map<String, Integer> mutableIntInputDefaults = new HashMap<>();
    /** Optional string inputs exposed to hooks/loops. */
    public Map<String, Supplier<String>> stringInputs = new HashMap<>();
    /** Optional mutable string inputs (defaults) that other systems can set at runtime. */
    public Map<String, String> mutableStringInputDefaults = new HashMap<>();
    /** Optional Pose2d inputs exposed to hooks/loops. */
    public Map<String, Supplier<Pose2d>> pose2dInputs = new HashMap<>();
    /** Optional mutable Pose2d inputs (defaults) that other systems can set at runtime. */
    public Map<String, Pose2d> mutablePose2dInputDefaults = new HashMap<>();
    /** Optional Pose3d inputs exposed to hooks/loops. */
    public Map<String, Supplier<Pose3d>> pose3dInputs = new HashMap<>();
    /** Optional mutable Pose3d inputs (defaults) that other systems can set at runtime. */
    public Map<String, Pose3d> mutablePose3dInputDefaults = new HashMap<>();
    /** Optional object inputs exposed to state hooks. */
    public Map<String, Supplier<?>> objectInputs = new HashMap<>();
    /** Optional transition graph that defines required intermediate states and guards. */
    public StateGraph<?> stateGraph = null;
    /** Simulation model description used when running in simulation environments. */
    public MechanismSimulationConfig simulationConfig = null;
    /** Cached elevator-specific simulation hints provided through {@link #setSimulationElevator}. */
    public ElevatorSimulationParameters elevatorSimulationParameters = null;
    /** Cached arm-specific simulation hints provided through {@link #setSimulationArm}. */
    public ArmSimulationParameters armSimulationParameters = null;
    /** Cached simple-motor simulation hints provided through {@link #setSimulationSimpleMotor}. */
    public SimpleMotorSimulationParameters simpleMotorSimulationParameters = null;
    /** Optional visualization metadata consumed by the mechanism visualizer. */
    public ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig visualizationConfig = null;
    /** Optional field-heading visualization config for turret mechanisms. */
    public Supplier<TurretMechanism.FieldHeadingVisualization> turretHeadingVisualization = null;
    /** Optional sensor simulation configuration used to generate virtual readings. */
    public MechanismSensorSimulationConfig sensorSimulationConfig = null;
    public boolean shouldCustomEncoder = false;
    public DoubleSupplier customEncoderPos;
    FeedforwardProfile resolveMainSimpleFeedforwardProfile() {
        if (mainSimpleFeedforwardProfileName == null || mainSimpleFeedforwardProfileName.isBlank()) {
            return null;
        }
        FeedforwardProfile profile = controlLoopFeedforwardProfiles.get(mainSimpleFeedforwardProfileName);
        if (profile == null) {
            throw new IllegalStateException("Main feedforward profile selected but not registered: " + mainSimpleFeedforwardProfileName);
        }
        return profile;
    }

    ArmFeedforwardProfile resolveMainArmFeedforwardProfile() {
        if (mainArmFeedforwardProfileName == null || mainArmFeedforwardProfileName.isBlank()) {
            return null;
        }
        ArmFeedforwardProfile profile = armFeedforwardProfiles.get(mainArmFeedforwardProfileName);
        if (profile == null) {
            throw new IllegalStateException("Main arm feedforward profile selected but not registered: " + mainArmFeedforwardProfileName);
        }
        return profile;
    }

    ElevatorFeedforwardProfile resolveMainElevatorFeedforwardProfile() {
        if (mainElevatorFeedforwardProfileName == null || mainElevatorFeedforwardProfileName.isBlank()) {
            return null;
        }
        ElevatorFeedforwardProfile profile = elevatorFeedforwardProfiles.get(mainElevatorFeedforwardProfileName);
        if (profile == null) {
            throw new IllegalStateException("Main elevator feedforward profile selected but not registered: " + mainElevatorFeedforwardProfileName);
        }
        return profile;
    }

    PidProfile resolveMainPidProfile() {
        if (mainPidProfileName == null || mainPidProfileName.isBlank()) {
            return null;
        }
        return controlLoopPidProfiles.get(mainPidProfileName);
    }

    String mainPidProfileName() {
        return mainPidProfileName;
    }

    String mainSimpleFeedforwardProfileName() {
        return mainSimpleFeedforwardProfileName;
    }

    String mainArmFeedforwardProfileName() {
        return mainArmFeedforwardProfileName;
    }

    String mainElevatorFeedforwardProfileName() {
        return mainElevatorFeedforwardProfileName;
    }

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
     * Sectioned fluent API: encoder.
     */
    public MechanismConfig<T> encoder(Consumer<EncoderSection<T>> section) {
        if (section != null) {
            section.accept(new EncoderSection<>(this));
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
            owner.data.motors().add(new MotorControllerConfig(motor.resolveController(), id));
            return this;
        }

        /**
         * Adds a motor controller using a {@link MotorControllerType} registry key (data-only / deploy-friendly).
         */
        public MotorsSection<T> add(MotorControllerType type, int id) {
            owner.data.motors().add(new MotorControllerConfig(type, id));
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

    public static final class EncoderSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private EncoderSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        public EncoderSection<T> config(ca.frc6390.athena.hardware.encoder.EncoderConfig config) {
            owner.updateData(builder -> builder.encoder(config));
            return this;
        }

        public EncoderSection<T> fromMotor(int motorId) {
            int abs = Math.abs(motorId);
            MotorControllerConfig motor = owner.data.motors().stream()
                    .filter(cfg -> cfg != null && cfg.id == abs)
                    .findFirst()
                    .orElseThrow(() -> new IllegalStateException("No motor controller configured with ID " + abs));

            ca.frc6390.athena.hardware.encoder.EncoderConfig encoderCfg = motor.encoderConfig;
            if (encoderCfg == null) {
                encoderCfg = new ca.frc6390.athena.hardware.encoder.EncoderConfig()
                        .setType(resolveIntegratedEncoderType(motor.type))
                        .setId(motor.id)
                        .setCanbus(motor.canbus);
                motor.encoderConfig = encoderCfg;
            } else if (encoderCfg.type == null) {
                ca.frc6390.athena.hardware.encoder.EncoderConfig resolved = new ca.frc6390.athena.hardware.encoder.EncoderConfig()
                        .setType(resolveIntegratedEncoderType(motor.type))
                        .setId(encoderCfg.id != 0 ? encoderCfg.id : motor.id)
                        .setCanbus(encoderCfg.canbus != null ? encoderCfg.canbus : motor.canbus)
                        .setGearRatio(encoderCfg.gearRatio)
                        .setConversion(encoderCfg.conversion)
                        .setConversionOffset(encoderCfg.conversionOffset)
                        .setOffset(encoderCfg.offset)
                        .setDiscontinuity(encoderCfg.discontinuityPoint, encoderCfg.discontinuityRange)
                        .setInverted(encoderCfg.inverted);
                encoderCfg = resolved;
                motor.encoderConfig = encoderCfg;
            }
            encoderCfg.setInverted(motorId < 0);
            final ca.frc6390.athena.hardware.encoder.EncoderConfig finalEncoderCfg = encoderCfg;
            owner.updateData(builder -> builder.encoder(finalEncoderCfg));
            return this;
        }

        public EncoderSection<T> encoder(AthenaEncoder type, int id) {
            config(ca.frc6390.athena.hardware.encoder.EncoderConfig.type(type.resolve(), id));
            return this;
        }

        public EncoderSection<T> gearRatio(double gearRatio) {
            owner.updateData(builder -> builder.encoderGearRatio(gearRatio));
            return this;
        }

        public EncoderSection<T> conversion(double conversion) {
            owner.updateData(builder -> builder.encoderConversion(conversion));
            return this;
        }

        public EncoderSection<T> conversionOffset(double conversionOffset) {
            owner.updateData(builder -> builder.encoderConversionOffset(conversionOffset));
            return this;
        }

        public EncoderSection<T> mutate(Consumer<ca.frc6390.athena.hardware.encoder.EncoderConfig> mutator) {
            if (mutator == null) {
                return this;
            }
            ca.frc6390.athena.hardware.encoder.EncoderConfig cfg = owner.data.encoder();
            if (cfg == null) {
                cfg = new ca.frc6390.athena.hardware.encoder.EncoderConfig();
                ca.frc6390.athena.hardware.encoder.EncoderConfig finalCfg = cfg;
                owner.updateData(builder -> builder.encoder(finalCfg));
            }
            mutator.accept(cfg);
            return this;
        }

        public EncoderSection<T> offset(double offset) {
            owner.updateData(builder -> builder.encoderOffset(offset));
            return this;
        }

        public EncoderSection<T> discontinuityPoint(double discontinuityPoint) {
            owner.updateData(builder -> builder.encoderDiscontinuityPoint(discontinuityPoint));
            return this;
        }

        public EncoderSection<T> discontinuityRange(double discontinuityRange) {
            owner.updateData(builder -> builder.encoderDiscontinuityRange(discontinuityRange));
            return this;
        }

        public EncoderSection<T> absolute(boolean absolute) {
            owner.updateData(builder -> builder.useAbsolute(absolute));
            return this;
        }

        public EncoderSection<T> custom(java.util.function.DoubleSupplier positionSupplier) {
            owner.shouldCustomEncoder = true;
            owner.customEncoderPos = positionSupplier;
            return this;
        }
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
            owner.inputs.put(k, supplier);
            return this;
        }

        /**
         * Declares a mutable boolean input with a default value.
         * Other mechanisms/superstructures can set it via {@link Mechanism#setBoolVal(String, boolean)}.
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
            owner.doubleInputs.put(k, supplier);
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
            owner.intInputs.put(k, supplier);
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
            owner.stringInputs.put(k, supplier);
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
            owner.pose2dInputs.put(k, supplier);
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
            owner.pose3dInputs.put(k, supplier);
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
            owner.objectInputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
            return this;
        }
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

        public ControlSection<T> pidUseVelocity(boolean useVelocity) {
            owner.updateData(builder -> builder.pidUseVelocity(useVelocity));
            return this;
        }

        public ControlSection<T> setpointAsOutput(boolean enabled) {
            owner.updateData(builder -> builder.useSetpointAsOutput(enabled));
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
         * Registers a named PID profile for use by custom control loops and/or the mechanism's main PID.
         *
         * <p>Output type defaults to {@link OutputType#PERCENT}. Most teams will want {@link OutputType#VOLTAGE}
         * if they tune gains in volts.</p>
         */
        public ControlSection<T> pidProfile(String name, double kP, double kI, double kD) {
            return pidProfile(name, OutputType.PERCENT, kP, kI, kD, Double.NaN, Double.NaN);
        }

        public ControlSection<T> pidProfile(String name, OutputType outputType, double kP, double kI, double kD) {
            return pidProfile(name, outputType, kP, kI, kD, Double.NaN, Double.NaN);
        }

        public ControlSection<T> pidProfile(String name, double kP, double kI, double kD, double iZone) {
            return pidProfile(name, OutputType.PERCENT, kP, kI, kD, iZone, Double.NaN);
        }

        public ControlSection<T> pidProfile(String name, OutputType outputType, double kP, double kI, double kD, double iZone) {
            return pidProfile(name, outputType, kP, kI, kD, iZone, Double.NaN);
        }

        public ControlSection<T> pidProfile(String name, double kP, double kI, double kD, double iZone, double tolerance) {
            return pidProfile(name, OutputType.PERCENT, kP, kI, kD, iZone, tolerance);
        }

        public ControlSection<T> pidProfile(String name, OutputType outputType, double kP, double kI, double kD, double iZone, double tolerance) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("PID profile name cannot be blank");
            }
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.PERCENT;
            if (resolvedOutput != OutputType.PERCENT && resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("PID profile output type must be PERCENT or VOLTAGE");
            }
            owner.controlLoopPidProfiles.put(name, new PidProfile(resolvedOutput, kP, kI, kD, iZone, tolerance));
            return this;
        }

        /**
         * Selects which named PID profile should be used by the mechanism's built-in PID (if any).
         */
        public ControlSection<T> mainPid(String name) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("main PID profile name cannot be blank");
            }
            owner.mainPidProfileName = name;
            return this;
        }

        public ControlSection<T> feedforwardProfile(String name, double kS, double kV, double kA) {
            return feedforwardProfile(name, OutputType.VOLTAGE, kS, kV, kA);
        }

        public ControlSection<T> feedforwardProfile(String name, OutputType outputType, double kS, double kV, double kA) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("feedforward profile name cannot be blank");
            }
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.VOLTAGE;
            if (resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("feedforward profile output type must be VOLTAGE");
            }
            owner.controlLoopFeedforwardProfiles.put(name, new FeedforwardProfile(resolvedOutput, new SimpleMotorFeedforward(kS, kV, kA)));
            return this;
        }

        /**
         * Registers a named arm feedforward profile (usable by ArmMechanism main FF).
         */
        public ControlSection<T> armFeedforwardProfile(String name, double kS, double kG, double kV, double kA) {
            return armFeedforwardProfile(name, OutputType.VOLTAGE, new ArmFeedforward(kS, kG, kV, kA));
        }

        public ControlSection<T> armFeedforwardProfile(String name, OutputType outputType, ArmFeedforward feedforward) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("arm feedforward profile name cannot be blank");
            }
            Objects.requireNonNull(feedforward, "feedforward");
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.VOLTAGE;
            if (resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("arm feedforward profile output type must be VOLTAGE");
            }
            owner.armFeedforwardProfiles.put(name, new ArmFeedforwardProfile(resolvedOutput, feedforward));
            return this;
        }

        /**
         * Registers a named elevator feedforward profile (usable by ElevatorMechanism main FF).
         */
        public ControlSection<T> elevatorFeedforwardProfile(String name, double kS, double kG, double kV, double kA) {
            return elevatorFeedforwardProfile(name, OutputType.VOLTAGE, new ElevatorFeedforward(kS, kG, kV, kA));
        }

        public ControlSection<T> elevatorFeedforwardProfile(String name, OutputType outputType, ElevatorFeedforward feedforward) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("elevator feedforward profile name cannot be blank");
            }
            Objects.requireNonNull(feedforward, "feedforward");
            OutputType resolvedOutput = outputType != null ? outputType : OutputType.VOLTAGE;
            if (resolvedOutput != OutputType.VOLTAGE) {
                throw new IllegalArgumentException("elevator feedforward profile output type must be VOLTAGE");
            }
            owner.elevatorFeedforwardProfiles.put(name, new ElevatorFeedforwardProfile(resolvedOutput, feedforward));
            return this;
        }

        /**
         * Selects which named simple-motor feedforward profile should be used by the mechanism's built-in FF
         * (SimpleMotorMechanism/Flywheel/Turret).
         */
        public ControlSection<T> mainSimpleFeedforward(String name) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("main feedforward profile name cannot be blank");
            }
            owner.mainSimpleFeedforwardProfileName = name;
            return this;
        }

        public ControlSection<T> mainArmFeedforward(String name) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("main arm feedforward profile name cannot be blank");
            }
            owner.mainArmFeedforwardProfileName = name;
            return this;
        }

        public ControlSection<T> mainElevatorFeedforward(String name) {
            Objects.requireNonNull(name, "name");
            if (name.isBlank()) {
                throw new IllegalArgumentException("main elevator feedforward profile name cannot be blank");
            }
            owner.mainElevatorFeedforwardProfileName = name;
            return this;
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

    public static final class HooksSection<T extends Mechanism> {
        private final MechanismConfig<T> owner;

        private HooksSection(MechanismConfig<T> owner) {
            this.owner = owner;
        }

        @SafeVarargs
        public final <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> onStateEnter(
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
        public final <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> onStatePeriodic(
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
        public final <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> onStateExit(
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

        public <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> always(MechanismBinding<T, E> binding) {
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
        public <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> stateTrigger(
                E state,
                StateTrigger<T, E> trigger) {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(trigger, "trigger");
            owner.stateTriggerBindings.add(new StateTriggerBinding<>(state, (StateTrigger<T, ?>) trigger));
            return this;
        }

        public <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> onAnyStateExit(MechanismBinding<T, E> binding) {
            Objects.requireNonNull(binding, "binding");
            owner.exitAlwaysHooks.add(binding);
            return this;
        }

        public HooksSection<T> onRobotPeriodic(Consumer<T> hook) {
            Objects.requireNonNull(hook, "hook");
            owner.periodicHooks.add(hook);
            return this;
        }

        public HooksSection<T> onRobotPeriodic(Consumer<T> hook, double periodMs) {
            Objects.requireNonNull(hook, "hook");
            owner.periodicHookBindings.add(new PeriodicHookBinding<>(hook, periodMs));
            return this;
        }

        public <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> onStateTransition(
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
        public final <E extends Enum<E> & SetpointProvider<Double>> HooksSection<T> onStateTransition(
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
    }

    @FunctionalInterface
    public interface StateTrigger<T extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {
        boolean shouldQueue(MechanismContext<T, E> ctx);
    }

    public record StateTriggerBinding<T extends Mechanism>(Enum<?> state, StateTrigger<T, ?> trigger) {
        public StateTriggerBinding {
            Objects.requireNonNull(state, "state");
            Objects.requireNonNull(trigger, "trigger");
        }
    }


    public MechanismConfigRecord data() {
        return data;
    }

    public MechanismConfig<T> setData(MechanismConfigRecord data) {
        this.data = data != null ? data : MechanismConfigRecord.defaults();
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

    private void updateData(Consumer<MechanismConfigRecord.Builder> mutator) {
        MechanismConfigRecord.Builder builder = data.toBuilder();
        mutator.accept(builder);
        data = builder.build();
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
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulMechanism<E>> statefulGeneric(E initialState){
        return custom(config -> new StatefulMechanism<>(config, initialState));
    }

    /**
     * Named variant of {@link #statefulGeneric(Enum)}.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulMechanism<E>> statefulGeneric(String name, E initialState) {
        return statefulGeneric(initialState).named(name);
    }

    /**
     * Creates a state-aware mechanism configuration using a caller-supplied factory.
     *
     * @param factory builder that receives this config and the initial state
     * @param initialState starting state when the mechanism is constructed
     * @param <E> state enum type that provides setpoints
     * @param <T> concrete mechanism type returned from the factory
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulMechanism<E>> MechanismConfig<T> stateful(BiFunction<MechanismConfig<T>, E, T> factory, E initialState) {
        return custom(config -> factory.apply(config, initialState));
    }

    /**
     * Creates a configuration that will build an {@link ElevatorMechanism} with optional feedforward control.
     *
     * <p>Feedforward is selected via {@link #control(Consumer)} using
     * {@code .elevatorFeedforwardProfile(...).mainElevatorFeedforward(...)}.</p>
     */
    public static MechanismConfig<ElevatorMechanism> elevator() {
        return custom(config -> {
            ElevatorFeedforwardProfile ff = config.resolveMainElevatorFeedforwardProfile();
            return new ElevatorMechanism(config,
                    ff != null ? ff.feedforward() : null,
                    ff != null ? ff.outputType() : null);
        });
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
     * Builds a stateful elevator configuration with an initial state and feedforward gains.
     *
     * @param feedforward feedforward model tuned for the elevator
     * @param initialState state machine starting point
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulElevatorMechanism<E>> statefulElevator(E initialState) {
        return custom(config -> {
            ElevatorFeedforwardProfile ff = config.resolveMainElevatorFeedforwardProfile();
            return new StatefulElevatorMechanism<>(config,
                    ff != null ? ff.feedforward() : null,
                    ff != null ? ff.outputType() : null,
                    initialState);
        });
    }

    /**
     * Named variant of {@link #statefulElevator(Enum)}.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulElevatorMechanism<E>> statefulElevator(
            String name,
            E initialState) {
        return statefulElevator(initialState).named(name);
    }

    /**
     * Builds a stateful elevator configuration backed by a caller-supplied factory.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulElevatorMechanism<E>> MechanismConfig<T> statefulElevator(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful arm configuration that uses the provided feedforward model.
     *
     * @param feedforward feedforward model tuned for the arm
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulArmMechanism<E>> statefulArm(E initialState) {
        return custom(config -> {
            ArmFeedforwardProfile ff = config.resolveMainArmFeedforwardProfile();
            return new StatefulArmMechanism<>(config,
                    ff != null ? ff.feedforward() : null,
                    ff != null ? ff.outputType() : null,
                    initialState);
        });
    }

    /**
     * Named variant of {@link #statefulArm(Enum)}.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulArmMechanism<E>> statefulArm(
            String name,
            E initialState) {
        return statefulArm(initialState).named(name);
    }

    /**
     * Builds a stateful arm configuration backed by a caller-supplied factory.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulArmMechanism<E>> MechanismConfig<T> statefulArm(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful turret configuration (simple motor with continuous rotation) with feedforward support.
     *
     * @param feedforward feedforward model tuned for the mechanism
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulTurretMechanism<E>> statefulTurret(E initialState) {
        MechanismConfig<StatefulTurretMechanism<E>> cfg =
                MechanismConfig.<StatefulTurretMechanism<E>>custom(
                        config -> {
                            FeedforwardProfile ff = config.resolveMainSimpleFeedforwardProfile();
                            return new StatefulTurretMechanism<>(config,
                                    ff != null ? ff.feedforward() : null,
                                    ff != null ? ff.outputType() : null,
                                    initialState);
                        });
        cfg.autoContinuousPidForUnboundedTurret = true;
        return cfg;
    }

    /**
     * Named variant of {@link #statefulTurret(Enum)}.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulTurretMechanism<E>> statefulTurret(
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
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulTurretMechanism<E>> MechanismConfig<T> statefulTurret(Function<MechanismConfig<T>, T> factory) {
        MechanismConfig<T> cfg = MechanismConfig.<T>custom(factory);
        cfg.autoContinuousPidForUnboundedTurret = true;
        return cfg;
    }

    /**
     * Builds a stateful flywheel configuration with feedforward support.
     *
     * @param feedforward feedforward model tuned for the flywheel
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulFlywheelMechanism<E>> statefulFlywheel(E initialState) {
        return custom(config -> {
            FeedforwardProfile ff = config.resolveMainSimpleFeedforwardProfile();
            return new StatefulFlywheelMechanism<>(config,
                    ff != null ? ff.feedforward() : null,
                    ff != null ? ff.outputType() : null,
                    initialState);
        });
    }

    /**
     * Named variant of {@link #statefulFlywheel(Enum)}.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulFlywheelMechanism<E>> statefulFlywheel(
            String name,
            E initialState) {
        return statefulFlywheel(initialState).named(name);
    }

    /**
     * Builds a stateful flywheel configuration backed by a caller-supplied factory.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulFlywheelMechanism<E>> MechanismConfig<T> statefulFlywheel(Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Creates a turret configuration that yields a {@link TurretMechanism}.
     *
     * @param feedforward feedforward model tuned for the mechanism
     */
    public static MechanismConfig<TurretMechanism> turret() {
        MechanismConfig<TurretMechanism> cfg =
                MechanismConfig.<TurretMechanism>custom(config -> {
                    FeedforwardProfile ff = config.resolveMainSimpleFeedforwardProfile();
                    return new TurretMechanism(config,
                            ff != null ? ff.feedforward() : null,
                            ff != null ? ff.outputType() : null);
                });
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
     * @param feedforward feedforward model to pass through to the factory
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
     *
     * @param feedforward feedforward model tuned for the flywheel
     */
    public static MechanismConfig<FlywheelMechanism> flywheel() {
        return custom(config -> {
            FeedforwardProfile ff = config.resolveMainSimpleFeedforwardProfile();
            return new FlywheelMechanism(config,
                    ff != null ? ff.feedforward() : null,
                    ff != null ? ff.outputType() : null);
        });
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
     * <p>Feedforward is selected via {@link #control(Consumer)} using
     * {@code .armFeedforwardProfile(...).mainArmFeedforward(...)}.</p>
     */
    public static MechanismConfig<ArmMechanism> arm() {
        return custom(config -> {
            ArmFeedforwardProfile ff = config.resolveMainArmFeedforwardProfile();
            return new ArmMechanism(config,
                    ff != null ? ff.feedforward() : null,
                    ff != null ? ff.outputType() : null);
        });
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

    private static EncoderType resolveIntegratedEncoderType(MotorControllerType type) {
        if (type == null) {
            throw new IllegalStateException("Motor controller config is missing a type");
        }
        String key = type.getKey();
        if (key == null || key.isBlank()) {
            throw new IllegalStateException("Motor controller type key is missing");
        }
        String encoderKey = integratedEncoderKey(key);
        if (encoderKey == null) {
            throw new IllegalStateException(
                    "Motor type '" + key + "' does not expose an integrated encoder. "
                            + "Configure an encoder explicitly with encoder(e -> e.encoder(...)).");
        }
        return EncoderRegistry.get().encoder(encoderKey);
    }

    private static String integratedEncoderKey(String motorKey) {
        if (motorKey.startsWith("rev:sparkmax")) {
            return "rev:sparkmax";
        }
        if (motorKey.startsWith("rev:sparkflex")) {
            return "rev:sparkflex";
        }
        if (motorKey.startsWith("ctre:talonfx")) {
            return "ctre:talonfx-integrated";
        }
        return null;
    }
    /**
     * Sets a debounce delay between state-machine transitions.
     *
     * @param delay minimum delay (seconds) between transitions
     * @return this config for chaining
     */
    public MechanismConfig<T> setStateMachineDelay(double delay){
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
    public <E extends Enum<E>> MechanismConfig<T> setStateGraph(StateGraph<E> stateGraph){
        this.stateGraph = Objects.requireNonNull(stateGraph, "stateGraph");
        return this;
    }

    @FunctionalInterface
    public interface MechanismBinding<M extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {
        void apply(MechanismContext<M, E> context);
    }

    @FunctionalInterface
    public interface MechanismTransitionBinding<M extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {
        void apply(MechanismContext<M, E> context, E from, E to);
    }

    @FunctionalInterface
    public interface MechanismControlLoop<M extends Mechanism> {
        double calculate(MechanismControlContext<M> context);
    }

    public record ControlLoopBinding<M extends Mechanism>(
            String name,
            double periodSeconds,
            MechanismControlLoop<M> loop) { }

    public record StateTransitionPair<E extends Enum<E> & SetpointProvider<Double>>(E from, E to) { }

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

    public record PidProfile(OutputType outputType, double kP, double kI, double kD, double iZone, double tolerance) { }

    public record FeedforwardProfile(OutputType outputType, SimpleMotorFeedforward feedforward) {
        public FeedforwardProfile {
            Objects.requireNonNull(feedforward, "feedforward");
        }
    }

    public record ArmFeedforwardProfile(OutputType outputType, ArmFeedforward feedforward) {
        public ArmFeedforwardProfile {
            Objects.requireNonNull(feedforward, "feedforward");
        }
    }

    public record ElevatorFeedforwardProfile(OutputType outputType, ElevatorFeedforward feedforward) {
        public ElevatorFeedforwardProfile {
            Objects.requireNonNull(feedforward, "feedforward");
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
    public MechanismConfig<T> setVisualizationConfig(MechanismVisualizationConfig visualizationConfig) {
        this.visualizationConfig = Objects.requireNonNull(visualizationConfig);
        return this;
    }

    /**
     * Supplies a field-heading visualization for turret mechanisms (Field2d line for AdvantageScope).
     */
    public MechanismConfig<T> setTurretHeadingVisualization(Supplier<TurretMechanism.FieldHeadingVisualization> supplier) {
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
        public ElevatorSimulationParameters setCarriageMassKg(double carriageMassKg) {
            this.carriageMassKg = carriageMassKg;
            return this;
        }

        /**
         * Sets the radius of the winch drum in meters.
         *
         * @param drumRadiusMeters radius in meters
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters setDrumRadiusMeters(double drumRadiusMeters) {
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
        public ElevatorSimulationParameters setRangeMeters(double minHeightMeters, double maxHeightMeters) {
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
        public ElevatorSimulationParameters setStartingHeightMeters(double startingHeightMeters) {
            this.startingHeightMeters = startingHeightMeters;
            return this;
        }

        /**
         * Enables or disables gravity effects in the elevator simulation.
         *
         * @param simulateGravity true to include gravity
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters setSimulateGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        /**
         * Overrides the nominal battery voltage used to clamp the simulated motor output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public ElevatorSimulationParameters setNominalVoltage(double nominalVoltage) {
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
        public ElevatorSimulationParameters setUnitsPerMeter(double unitsPerMeter) {
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
        public ArmSimulationParameters setMotorReduction(double motorReduction) {
            this.motorReduction = motorReduction;
            return this;
        }

        /**
         * Sets the simulated arm's moment of inertia about the pivot.
         *
         * @param momentOfInertia inertia in kg·m^2
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters setMomentOfInertia(double momentOfInertia) {
            this.momentOfInertia = momentOfInertia;
            return this;
        }

        /**
         * Sets the link length measured from the pivot to the end effector.
         *
         * @param armLengthMeters arm length in meters
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters setArmLengthMeters(double armLengthMeters) {
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
        public ArmSimulationParameters setAngleRangeRadians(double minAngleRadians, double maxAngleRadians) {
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
        public ArmSimulationParameters setStartingAngleRadians(double startingAngleRadians) {
            this.startingAngleRadians = startingAngleRadians;
            return this;
        }

        /**
         * Enables or disables gravity torque in the arm simulation.
         *
         * @param simulateGravity true to include gravity
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters setSimulateGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        /**
         * Overrides the nominal battery voltage used to clamp simulated motor output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public ArmSimulationParameters setNominalVoltage(double nominalVoltage) {
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
        public ArmSimulationParameters setUnitsPerRadian(double unitsPerRadian) {
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
        public SimpleMotorSimulationParameters setMomentOfInertia(double momentOfInertia) {
            this.momentOfInertia = momentOfInertia;
            return this;
        }

        /**
         * Overrides the nominal battery voltage used to clamp simulated motor output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public SimpleMotorSimulationParameters setNominalVoltage(double nominalVoltage) {
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
        public SimpleMotorSimulationParameters setUnitsPerRadian(double unitsPerRadian) {
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
        MechanismConfigRecord cfg = data;
        if (autoContinuousPidForUnboundedTurret) {
            cfg = applyAutoContinuousPidForUnboundedTurret(cfg);
            data = cfg;
        }
        for (MotorControllerConfig motor : cfg.motors()) {
            motor.setNeutralMode(cfg.motorNeutralMode())
                    .setCurrentLimit(cfg.motorCurrentLimit())
                    .setCanbus(cfg.canbus());
        }

        if (cfg.encoder() != null) {
             cfg.encoder().setCanbus(cfg.canbus())
                    .setConversion(cfg.encoderConversion())
                    .setConversionOffset(cfg.encoderConversionOffset())
                    .setGearRatio(cfg.encoderGearRatio())
                    .setOffset(cfg.encoderOffset())
                    .setDiscontinuity(cfg.encoderDiscontinuityPoint(), cfg.encoderDiscontinuityRange());
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

        if (stateGraph != null && mechanism instanceof StatefulMechanism<?> statefulMechanism) {
            applyStateGraph(statefulMechanism, stateGraph);
        }

        return mechanism;
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
            if (config.type == null) {
                throw new IllegalStateException("Motor controller config is missing a type");
            }
            // Throws with a clear message if the vendor module is absent.
            MotorRegistry.get().motor(config.type.getKey());
        });
    }

    @SuppressWarnings("unchecked")
    private static <E extends Enum<E> & StateMachine.SetpointProvider<Double>> void applyStateGraph(
            StatefulMechanism<?> statefulMechanism,
            StateGraph<?> stateGraph) {
        StatefulMechanism<E> typedMechanism = (StatefulMechanism<E>) statefulMechanism;
        StateGraph<E> typedGraph = (StateGraph<E>) stateGraph;
        typedMechanism.setStateGraph(typedGraph);
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
            if (config.type == null) {
                throw new IllegalStateException("Motor controller config is missing a type");
            }
            MechanismSimulationConfig.requireSupportedMotorSim(config.type);
        });
    }

}
