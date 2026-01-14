package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
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
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSimulationModel;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismSensorSimulationConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Fluent builder that captures the hardware and control configuration for a {@link Mechanism}.
 * Teams populate this object with motor, sensor, and control metadata before calling
 * {@link #build()} to construct the runtime mechanism instance (and optional simulation models).
 *
 * @param <T> concrete mechanism type that will be created from this configuration
 */
public class MechanismConfig<T extends Mechanism> {

    /** Ordered list of motor controller configurations that will be grouped together. */
    public ArrayList<MotorControllerConfig> motors = new ArrayList<>();
    /** Primary encoder used for feedback, typically attached to the drivetrain output. */
    public EncoderConfig encoder = null;
    /** PID controller instance for traditional position/velocity control loops. */
    public PIDController pidController = null;
    /** Profiled PID controller that enforces velocity/acceleration constraints. */
    public ProfiledPIDController profiledPIDController = null;
    /** When true, treats the attached encoder as an absolute device during initialization. */
    public boolean useAbsolute = false;
    /** When true, assumes mechanism outputs are already in volts instead of a [-1, 1] scale. */
    public boolean useVoltage = false;
    /** When true, bypasses PID and writes the requested setpoint straight to the motor output. */
    public boolean useSetpointAsOutput = false;
    /** When true, PID compares velocity instead of position. */
    public boolean pidUseVelocity = false;
    /** Enables a user-specified control-loop period instead of WPILib's default loop timing. */
    public boolean customPIDCycle = false;
    /** Enables continuous PID input for cyclical mechanisms (turrets, wheels, etc.). */
    public boolean pidContinous = false;
    /** Factory used to instantiate the final mechanism once configuration is complete. */
    public Function<MechanismConfig<T>, T> factory = null;
    /** Additional limit switches wired into the mechanism for soft/hard stop behavior. */
    public ArrayList<GenericLimitSwitchConfig> limitSwitches = new ArrayList<>();
    
    /** CAN bus name shared by all motors/encoders in this mechanism (defaults to the roboRIO bus). */
    public String canbus = "rio";
    /** Overall gear ratio from motor encoder to mechanism output (motor rotations per output rotation). */
    public double encoderGearRatio = 1;
    /** Scalar that converts encoder units into meaningful output units (rad, meters, etc.). */
    public double encoderConversion = 1;
    /** Offset applied to the conversion factor once, useful for calibrating absolute encoders. */
    public double encoderConversionOffset = 0;
    /** Raw zeroing offset for the encoder reading, often captured during homing. */
    public double encoderOffset = 0;
    /** Peak current limit (amps) applied to each configured motor controller. */
    public double motorCurrentLimit = 40;
    /** Acceptable error band for PID controllers before considering the setpoint reached. */
    public double tolerance = 0;
    /** Delay (seconds) applied between state-machine transitions to debounce sequencing. */
    public double stateMachineDelay = 0;
    /** Custom control-loop period (seconds) when {@link #customPIDCycle} is true. */
    public double pidPeriod = 0;
    /** Integral zone width; integrator is only active when the error is within this band. */
    public double pidIZone = 0;
    /** Continuous input minimum bound used when {@link #pidContinous} is set. */
    public double continousMin, continousMax;
    /** Optional minimum bound for mechanism setpoints. */
    public double minBound = Double.NaN;
    /** Optional maximum bound for mechanism setpoints. */
    public double maxBound = Double.NaN;
    /** Optional motion limits applied to profiled PID constraints (max velocity/acceleration). */
    public MotionLimits.AxisLimits motionLimits = null;

    /** Neutral motor behavior applied to all controllers (coast vs. brake). */
    public MotorNeutralMode motorNeutralMode = MotorNeutralMode.Brake;
    /** Optional per-state callbacks that run when the mechanism state machine enters the state. */
    public Map<Enum<?>, Function<T, Boolean>> stateActions = new HashMap<>();
    /** Optional state hooks that run every loop while a state is active. */
    public Map<Enum<?>, List<MechanismBinding<T, ?>>> stateHooks = new HashMap<>();
    /** Optional hooks that run every loop regardless of the active state. */
    public List<MechanismBinding<T, ?>> alwaysHooks = new ArrayList<>();
    /** Optional boolean inputs exposed to state hooks. */
    public Map<String, BooleanSupplier> inputs = new HashMap<>();
    /** Optional double inputs exposed to state hooks. */
    public Map<String, DoubleSupplier> doubleInputs = new HashMap<>();
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

    /**
     * Creates a configuration builder that instantiates a plain {@link Mechanism} with no additional
     * behaviors. Useful for simple rollers or mocked subsystems.
     */
    public static MechanismConfig<Mechanism> generic(){
        return custom(Mechanism::new);
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
     * Creates a configuration that will build an {@link ElevatorMechanism} with feedforward control.
     *
     * @param feedforward feedforward model tuned for the elevator
     */
    public static MechanismConfig<ElevatorMechanism> elevator(ElevatorFeedforward feedforward) {
        return custom(config -> new ElevatorMechanism(config, feedforward));
    }

    /**
     * Creates a configuration that wraps a caller-supplied elevator mechanism factory.
     *
     * @param feedforward feedforward model to pass through to the factory
     * @param factory constructor logic for the elevator mechanism
     * @param <T> concrete elevator type created by the factory
     */
    public static <T extends ElevatorMechanism> MechanismConfig<T> elevator(ElevatorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful elevator configuration with an initial state and feedforward gains.
     *
     * @param feedforward feedforward model tuned for the elevator
     * @param initialState state machine starting point
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulElevatorMechanism<E>> statefulElevator(ElevatorFeedforward feedforward, E initialState) {
        return custom(config -> new StatefulElevatorMechanism<>(config, feedforward, initialState));
    }

    /**
     * Builds a stateful elevator configuration backed by a caller-supplied factory.
     *
     * @param feedforward feedforward model to pass through to the factory
     * @param factory constructor logic for the concrete elevator
     * @param <E> state enum type that provides setpoints
     * @param <T> concrete mechanism type created by the factory
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulElevatorMechanism<E>> MechanismConfig<T> statefulElevator(ElevatorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful arm configuration that uses the provided feedforward model.
     *
     * @param feedforward feedforward model tuned for the arm
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulArmMechanism<E>> statefulArm(ArmFeedforward feedforward, E initialState) {
        return custom(config -> new StatefulArmMechanism<>(config, feedforward, initialState));
    }

    /**
     * Builds a stateful arm configuration backed by a caller-supplied factory.
     *
     * @param feedforward feedforward model to pass through to the factory
     * @param factory constructor logic for the concrete arm mechanism
     * @param <E> state enum type that provides setpoints
     * @param <T> concrete mechanism type created by the factory
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulArmMechanism<E>> MechanismConfig<T> statefulArm(ArmFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful turret configuration (simple motor with continuous rotation) with feedforward support.
     *
     * @param feedforward feedforward model tuned for the mechanism
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulTurretMechanism<E>> statefulTurret(SimpleMotorFeedforward feedforward, E initialState) {
        return custom(config -> new StatefulTurretMechanism<>(config, feedforward, initialState));
    }

    /**
     * Builds a stateful turret configuration backed by a caller-supplied factory.
     *
     * @param feedforward feedforward model to pass through to the factory
     * @param factory constructor logic for the concrete mechanism
     * @param <E> state enum type that provides setpoints
     * @param <T> concrete mechanism type created by the factory
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulTurretMechanism<E>> MechanismConfig<T> statefulTurret(SimpleMotorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Builds a stateful flywheel configuration with feedforward support.
     *
     * @param feedforward feedforward model tuned for the flywheel
     * @param initialState starting state for the state machine
     * @param <E> state enum type that provides setpoints
     */
    public static <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<StatefulFlywheelMechanism<E>> statefulFlywheel(SimpleMotorFeedforward feedforward, E initialState) {
        return custom(config -> new StatefulFlywheelMechanism<>(config, feedforward, initialState));
    }

    /**
     * Builds a stateful flywheel configuration backed by a caller-supplied factory.
     */
    public static <E extends Enum<E> & SetpointProvider<Double>, T extends StatefulFlywheelMechanism<E>> MechanismConfig<T> statefulFlywheel(SimpleMotorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Creates a turret configuration that yields a {@link TurretMechanism}.
     *
     * @param feedforward feedforward model tuned for the mechanism
     */
    public static MechanismConfig<TurretMechanism> turret(SimpleMotorFeedforward feedforward) {
        return custom(config -> new TurretMechanism(config, feedforward));
    }

    /**
     * Creates a turret configuration backed by a caller-supplied factory.
     *
     * @param feedforward feedforward model to pass through to the factory
     * @param factory constructor logic for the concrete mechanism
     * @param <T> concrete mechanism type created by the factory
     */
    public static <T extends TurretMechanism> MechanismConfig<T> turret(SimpleMotorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Creates a flywheel configuration that yields a {@link FlywheelMechanism}.
     *
     * @param feedforward feedforward model tuned for the flywheel
     */
    public static MechanismConfig<FlywheelMechanism> flywheel(SimpleMotorFeedforward feedforward) {
        return custom(config -> new FlywheelMechanism(config, feedforward));
    }

    /**
     * Creates a flywheel configuration backed by a caller-supplied factory.
     */
    public static <T extends FlywheelMechanism> MechanismConfig<T> flywheel(SimpleMotorFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
        return custom(factory);
    }

    /**
     * Creates a configuration that builds a classic {@link ArmMechanism}.
     *
     * @param feedforward feedforward model tuned for the arm
     */
    public static MechanismConfig<ArmMechanism> arm(ArmFeedforward feedforward) {
        return custom(config -> new ArmMechanism(config, feedforward));
    }

    /**
     * Creates an arm configuration backed by a caller-supplied factory.
     *
     * @param feedforward feedforward model to pass through to the factory
     * @param factory constructor logic for the arm mechanism
     * @param <T> concrete mechanism type created by the factory
     */
    public static <T extends ArmMechanism> MechanismConfig<T> arm(ArmFeedforward feedforward, Function<MechanismConfig<T>, T> factory) {
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
     * Registers a motor controller configuration to be owned by this mechanism.
     *
     * @param config already-constructed motor configuration
     * @return this config for chaining
     */
    public MechanismConfig<T> addMotor(MotorControllerConfig config){
        motors.add(config);
        return this;
    }

    /**
     * Registers a motor controller by type and CAN ID.
     *
     * @param type motor controller platform (Falcon, SparkMax, etc.)
     * @param id CAN ID of the controller on the configured bus
     * @return this config for chaining
     */
    public MechanismConfig<T> addMotor(MotorControllerType type, int id){
        return addMotor(new MotorControllerConfig(type, id));
    }

    /**
     * Registers a motor controller using a team-facing {@link AthenaMotor} entry, which resolves to
     * the correct vendor-specific controller type through the hardware registry.
     *
     * @param motor logical motor entry (Falcon, Kraken, NEO, etc.)
     * @param id CAN ID of the controller on the configured bus (negative to mark inverted)
     * @return this config for chaining
     */
    public MechanismConfig<T> addMotor(AthenaMotor motor, int id) {
        return addMotor(motor.resolveController(), id);
    }

    /**
     * Registers multiple motor controllers of the same type in one call.
     *
     * @param type motor controller platform for all supplied IDs
     * @param ids CAN IDs to register; negative IDs invert the attached encoder when used with
     *            {@link #setEncoderFromMotor(int)}
     * @return this config for chaining
     */
    public MechanismConfig<T> addMotors(MotorControllerType type, int... ids){
        for (int i = 0; i < ids.length; i++) {
            addMotor(new MotorControllerConfig(type, ids[i]));
         }
         return this;
    }

    /**
     * Registers multiple controllers by logical motor type, mirroring
     * {@link #addMotors(MotorControllerType, int...)} while keeping caller code vendor-agnostic.
     *
     * @param motor logical motor entry (Falcon, Kraken, NEO, etc.)
     * @param ids CAN IDs to register; negative IDs mark inversion
     * @return this config for chaining
     */
    public MechanismConfig<T> addMotors(AthenaMotor motor, int... ids) {
        return addMotors(motor.resolveController(), ids);
    }

    /**
     * Specifies the encoder configuration to use for feedback.
     *
     * @param encoder encoder configuration (absolute/relative, ports, offsets)
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoder(EncoderConfig encoder){
        this.encoder  = encoder;
        return this;
    }

    /**
     * Creates and registers an encoder configuration from a type/ID pair.
     *
     * @param type encoder hardware type
     * @param id primary identifier (CAN ID, DIO port, etc.)
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoder(EncoderType type, int id){
        return setEncoder(EncoderConfig.type(type, id));
    }

    /**
     * Creates and registers an encoder configuration from a logical {@link AthenaEncoder}, keeping
     * mechanism configuration vendor-agnostic at call sites.
     *
     * @param encoder logical encoder entry
     * @param id primary identifier (CAN ID, DIO port, etc.)
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoder(AthenaEncoder encoder, int id) {
        return setEncoder(encoder.resolve(), id);
    }

    /**
     * Associates the mechanism encoder with one of the configured motors. Passing a negative ID
     * marks the encoder as inverted.
     *
     * @param id CAN ID of the motor whose integrated sensor should be used (negative to invert)
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoderFromMotor(int id){
        return setEncoder(motors.stream().filter((motors) -> motors.id == Math.abs(id)).findFirst().get().encoderConfig.setInverted(id < 0));
    }

    /**
     * Convenience helper that constructs a {@link PIDController} with the supplied gains.
     *
     * @param p proportional gain
     * @param i integral gain
     * @param d derivative gain
     * @return this config for chaining
     */
    public MechanismConfig<T> setPID(double p, double i, double d){
        return setPID(new PIDController(p, i, d));
    }

    /**
     * Provides a fully configured {@link PIDController} to use for closed-loop control.
     *
     * @param pidController controller instance
     * @return this config for chaining
     */
    public MechanismConfig<T> setPID(PIDController pidController){
        this.pidController = pidController;
        return this;
    }

    /**
     * Builds and registers a {@link ProfiledPIDController} with the supplied gains and motion
     * constraints.
     *
     * @param p proportional gain
     * @param i integral gain
     * @param d derivative gain
     * @param maxVel maximum goal velocity
     * @param maxAccel maximum goal acceleration
     * @return this config for chaining
     */
    public MechanismConfig<T> setProfiledPID(double p, double i, double d, double maxVel, double maxAccel){
        return setProfiledPID(p, i, d, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }

    /**
     * Builds and registers a {@link ProfiledPIDController} with custom trapezoidal constraints.
     *
     * @param p proportional gain
     * @param i integral gain
     * @param d derivative gain
     * @param constraints trapezoidal constraints for position goals
     * @return this config for chaining
     */
    public MechanismConfig<T> setProfiledPID(double p, double i, double d, TrapezoidProfile.Constraints constraints){
        return setProfiledPID(new ProfiledPIDController(p, i, d, constraints));
    }

    /**
     * Registers a fully constructed {@link ProfiledPIDController}.
     *
     * @param profiledPIDController controller instance
     * @return this config for chaining
     */
    public MechanismConfig<T> setProfiledPID(ProfiledPIDController profiledPIDController){
        this.profiledPIDController = profiledPIDController;
        return this;
    }

    /**
     * Sets the base motion limits used to clamp profiled PID constraints.
     *
     * @param limits max velocity/acceleration caps for this mechanism
     * @return this config for chaining
     */
    public MechanismConfig<T> setMotionLimits(MotionLimits.AxisLimits limits) {
        this.motionLimits = limits;
        return this;
    }

    /**
     * Sets the base motion limits used to clamp profiled PID constraints.
     *
     * @param maxVelocity maximum velocity
     * @param maxAcceleration maximum acceleration
     * @return this config for chaining
     */
    public MechanismConfig<T> setMotionLimits(double maxVelocity, double maxAcceleration) {
        return setMotionLimits(new MotionLimits.AxisLimits(maxVelocity, maxAcceleration));
    }

    /**
     * Sets the acceptable error band for whichever PID controller is active.
     *
     * @param tolerance tolerance in mechanism units
     * @return this config for chaining
     */
    public MechanismConfig<T> setTolerance(double tolerance){
        this.tolerance = tolerance;
        return this;
    }
    /**
     * Sets the CAN bus used for all motors and sensors in this mechanism.
     *
     * @param canbus bus name such as {@code "rio"} or a pinned CANivore name
     * @return this config for chaining
     */
    public MechanismConfig<T> setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    }

    /**
     * Declares the gear ratio between the motor encoder and the mechanism output.
     *
     * @param ratio motor rotations per output rotation
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoderGearRatio(double ratio){
        this.encoderGearRatio = ratio;
        return this;
    }

    /**
     * Specifies a conversion factor from encoder units to real-world units (radians, meters, etc.).
     *
     * @param conversion multiplier applied to raw encoder units
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoderConversion(double conversion){
        this.encoderConversion = conversion;
        return this;
    }

    /**
     * Applies an additive offset to the encoder conversion factor once during {@link #build()}.
     *
     * @param conversionOffset offset applied to the conversion scalar
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoderConversionOffset(double conversionOffset){
        this.encoderConversionOffset = conversionOffset;
        return this;
    }

    /**
     * Sets a raw encoder offset in native units (useful for zeroing absolute encoders).
     *
     * @param offset raw sensor offset
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoderOffset(double offset){
        this.encoderOffset = offset;
        return this;
    }

    /**
     * Sets the peak current limit sent to each registered motor controller.
     *
     * @param limit current limit in amps
     * @return this config for chaining
     */
    public MechanismConfig<T> setCurrentLimit(double limit){
        this.motorCurrentLimit = limit;
        return this;
    }

    /**
     * Mutates the current encoder configuration via a {@link Consumer}. The encoder must already be
     * configured by {@link #setEncoder(EncoderConfig)}.
     *
     * @param func consumer that mutates the encoder
     * @return this config for chaining
     */
    public MechanismConfig<T> setEncoderConfig(Consumer<EncoderConfig> func){
        func.accept(encoder);
        return this;
    }

    /**
     * Marks the attached encoder as absolute so the mechanism initializes using its reported angle.
     *
     * @param useAbsolute whether to treat the encoder as absolute
     * @return this config for chaining
     */
    public MechanismConfig<T> setUseEncoderAbsolute(boolean useAbsolute){
        this.useAbsolute = useAbsolute;
        return this;
    }

    /**
     * Indicates that mechanism outputs will be provided in volts instead of a normalized [-1, 1]
     * duty-cycle signal.
     *
     * @param useVoltage whether the mechanism outputs real volt commands
     * @return this config for chaining
     */
    public MechanismConfig<T> setUseVoltage(boolean useVoltage){
        this.useVoltage = useVoltage;
        return this;
    }

    /**
     * Declares the neutral behavior to apply to every registered motor controller.
     *
     * @param mode neutral mode (brake/coast)
     * @return this config for chaining
     */
    public MechanismConfig<T> setNeutralMode(MotorNeutralMode mode){
        this.motorNeutralMode = mode;
        return this;
    }

    /**
     * Binds a {@link MechanismSensorSimulationConfig} that produces virtual sensor readings when the
     * robot runs in simulation.
     *
     * @param config sensor simulation configuration
     * @return this config for chaining
     */
    public MechanismConfig<T> setSensorSimulation(MechanismSensorSimulationConfig config) {
        this.sensorSimulationConfig = config;
        return this;
    }

    /**
     * Registers an additional limit switch configuration with the mechanism.
     *
     * @param config limit switch configuration (device type, thresholds)
     * @return this config for chaining
     */
    public MechanismConfig<T> addLimitSwitch(GenericLimitSwitchConfig config) {
        limitSwitches.add(config);
        return this;
    }

    /**
     * Bypasses the PID loop and directly applies state-machine setpoints to the motor when true.
     *
     * @param useSetpointAsOutput whether to forward the raw setpoint value to the motor output
     * @return this config for chaining
     */
    public MechanismConfig<T> setUseSetpointAsOutput(boolean useSetpointAsOutput) {
        this.useSetpointAsOutput = useSetpointAsOutput;
        return this;
    }

    /**
     * Uses velocity as the PID measurement instead of position.
     *
     * @param useVelocity true to close the loop on velocity
     * @return this config for chaining
     */
    public MechanismConfig<T> setPidUseVelocity(boolean useVelocity) {
        this.pidUseVelocity = useVelocity;
        return this;
    }

    /**
     * Adds a lower travel limit switch with a soft-stop position.
     *
     * @param id device identifier
     * @param position mechanism position associated with the switch trip
     * @return this config for chaining
     */
    public MechanismConfig<T> addLowerLimitSwitch(int id, double position){
        return addLimitSwitch(id, position, false, 0);
    }

    /**
     * Adds a lower travel limit switch that optionally hard-stops the mechanism.
     *
     * @param id device identifier
     * @param position mechanism position associated with the switch trip
     * @param stopMotors true to cut motor output when tripped
     * @return this config for chaining
     */
    public MechanismConfig<T> addLowerLimitSwitch(int id, double position, boolean stopMotors){
        return addLimitSwitch(GenericLimitSwitchConfig.create(id).setPosition(position).setHardstop(stopMotors, -1));
    }

    /**
     * Adds an upper travel limit switch with a soft-stop position.
     *
     * @param id device identifier
     * @param position mechanism position associated with the switch trip
     * @return this config for chaining
     */
    public MechanismConfig<T> addUpperLimitSwitch(int id, double position){
        return addLimitSwitch(id, position, false, 0);
    }

    /**
     * Adds an upper travel limit switch that optionally hard-stops the mechanism.
     *
     * @param id device identifier
     * @param position mechanism position associated with the switch trip
     * @param stopMotors true to cut motor output when tripped
     * @return this config for chaining
     */
    public MechanismConfig<T> addUpperLimitSwitch(int id, double position, boolean stopMotors){
        return addLimitSwitch(GenericLimitSwitchConfig.create(id).setPosition(position).setHardstop(stopMotors, 1));
    }

    /**
     * Adds a limit switch that defaults to soft-stop behavior.
     *
     * @param id device identifier
     * @param position mechanism position associated with the switch trip
     * @return this config for chaining
     */
    public MechanismConfig<T> addLimitSwitch(int id, double position){
        return addLimitSwitch(id, position, false, 0);
    }

    /**
     * Adds a limit switch with full control over motor suppression behavior.
     *
     * @param id device identifier
     * @param position mechanism position associated with the switch trip
     * @param stopMotors true to cut motor output when tripped
     * @param blockDirection direction multiplier (1, -1, or 0) that blocks motion past the switch
     * @return this config for chaining
     */
    public MechanismConfig<T> addLimitSwitch(int id, double position, boolean stopMotors, int blockDirection){
        return addLimitSwitch(GenericLimitSwitchConfig.create(id).setPosition(position).setHardstop(stopMotors, blockDirection));
    }

    /**
     * Sets a debounce delay between state-machine transitions.
     *
     * @param delay minimum delay (seconds) between transitions
     * @return this config for chaining
     */
    public MechanismConfig<T> setStateMachineDelay(double delay){
        this.stateMachineDelay = delay;
        return this;
    }

    /**
     * Clamps all setpoints to the provided bounds.
     *
     * @param min minimum setpoint value
     * @param max maximum setpoint value
     * @return this config for chaining
     */
    public MechanismConfig<T> setBounds(double min, double max) {
        this.minBound = min;
        this.maxBound = max;
        return this;
    }

    /**
     * Clears any configured setpoint bounds.
     *
     * @return this config for chaining
     */
    public MechanismConfig<T> clearBounds() {
        this.minBound = Double.NaN;
        this.maxBound = Double.NaN;
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

    /**
     * Adds a named external boolean input for state hooks.
     */
    public MechanismConfig<T> addInput(String key, BooleanSupplier supplier) {
        inputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
        return this;
    }

    /**
     * Adds a named boolean input (alias for {@link #addInput(String, BooleanSupplier)}).
     */
    public MechanismConfig<T> addBooleanInput(String key, BooleanSupplier supplier) {
        return addInput(key, supplier);
    }

    /**
     * Adds a named double input for state hooks.
     */
    public MechanismConfig<T> addDoubleInput(String key, DoubleSupplier supplier) {
        doubleInputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
        return this;
    }

    /**
     * Adds a named object input for state hooks.
     */
    public MechanismConfig<T> addObjectInput(String key, Supplier<?> supplier) {
        objectInputs.put(Objects.requireNonNull(key, "key"), Objects.requireNonNull(supplier, "supplier"));
        return this;
    }

    /**
     * Registers a hook that runs every loop while the supplied states are active.
     */
    @SafeVarargs
    public final <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<T> addOnStateHook(
            MechanismBinding<T, E> binding,
            E... states) {
        Objects.requireNonNull(binding, "binding");
        if (states == null || states.length == 0) {
            alwaysHooks.add(binding);
            return this;
        }
        for (E state : states) {
            Objects.requireNonNull(state, "states cannot contain null");
            stateHooks.computeIfAbsent(state, key -> new ArrayList<>()).add(binding);
        }
        return this;
    }

    /**
     * Registers a hook that runs every loop regardless of the active state.
     */
    public <E extends Enum<E> & SetpointProvider<Double>> MechanismConfig<T> addOnStateHook(
            MechanismBinding<T, E> binding) {
        Objects.requireNonNull(binding, "binding");
        alwaysHooks.add(binding);
        return this;
    }

    @FunctionalInterface
    public interface MechanismBinding<M extends Mechanism, E extends Enum<E> & SetpointProvider<Double>> {
        void apply(MechanismContext<M, E> context);
    }

    /**
     * Registers a callback that runs whenever the state machine enters any of the supplied states.
     * Returning {@code true} from the callback suppresses motor output for that loop iteration.
     *
     * @param action callback that receives the built mechanism and returns whether to suppress motors
     * @param states states that trigger the callback
     * @return this config for chaining
     */
    public MechanismConfig<T> setStateAction(Function<T, Boolean> action, Enum<?>... states) {
        Arrays.stream(states).forEach(state -> stateActions.put(state, action));
        return this; 
    }

    /**
     * Registers a state-entry callback that never suppresses motor output.
     *
     * @param action callback to run when entering the provided states
     * @param states states that trigger the callback
     * @return this config for chaining
     */
    public MechanismConfig<T> setStateAction(Consumer<T> action, Enum<?>... states) {
        return setStateAction(mech -> {action.accept(mech); return false;}, states);
    }

    /**
     * Registers a state-entry callback that suppresses motor output while it executes.
     *
     * @param action callback to run when entering the provided states
     * @param states states that trigger the callback
     * @return this config for chaining
     */
    public MechanismConfig<T> setStateActionSupressMotors(Consumer<T> action, Enum<?>... states) {
        return setStateAction(mech -> {action.accept(mech); return true;}, states);
    }

    /**
     * Specifies a custom PID loop period to run outside of the default 20 ms cycle.
     *
     * @param customPIDCycle whether the mechanism manages its own loop timing
     * @param period loop period in seconds when {@code customPIDCycle} is true
     * @return this config for chaining
     */
    public MechanismConfig<T> setCustomPIDCycle(boolean customPIDCycle, double period) {
        this.customPIDCycle = customPIDCycle;
        this.pidPeriod = period;
        return this;
    }

    /**
     * Sets the integral zone used by the configured PID controller(s).
     *
     * @param pidIZone error band in mechanism units within which I gains are active
     * @return this config for chaining
     */
    public MechanismConfig<T> setPIDIZone(double pidIZone){
        this.pidIZone = pidIZone;
        return this;
    }

    /**
     * Enables continuous PID input for mechanisms that wrap (e.g., turret angles).
     *
     * @param continousMin lower bound of the wrap range
     * @param continousMax upper bound of the wrap range
     * @return this config for chaining
     */
    public MechanismConfig<T> setPIDEnableContinous(double continousMin, double continousMax){
        this.continousMin = continousMin;
        this.continousMax = continousMax;
        return this;
    }

    /**
     * Attaches a pre-built simulation configuration to the mechanism.
     *
     * @param simulationConfig simulation model configuration
     * @return this config for chaining
     */
    public MechanismConfig<T> setSimulationConfig(MechanismSimulationConfig simulationConfig){
        this.simulationConfig = simulationConfig;
        return this;
    }

    /**
     * Supplies a custom simulation factory that consumes the built mechanism.
     *
     * @param simulationFactory function that returns a {@link MechanismSimulationModel} for the
     *                          constructed mechanism
     * @return this config for chaining
     */
    @SuppressWarnings("unchecked")
    public MechanismConfig<T> setSimulation(Function<T, MechanismSimulationModel> simulationFactory){
        Objects.requireNonNull(simulationFactory);
        this.simulationConfig = MechanismSimulationConfig.builder()
                .withFactory(mechanism -> simulationFactory.apply((T) mechanism))
                .build();
        return this;
    }

    /**
     * Sets elevator-specific simulation parameters. Use this when the mechanism is configured as an
     * elevator so Athena can derive sane defaults for the physics model without asking for duplicate
     * information.
     *
     * @param parameters optional hints about the elevator's physical properties
     * @return this config for chaining
     */
    public MechanismConfig<T> setSimulationElevator(ElevatorSimulationParameters parameters) {
        this.elevatorSimulationParameters = Objects.requireNonNull(parameters);
        return this;
    }

    /**
     * Sets arm-specific simulation parameters. Use this for rotary joints (shoulder, wrist, etc.)
     * to feed additional data such as inertia or angle limits into the simulation.
     *
     * @param parameters optional hints about the arm's physical properties
     * @return this config for chaining
     */
    public MechanismConfig<T> setSimulationArm(ArmSimulationParameters parameters) {
        this.armSimulationParameters = Objects.requireNonNull(parameters);
        return this;
    }

    /**
     * Sets simple-motor simulation parameters. Targets single-axis mechanisms driven mainly by
     * velocity (rollers, flywheels, etc.).
     *
     * @param parameters optional hints about the mechanism inertia and feedback conversion
     * @return this config for chaining
     */
    public MechanismConfig<T> setSimulationSimpleMotor(SimpleMotorSimulationParameters parameters) {
        this.simpleMotorSimulationParameters = Objects.requireNonNull(parameters);
        return this;
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

        motors.forEach(
            (motor) -> motor.setNeutralMode(motorNeutralMode)
                            .setCurrentLimit(motorCurrentLimit)
                            .setCanbus(canbus)
                            );

        if (encoder != null) {
             encoder.setCanbus(canbus)
                    .setConversion(encoderConversion)
                    .setConversionOffset(encoderConversionOffset)
                    .setGearRatio(encoderGearRatio)
                    .setOffset(encoderOffset);
        }

        if(pidController != null){
            pidController.setTolerance(tolerance);
            pidController.setIZone(pidIZone);
            if(pidContinous){
                pidController.enableContinuousInput(continousMin, continousMax);
            }
        }

        if(profiledPIDController != null){
            profiledPIDController.setTolerance(tolerance);
            profiledPIDController.setIZone(pidIZone);
            if(pidContinous){
                profiledPIDController.enableContinuousInput(continousMin, continousMax);
            }
        }

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
        if (motors == null || motors.isEmpty()) {
            return;
        }
        motors.forEach(config -> {
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
        if (motors == null || motors.isEmpty()) {
            throw new IllegalStateException("Simulation requires at least one motor controller to be configured");
        }
        motors.forEach(config -> {
            if (config.type == null) {
                throw new IllegalStateException("Motor controller config is missing a type");
            }
            MechanismSimulationConfig.requireSupportedMotorSim(config.type);
        });
    }
}
