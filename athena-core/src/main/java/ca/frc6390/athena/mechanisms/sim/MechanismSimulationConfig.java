package ca.frc6390.athena.mechanisms.sim;

import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.mechanisms.ArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismTravelRange;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Declarative configuration for mechanism simulations. Teams can either provide a fully custom
 * factory or use the bundled helpers for single-jointed arms, elevators, and simple flywheel-style
 * mechanisms.
 */
public final class MechanismSimulationConfig {

    private final Function<Mechanism, MechanismSimulationModel> factory;
    private final double updatePeriodSeconds;
    private MechanismConfig<?> sourceConfig;

    private MechanismSimulationConfig(Function<Mechanism, MechanismSimulationModel> factory,
                                      double updatePeriodSeconds) {
        this.factory = Objects.requireNonNull(factory);
        if (!(updatePeriodSeconds > 0.0)) {
            throw new IllegalArgumentException("Simulation update period must be positive");
        }
        this.updatePeriodSeconds = updatePeriodSeconds;
    }

    public MechanismSimulationModel createSimulation(Mechanism mechanism) {
        return factory.apply(mechanism);
    }

    public double updatePeriodSeconds() {
        return updatePeriodSeconds;
    }

    public static MechanismSimulationConfig of(Function<Mechanism, MechanismSimulationModel> factory) {
        return builder().withFactory(factory).build();
    }

    public static Builder builder() {
        return new Builder();
    }

    /**
     * Internal hook used by {@link MechanismConfig#build()} so the simulation config can read the
     * original mechanism builder parameters (motor list, encoder gearing, etc.). Users should not
     * call this directly.
     */
    public MechanismSimulationConfig bindSourceConfig(MechanismConfig<?> sourceConfig) {
        this.sourceConfig = sourceConfig;
        return this;
    }

    private double getGearingFromConfig() {
        if (sourceConfig == null) {
            return Double.NaN;
        }
        double ratio = sourceConfig.data().encoderGearRatio();
        if (Math.abs(ratio) < 1e-6) {
            return Double.NaN;
        }
        return Math.abs(1.0 / ratio);
    }

    private static double applyMotorInversion(Mechanism mechanism, double output) {
        if (mechanism == null) {
            return output;
        }
        MotorControllerGroup group = mechanism.motors().device();
        if (group == null) {
            return output;
        }
        MotorController[] controllers = group.getControllers();
        if (controllers == null || controllers.length == 0) {
            return output;
        }
        return controllers[0].isInverted() ? -output : output;
    }

    private double deriveDrumRadiusMeters() {
        if (sourceConfig == null) {
            return 0.02;
        }
        double conversion = sourceConfig.data().encoderConversion();
        double ratio = sourceConfig.data().encoderGearRatio();
        if (Math.abs(conversion) < 1e-6 || Math.abs(ratio) < 1e-6) {
            return 0.02;
        }
        double linearPerOutputRotation = conversion * ratio;
        if (!Double.isFinite(linearPerOutputRotation) || Math.abs(linearPerOutputRotation) < 1e-6) {
            return 0.02;
        }
        return Math.abs(linearPerOutputRotation / (2.0 * Math.PI));
    }

    private double deriveUnitsPerMeter(double drumRadiusMeters) {
        if (sourceConfig == null) {
            return 1.0;
        }
        if (!Double.isFinite(drumRadiusMeters) || Math.abs(drumRadiusMeters) < 1e-6) {
            drumRadiusMeters = 0.02;
        }
        double conversion = sourceConfig.data().encoderConversion();
        double ratio = sourceConfig.data().encoderGearRatio();
        if (Math.abs(conversion) < 1e-6 || Math.abs(ratio) < 1e-6) {
            return 1.0;
        }
        double unitsPerOutputRotation = conversion * ratio;
        double metersPerOutputRotation = 2.0 * Math.PI * drumRadiusMeters;
        if (Math.abs(metersPerOutputRotation) < 1e-6) {
            return 1.0;
        }
        return unitsPerOutputRotation / metersPerOutputRotation;
    }

    private double deriveUnitsPerRadian() {
        if (sourceConfig == null) {
            return 1.0;
        }
        double conversion = sourceConfig.data().encoderConversion();
        double ratio = sourceConfig.data().encoderGearRatio();
        if (Math.abs(conversion) < 1e-6 || Math.abs(ratio) < 1e-6) {
            return 1.0;
        }
        double unitsPerOutputRotation = conversion * ratio;
        return unitsPerOutputRotation / (2.0 * Math.PI);
    }

    /**
     * Helper for constructing an {@link ElevatorMechanism} simulation using {@link ElevatorSim}.
     */
    /**
     * Creates an elevator-suitable simulation config. Supply optional physical parameters via the
     * builder; any missing values will fall back on the mechanism configuration (motor gearing,
     * encoder conversion) or reasonable defaults.
     *
     * @param params optional physical parameters for the elevator model
     * @return simulation configuration
     */
    public static MechanismSimulationConfig elevator(ElevatorParameters params) {
        Objects.requireNonNull(params);
        final MechanismSimulationConfig[] holder = new MechanismSimulationConfig[1];
        MechanismSimulationConfig config = new MechanismSimulationConfig(
                mechanism -> {
                    MechanismSimulationConfig self = holder[0];
                    if (!(mechanism instanceof ElevatorMechanism elevator)) {
                        throw new IllegalArgumentException(
                                "Elevator simulation requires an ElevatorMechanism instance.");
                    }
                    DCMotor motor = resolveMotor(mechanism, params.motorOverride);
                    MechanismConfig.ElevatorSimulationParameters configParams =
                            self != null && self.sourceConfig != null
                                    ? self.sourceConfig.elevatorSimulationParameters()
                                    : null;
                    double gearing = !Double.isNaN(params.gearing)
                            ? params.gearing
                            : (!Double.isNaN(self.getGearingFromConfig()) ? self.getGearingFromConfig() : 1.0);
                    double carriageMass = !Double.isNaN(params.carriageMassKg)
                            ? params.carriageMassKg
                            : (configParams != null && !Double.isNaN(configParams.carriageMassKg)
                                    ? configParams.carriageMassKg
                                    : 20.0);
                    double drumRadius = !Double.isNaN(params.drumRadiusMeters)
                            ? params.drumRadiusMeters
                            : (configParams != null && !Double.isNaN(configParams.drumRadiusMeters)
                                    ? configParams.drumRadiusMeters
                                    : self.deriveDrumRadiusMeters());
                    double minHeight = configParams != null ? configParams.minHeightMeters : params.minHeightMeters;
                    double maxHeight = configParams != null ? configParams.maxHeightMeters : params.maxHeightMeters;
                    double startingHeight = configParams != null
                            ? configParams.startingHeightMeters
                            : params.startingHeightMeters;
                    boolean simulateGravity = configParams != null
                            ? configParams.simulateGravity
                            : params.simulateGravity;
                    double unitsPerMeter = !Double.isNaN(params.unitsPerMeter)
                            ? params.unitsPerMeter
                            : self.deriveUnitsPerMeter(drumRadius);
                    if (configParams != null && !Double.isNaN(configParams.unitsPerMeterOverride)) {
                        unitsPerMeter = configParams.unitsPerMeterOverride;
                    }
                    double nominalVoltage = configParams != null ? configParams.nominalVoltage : params.nominalVoltage;

                    ElevatorSim sim = new ElevatorSim(
                            motor,
                            gearing,
                            carriageMass,
                            drumRadius,
                            minHeight,
                            maxHeight,
                            simulateGravity,
                            startingHeight);
                    return new ElevatorModel(elevator, sim, unitsPerMeter, nominalVoltage, minHeight, maxHeight);
                },
                params.updatePeriodSeconds);
        holder[0] = config;
        return config;
    }

    /**
     * Helper for constructing a {@link ArmMechanism} simulation using {@link SingleJointedArmSim}.
     */
    /**
     * Creates a rotary-joint simulation config (single-axis arm/wrist/turret). Missing values are
     * inferred from the mechanism setup when possible.
     *
     * @param params optional physical parameters for the arm model
     * @return simulation configuration
     */
    public static MechanismSimulationConfig arm(ArmParameters params) {
        Objects.requireNonNull(params);
        final MechanismSimulationConfig[] holder = new MechanismSimulationConfig[1];
        MechanismSimulationConfig config = new MechanismSimulationConfig(
                mechanism -> {
                    MechanismSimulationConfig self = holder[0];
                    if (!(mechanism instanceof ArmMechanism arm)) {
                        throw new IllegalArgumentException(
                                "Arm simulation requires an ArmMechanism instance.");
                    }
                    DCMotor motor = resolveMotor(mechanism, params.motorOverride);
                    MechanismConfig.ArmSimulationParameters configParams =
                            self != null && self.sourceConfig != null
                                    ? self.sourceConfig.armSimulationParameters()
                                    : null;
                    double gearing = !Double.isNaN(params.gearing)
                            ? params.gearing
                            : (!Double.isNaN(self.getGearingFromConfig()) ? self.getGearingFromConfig() : 1.0);
                    double momentOfInertia = !Double.isNaN(params.momentOfInertia)
                            ? params.momentOfInertia
                            : (configParams != null && !Double.isNaN(configParams.momentOfInertia)
                                    ? configParams.momentOfInertia
                                    : 1.0);
                    double armLength = !Double.isNaN(params.armLengthMeters)
                            ? params.armLengthMeters
                            : (configParams != null && !Double.isNaN(configParams.armLengthMeters)
                                    ? configParams.armLengthMeters
                                    : 0.5);
                    double minAngle = configParams != null ? configParams.minAngleRadians : params.minAngleRadians;
                    double maxAngle = configParams != null ? configParams.maxAngleRadians : params.maxAngleRadians;
                    double startingAngle = configParams != null
                            ? configParams.startingAngleRadians
                            : params.startingAngleRadians;
                    boolean simulateGravity = configParams != null
                            ? configParams.simulateGravity
                            : params.simulateGravity;
                    double unitsPerRadian = !Double.isNaN(params.unitsPerRadian)
                            ? params.unitsPerRadian
                            : self.deriveUnitsPerRadian();
                    if (configParams != null && !Double.isNaN(configParams.unitsPerRadianOverride)) {
                        unitsPerRadian = configParams.unitsPerRadianOverride;
                    }
                    double nominalVoltage = configParams != null ? configParams.nominalVoltage : params.nominalVoltage;

                    SingleJointedArmSim sim = new SingleJointedArmSim(
                            motor,
                            gearing,
                            momentOfInertia,
                            armLength,
                            minAngle,
                            maxAngle,
                            simulateGravity,
                            startingAngle);
                    return new ArmModel(arm, sim, unitsPerRadian, nominalVoltage, minAngle, maxAngle);
                },
                params.updatePeriodSeconds);
        holder[0] = config;
        return config;
    }

    /**
     * Creates a generic simple-motor simulation (flywheel/roller). Uses mechanism motor data for
     * the motor model and supports optional overrides like inertia or units-per-radian.
     *
     * @param params optional physical parameters for the simple-motor model
     * @return simulation configuration
     */
    public static MechanismSimulationConfig simpleMotor(SimpleMotorParameters params) {
    Objects.requireNonNull(params);
        final MechanismSimulationConfig[] holder = new MechanismSimulationConfig[1];
        MechanismSimulationConfig config = new MechanismSimulationConfig(
                mechanism -> {
                    MechanismSimulationConfig self = holder[0];
                    Mechanism target = mechanism;
                    DCMotor motor = resolveMotor(mechanism, params.motorOverride);
                    MechanismConfig.SimpleMotorSimulationParameters configParams =
                            self != null && self.sourceConfig != null
                                    ? self.sourceConfig.simpleMotorSimulationParameters()
                                    : null;
                    double gearing = !Double.isNaN(params.gearing)
                            ? params.gearing
                            : (!Double.isNaN(self.getGearingFromConfig()) ? self.getGearingFromConfig() : 1.0);
                    double moi = !Double.isNaN(params.momentOfInertia)
                            ? params.momentOfInertia
                            : (configParams != null && !Double.isNaN(configParams.momentOfInertia)
                                    ? configParams.momentOfInertia
                                    : 0.01);
                    double unitsPerRadian = !Double.isNaN(params.unitsPerRadian)
                            ? params.unitsPerRadian
                            : self.deriveUnitsPerRadian();
                    if (configParams != null && !Double.isNaN(configParams.unitsPerRadianOverride)) {
                        unitsPerRadian = configParams.unitsPerRadianOverride;
                    }
                    double nominalVoltage = configParams != null ? configParams.nominalVoltage : params.nominalVoltage;

                    FlywheelSim sim = new FlywheelSim(
                            LinearSystemId.createFlywheelSystem(
                                    motor,
                                    moi,
                                    gearing),
                            motor);
                    return new SimpleMotorModel(target, sim, unitsPerRadian, nominalVoltage);
                },
                params.updatePeriodSeconds);
        holder[0] = config;
        return config;
    }

    public static final class Builder {
        private Function<Mechanism, MechanismSimulationModel> factory;
        private double updatePeriodSeconds = 0.02;

        private Builder() {
        }

        /**
         * Supplies the factory that produces a simulation model for the mechanism instance.
         *
         * @param factory function that creates a {@link MechanismSimulationModel}
         * @return this builder for chaining
         */
        public Builder withFactory(Function<Mechanism, MechanismSimulationModel> factory) {
            this.factory = Objects.requireNonNull(factory);
            return this;
        }

        /**
         * Sets the update period used when stepping the simulation model.
         *
         * @param updatePeriodSeconds loop period in seconds
         * @return this builder for chaining
         */
        public Builder withUpdatePeriodSeconds(double updatePeriodSeconds) {
            this.updatePeriodSeconds = updatePeriodSeconds;
            return this;
        }

        /**
         * Builds the immutable {@link MechanismSimulationConfig}.
         *
         * @return constructed simulation configuration
         */
        public MechanismSimulationConfig build() {
            return new MechanismSimulationConfig(
                    Objects.requireNonNull(factory, "Simulation factory must be provided"),
                    updatePeriodSeconds);
        }
    }

    /**
     * Validates that the given motor type can be mapped to a simulation model.
     *
     * @param type motor controller type to validate
     */
    public static void requireSupportedMotorSim(MotorControllerType type) {
        motorFromType(Objects.requireNonNull(type, "Motor type is required for simulation"), 1);
    }

    private static DCMotor resolveMotor(Mechanism mechanism, DCMotor override) {
        if (override != null) {
            return override;
        }
        if (mechanism == null) {
            throw new IllegalStateException("Cannot derive simulation motor without a mechanism reference");
        }

        MotorControllerGroup group = mechanism.motors().device();
        if (group == null) {
            throw new IllegalStateException("Cannot derive simulation motor without a motor group");
        }

        MotorController[] controllers = group.getControllers();
        if (controllers == null || controllers.length == 0) {
            throw new IllegalStateException("Simulation motor derivation requires at least one motor controller");
        }

        MotorControllerType baseType = null;
        int motorCount = 0;
        boolean mixedTypes = false;

        for (MotorController controller : controllers) {
            if (controller == null) {
                continue;
            }
            motorCount++;
            MotorControllerType controllerType = controller.getType();
            if (controllerType == null) {
                continue;
            }
            if (baseType == null) {
                baseType = controllerType;
            } else if (controllerType != baseType) {
                mixedTypes = true;
            }
        }

        if (motorCount <= 0) {
            throw new IllegalStateException("Simulation motor derivation requires at least one motor controller");
        }

        if (baseType == null || mixedTypes) {
            throw new IllegalStateException("Simulation motor derivation requires a consistent motor controller type");
        }

        return motorFromType(baseType, motorCount);
    }

    private static DCMotor motorFromType(MotorControllerType type, int motorCount) {
        int count = Math.max(1, motorCount);
        String key = type.getKey();
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("Motor controller type is missing a key for simulation mapping");
        }
        if (key.startsWith("ctre:talonfx")) {
            return DCMotor.getFalcon500(count);
        }
        if (key.startsWith("rev:sparkmax")) {
            return DCMotor.getNEO(count);
        }
        if (key.startsWith("rev:sparkflex")) {
            return DCMotor.getNeoVortex(count);
        }

        throw new IllegalArgumentException(
                "No simulation motor mapping for controller type '" + key
                        + "'. Provide a DCMotor override in the simulation parameters or map the type here.");
    }

    public static final class ElevatorParameters {
        private double gearing = Double.NaN;
        private double carriageMassKg = Double.NaN;
        private double drumRadiusMeters = Double.NaN;
        private double minHeightMeters = 0.0;
        private double maxHeightMeters = 2.0;
        private boolean simulateGravity = true;
        private double startingHeightMeters = 0.0;
        private double unitsPerMeter = Double.NaN;
        private double nominalVoltage = 12.0;
        private double updatePeriodSeconds = 0.02;
        private DCMotor motorOverride = null;

        private ElevatorParameters() {
        }

        /**
         * Creates a new parameter builder for elevator simulations.
         *
         * @return new parameter instance
         */
        public static ElevatorParameters create() {
            return new ElevatorParameters();
        }

        /**
         * Overrides the effective gearing between the motor and elevator carriage.
         *
         * @param gearing motor rotations per carriage rotation
         * @return this parameter builder for chaining
         */
        public ElevatorParameters gearing(double gearing) {
            this.gearing = gearing;
            return this;
        }

        /**
         * Sets the elevator carriage mass used in the physics model.
         *
         * @param carriageMassKg mass in kilograms
         * @return this parameter builder for chaining
         */
        public ElevatorParameters carriageMassKg(double carriageMassKg) {
            this.carriageMassKg = carriageMassKg;
            return this;
        }

        /**
         * Sets the drum/cable radius in meters.
         *
         * @param drumRadiusMeters radius in meters
         * @return this parameter builder for chaining
         */
        public ElevatorParameters drumRadiusMeters(double drumRadiusMeters) {
            this.drumRadiusMeters = drumRadiusMeters;
            return this;
        }

        /**
         * Sets the travel constraints for the elevator carriage.
         *
         * @param minHeight minimum height in meters
         * @param maxHeight maximum height in meters
         * @return this parameter builder for chaining
         */
        public ElevatorParameters heightLimits(double minHeight, double maxHeight) {
            this.minHeightMeters = minHeight;
            this.maxHeightMeters = maxHeight;
            return this;
        }

        /**
         * Enables or disables gravity in the simulation.
         *
         * @param simulateGravity true to simulate gravity
         * @return this parameter builder for chaining
         */
        public ElevatorParameters simulateGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        /**
         * Sets the elevator carriage starting height.
         *
         * @param startingHeightMeters starting height in meters
         * @return this parameter builder for chaining
         */
        public ElevatorParameters startingHeight(double startingHeightMeters) {
            this.startingHeightMeters = startingHeightMeters;
            return this;
        }

        /**
         * Overrides the conversion from encoder units to meters.
         *
         * @param unitsPerMeter encoder units per meter of travel
         * @return this parameter builder for chaining
         */
        public ElevatorParameters unitsPerMeter(double unitsPerMeter) {
            this.unitsPerMeter = unitsPerMeter;
            return this;
        }

        /**
         * Sets the nominal battery voltage that bounds the simulated output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public ElevatorParameters nominalVoltage(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            return this;
        }

        /**
         * Substitutes a specific {@link DCMotor} model instead of deriving one from the mechanism.
         *
         * @param motor motor model to use
         * @return this parameter builder for chaining
         */
        public ElevatorParameters withMotorOverride(DCMotor motor) {
            this.motorOverride = Objects.requireNonNull(motor);
            return this;
        }

        /**
         * Sets the simulation update period for the generated model.
         *
         * @param updatePeriodSeconds loop period in seconds
         * @return this parameter builder for chaining
         */
        public ElevatorParameters updatePeriodSeconds(double updatePeriodSeconds) {
            this.updatePeriodSeconds = updatePeriodSeconds;
            return this;
        }
    }

    public static final class ArmParameters {
        private double gearing = Double.NaN;
        private double momentOfInertia = Double.NaN;
        private double armLengthMeters = Double.NaN;
        private double minAngleRadians = -Math.PI;
        private double maxAngleRadians = Math.PI;
        private boolean simulateGravity = true;
        private double startingAngleRadians = 0.0;
        private double unitsPerRadian = Double.NaN;
        private double nominalVoltage = 12.0;
        private double updatePeriodSeconds = 0.02;
        private DCMotor motorOverride = null;

        private ArmParameters() {
        }

        /**
         * Creates a new parameter builder for single-jointed arm simulations.
         *
         * @return new parameter instance
         */
        public static ArmParameters create() {
            return new ArmParameters();
        }

        /**
         * Overrides the effective motor-to-arm gearing.
         *
         * @param gearing motor rotations per arm rotation
         * @return this parameter builder for chaining
         */
        public ArmParameters gearing(double gearing) {
            this.gearing = gearing;
            return this;
        }

        /**
         * Sets the arm's moment of inertia about the pivot.
         *
         * @param momentOfInertia inertia in kg·m^2
         * @return this parameter builder for chaining
         */
        public ArmParameters momentOfInertia(double momentOfInertia) {
            this.momentOfInertia = momentOfInertia;
            return this;
        }

        /**
         * Sets the arm length from pivot to end effector.
         *
         * @param armLengthMeters length in meters
         * @return this parameter builder for chaining
         */
        public ArmParameters armLengthMeters(double armLengthMeters) {
            this.armLengthMeters = armLengthMeters;
            return this;
        }

        /**
         * Sets the allowable angular travel range.
         *
         * @param minAngleRadians minimum angle in radians
         * @param maxAngleRadians maximum angle in radians
         * @return this parameter builder for chaining
         */
        public ArmParameters angleLimits(double minAngleRadians, double maxAngleRadians) {
            this.minAngleRadians = minAngleRadians;
            this.maxAngleRadians = maxAngleRadians;
            return this;
        }

        /**
         * Enables or disables gravity torque in the simulation.
         *
         * @param simulateGravity true to include gravity
         * @return this parameter builder for chaining
         */
        public ArmParameters simulateGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        /**
         * Sets the starting arm angle for the simulation.
         *
         * @param startingAngleRadians starting angle in radians
         * @return this parameter builder for chaining
         */
        public ArmParameters startingAngle(double startingAngleRadians) {
            this.startingAngleRadians = startingAngleRadians;
            return this;
        }

        /**
         * Overrides the conversion from encoder units to radians.
         *
         * @param unitsPerRadian encoder units per radian
         * @return this parameter builder for chaining
         */
        public ArmParameters unitsPerRadian(double unitsPerRadian) {
            this.unitsPerRadian = unitsPerRadian;
            return this;
        }

        /**
         * Sets the nominal battery voltage that bounds the simulated output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public ArmParameters nominalVoltage(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            return this;
        }

        /**
         * Substitutes a specific {@link DCMotor} model instead of deriving one from the mechanism.
         *
         * @param motor motor model to use
         * @return this parameter builder for chaining
         */
        public ArmParameters withMotorOverride(DCMotor motor) {
            this.motorOverride = Objects.requireNonNull(motor);
            return this;
        }

        /**
         * Sets the simulation update period for the generated model.
         *
         * @param updatePeriodSeconds loop period in seconds
         * @return this parameter builder for chaining
         */
        public ArmParameters updatePeriodSeconds(double updatePeriodSeconds) {
            this.updatePeriodSeconds = updatePeriodSeconds;
            return this;
        }
    }

    public static final class SimpleMotorParameters {
        private double gearing = Double.NaN;
        private double momentOfInertia = Double.NaN;
        private double unitsPerRadian = Double.NaN;
        private double nominalVoltage = 12.0;
        private double updatePeriodSeconds = 0.02;
        private DCMotor motorOverride = null;

        private SimpleMotorParameters() {
        }

        /**
         * Creates a new parameter builder for roller/flywheel simulations.
         *
         * @return new parameter instance
         */
        public static SimpleMotorParameters create() {
            return new SimpleMotorParameters();
        }

        /**
         * Overrides the effective motor-to-output gearing.
         *
         * @param gearing motor rotations per output rotation
         * @return this parameter builder for chaining
         */
        public SimpleMotorParameters gearing(double gearing) {
            this.gearing = gearing;
            return this;
        }

        /**
         * Sets the inertia of the rotating system.
         *
         * @param momentOfInertia inertia in kg·m^2
         * @return this parameter builder for chaining
         */
        public SimpleMotorParameters momentOfInertia(double momentOfInertia) {
            this.momentOfInertia = momentOfInertia;
            return this;
        }

        /**
         * Overrides the conversion from encoder units to radians.
         *
         * @param unitsPerRadian encoder units per radian
         * @return this parameter builder for chaining
         */
        public SimpleMotorParameters unitsPerRadian(double unitsPerRadian) {
            this.unitsPerRadian = unitsPerRadian;
            return this;
        }

        /**
         * Sets the nominal battery voltage that bounds the simulated output.
         *
         * @param nominalVoltage voltage in volts
         * @return this parameter builder for chaining
         */
        public SimpleMotorParameters nominalVoltage(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            return this;
        }

        /**
         * Substitutes a specific {@link DCMotor} model instead of deriving one from the mechanism.
         *
         * @param motor motor model to use
         * @return this parameter builder for chaining
         */
        public SimpleMotorParameters withMotorOverride(DCMotor motor) {
            this.motorOverride = Objects.requireNonNull(motor);
            return this;
        }

        /**
         * Sets the simulation update period for the generated model.
         *
         * @param updatePeriodSeconds loop period in seconds
         * @return this parameter builder for chaining
         */
        public SimpleMotorParameters updatePeriodSeconds(double updatePeriodSeconds) {
            this.updatePeriodSeconds = updatePeriodSeconds;
            return this;
        }
    }

    private static final class ElevatorModel implements MechanismSimulationModel {

        private final ElevatorMechanism mechanism;
        private final ElevatorSim sim;
        private final double unitsPerMeter;
        private final double nominalVoltage;
        private final double minHeightMeters;
        private final double maxHeightMeters;

        ElevatorModel(ElevatorMechanism mechanism,
                      ElevatorSim sim,
                      double unitsPerMeter,
                      double nominalVoltage,
                      double minHeightMeters,
                      double maxHeightMeters) {
            this.mechanism = mechanism;
            this.sim = sim;
            this.unitsPerMeter = unitsPerMeter;
            this.nominalVoltage = Math.abs(nominalVoltage);
            this.minHeightMeters = minHeightMeters;
            this.maxHeightMeters = maxHeightMeters;
            MechanismTravelRange.registerKnownRange(
                    mechanism,
                    minHeightMeters * unitsPerMeter,
                    maxHeightMeters * unitsPerMeter);
            reset();
        }

        @Override
        public void reset() {
            double height = clamp(sim.getPositionMeters());
            MechanismSimUtil.applyEncoderState(
                    mechanism,
                    height * unitsPerMeter,
                    sim.getVelocityMetersPerSecond() * unitsPerMeter);
        }

        @Override
        public void update(double dtSeconds) {
            double output = applyMotorInversion(mechanism, mechanism.output());
            double voltage = mechanism.outputIsVoltage() ? output : output * nominalVoltage;
            voltage = MathUtil.clamp(voltage, -nominalVoltage, nominalVoltage);
            sim.setInputVoltage(voltage);
            sim.update(dtSeconds);

            double height = clamp(sim.getPositionMeters());
            MechanismSimUtil.applyEncoderState(
                    mechanism,
                    height * unitsPerMeter,
                    sim.getVelocityMetersPerSecond() * unitsPerMeter);
        }

        private double clamp(double height) {
            return MathUtil.clamp(height, minHeightMeters, maxHeightMeters);
        }
    }

    private static final class ArmModel implements MechanismSimulationModel {

        private final ArmMechanism mechanism;
        private final SingleJointedArmSim sim;
        private final double unitsPerRadian;
        private final double nominalVoltage;
        private final double minAngleRad;
        private final double maxAngleRad;

        ArmModel(ArmMechanism mechanism,
                 SingleJointedArmSim sim,
                 double unitsPerRadian,
                 double nominalVoltage,
                 double minAngleRad,
                 double maxAngleRad) {
            this.mechanism = mechanism;
            this.sim = sim;
            this.unitsPerRadian = unitsPerRadian;
            this.nominalVoltage = Math.abs(nominalVoltage);
            this.minAngleRad = minAngleRad;
            this.maxAngleRad = maxAngleRad;
            MechanismTravelRange.registerKnownRange(
                    mechanism,
                    minAngleRad * unitsPerRadian,
                    maxAngleRad * unitsPerRadian);
            reset();
        }

        @Override
        public void reset() {
            double angle = clamp(sim.getAngleRads());
            MechanismSimUtil.applyEncoderState(
                    mechanism,
                    angle * unitsPerRadian,
                    sim.getVelocityRadPerSec() * unitsPerRadian);
        }

        @Override
        public void update(double dtSeconds) {
            double output = applyMotorInversion(mechanism, mechanism.output());
            double voltage = mechanism.outputIsVoltage() ? output : output * nominalVoltage;
            voltage = MathUtil.clamp(voltage, -nominalVoltage, nominalVoltage);
            sim.setInputVoltage(voltage);
            sim.update(dtSeconds);

            double angle = clamp(sim.getAngleRads());
            MechanismSimUtil.applyEncoderState(
                    mechanism,
                    angle * unitsPerRadian,
                    sim.getVelocityRadPerSec() * unitsPerRadian);
        }

        private double clamp(double angle) {
            return MathUtil.clamp(angle, minAngleRad, maxAngleRad);
        }
    }

    private static final class SimpleMotorModel implements MechanismSimulationModel {

        private final Mechanism mechanism;
        private final FlywheelSim sim;
        private final double unitsPerRadian;
        private final double nominalVoltage;
        private double simulatedAngleRad;
        private double lastAngularVelocityRadPerSec;

        SimpleMotorModel(Mechanism mechanism,
                         FlywheelSim sim,
                         double unitsPerRadian,
                         double nominalVoltage) {
            this.mechanism = mechanism;
            this.sim = sim;
            this.unitsPerRadian = unitsPerRadian;
            this.nominalVoltage = Math.abs(nominalVoltage);
            reset();
        }

        @Override
        public void reset() {
            simulatedAngleRad = 0.0;
            lastAngularVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
            MechanismSimUtil.applyEncoderState(
                    mechanism,
                    simulatedAngleRad * unitsPerRadian,
                    lastAngularVelocityRadPerSec * unitsPerRadian);
        }

        @Override
        public void update(double dtSeconds) {
            double output = applyMotorInversion(mechanism, mechanism.output());
            double voltage = mechanism.outputIsVoltage() ? output : output * nominalVoltage;
            voltage = MathUtil.clamp(voltage, -nominalVoltage, nominalVoltage);
            double clampedDt = dtSeconds;
            if (clampedDt < 0.0) {
                clampedDt = 0.0;
            }
            double prevVelocity = lastAngularVelocityRadPerSec;
            sim.setInputVoltage(voltage);
            sim.update(clampedDt);
            double currentVelocity = sim.getAngularVelocityRadPerSec();
            simulatedAngleRad += 0.5 * (prevVelocity + currentVelocity) * clampedDt;
            lastAngularVelocityRadPerSec = currentVelocity;

            MechanismSimUtil.applyEncoderState(
                    mechanism,
                    simulatedAngleRad * unitsPerRadian,
                    currentVelocity * unitsPerRadian);
        }
    }
}
