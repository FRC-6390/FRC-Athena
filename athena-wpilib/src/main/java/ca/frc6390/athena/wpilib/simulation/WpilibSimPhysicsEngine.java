package ca.frc6390.athena.wpilib.simulation;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimLimit;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimPlant;
import ca.frc6390.athena.sim.hardware.SimDigitalInputHandle;
import ca.frc6390.athena.sim.hardware.SimEncoderHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import ca.frc6390.athena.sim.runtime.SimPhysicsEngine;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleConsumer;

/**
 * WPILib-backed physics engine for Athena simulation models.
 */
public final class WpilibSimPhysicsEngine implements SimPhysicsEngine {
    private static final double DEFAULT_MOI = 0.01;
    private static final double DEFAULT_ARM_LENGTH_METERS = 0.5;
    private static final double DEFAULT_ELEVATOR_MASS_KG = 4.0;
    private static final double DEFAULT_ELEVATOR_DRUM_RADIUS_METERS = 0.025;

    private final Map<SimModel, ModelAdapter> adapters = new IdentityHashMap<>();
    private final DoubleConsumer batteryVoltageSink;
    private double estimatedBatteryVoltage = 12.0;

    /**
     * Creates a WPILib-backed physics engine without publishing battery voltage to HAL.
     */
    public WpilibSimPhysicsEngine() {
        this(null);
    }

    /**
     * Creates a WPILib-backed physics engine.
     *
     * @param batteryVoltageSink optional sink for estimated battery voltage
     */
    public WpilibSimPhysicsEngine(DoubleConsumer batteryVoltageSink) {
        this.batteryVoltageSink = batteryVoltageSink;
    }

    @Override
    public void step(Collection<SimModel> models, SimulationSession session, double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0.0 || models == null || models.isEmpty()) {
            return;
        }
        double currentDrawAmps = 0.0;
        for (SimModel model : models) {
            if (model == null || model.motors().isEmpty()) {
                continue;
            }
            ModelAdapter adapter = adapters.computeIfAbsent(model, this::createAdapter);
            currentDrawAmps += adapter.step(model, session, seconds);
        }
        updateBatteryVoltage(currentDrawAmps);
    }

    double estimatedBatteryVoltage() {
        return estimatedBatteryVoltage;
    }

    private void updateBatteryVoltage(double currentDrawAmps) {
        estimatedBatteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(Math.max(0.0, currentDrawAmps));
        if (batteryVoltageSink != null) {
            batteryVoltageSink.accept(estimatedBatteryVoltage);
        }
    }

    private ModelAdapter createAdapter(SimModel model) {
        if (model.plant().isPresent()) {
            return new PlantAdapter(model, model.plant().get());
        }
        return switch (model.kind()) {
            case ARM -> new ArmAdapter(model);
            case ELEVATOR -> new ElevatorAdapter(model);
            case FLYWHEEL -> new FlywheelAdapter(model);
            case MOTOR -> new MotorAdapter(model);
        };
    }

    private static double commandVoltage(SimModel model, SimulationSession session, double position) {
        List<MotorCommand> commands = new ArrayList<>();
        for (MotorDevice motor : model.motors()) {
            MotorCommand command = command(motor, session);
            if (command != null) {
                commands.add(command);
            }
        }
        if (commands.isEmpty()) {
            return 0.0;
        }

        double volts = 0.0;
        for (MotorCommand command : commands) {
            volts += switch (command.kind()) {
                case NEUTRAL -> 0.0;
                case PERCENT -> command.value() * 12.0;
                case VOLTAGE -> command.value();
                case POSITION -> (command.value() - position) * 12.0;
                case VELOCITY -> (command.value() - averageVelocity(model, session)) * 6.0;
            };
        }
        return clamp(volts / commands.size(), -12.0, 12.0);
    }

    private static MotorCommand command(MotorDevice motor, SimulationSession session) {
        SimMotorHandle handle = session.motor(motor);
        if (handle.commandKind() != SimMotorHandle.CommandKind.NEUTRAL) {
            return new MotorCommand(handle.commandKind(), handle.commandValue());
        }
        if (motor.follower() != null) {
            MotorCommand leader = command(motor.follower().leader(), session);
            if (leader == null) {
                return null;
            }
            if (motor.isInverted()
                    && (leader.kind() == SimMotorHandle.CommandKind.PERCENT
                    || leader.kind() == SimMotorHandle.CommandKind.VOLTAGE
                    || leader.kind() == SimMotorHandle.CommandKind.VELOCITY)) {
                return new MotorCommand(leader.kind(), -leader.value());
            }
            return leader;
        }
        return new MotorCommand(handle.commandKind(), handle.commandValue());
    }

    private static double averageVelocity(SimModel model, SimulationSession session) {
        double sum = 0.0;
        int count = 0;
        for (MotorDevice motor : model.motors()) {
            sum += session.motor(motor).integratedVelocityRotationsPerSecond();
            count++;
        }
        return count == 0 ? 0.0 : sum / count;
    }

    private static DCMotor gearbox(SimModel model) {
        MotorDevice first = model.motors().get(0);
        int count = Math.max(1, model.motors().size());
        if (!(first.kind().motorKind() instanceof MotorKinds motorKind)) {
            throw new IllegalArgumentException("No WPILib motor model for " + first.kind().key() + ".");
        }
        for (MotorDevice motor : model.motors()) {
            if (motor.kind().motorKind() != motorKind) {
                throw new IllegalArgumentException(
                        "A simulation physics leaf cannot combine different physical motor models.");
            }
        }
        return switch (motorKind) {
            case FALCON_500 -> DCMotor.getFalcon500(count);
            case KRAKEN_X60 -> DCMotor.getKrakenX60(count);
            case KRAKEN_X44 -> DCMotor.getKrakenX44(count);
            case MINION -> DCMotor.getMinion(count);
            case NEO -> DCMotor.getNEO(count);
            case NEO_550 -> DCMotor.getNeo550(count);
            case NEO_VORTEX -> DCMotor.getNeoVortex(count);
            case CIM -> DCMotor.getCIM(count);
            case MINI_CIM -> DCMotor.getMiniCIM(count);
            case BAG -> DCMotor.getBag(count);
            case VEX_775_PRO -> DCMotor.getVex775Pro(count);
            case ANDYMARK_9015 -> DCMotor.getAndymark9015(count);
            case ANDYMARK_RS775_125 -> DCMotor.getAndymarkRs775_125(count);
            case BANEBOTS_RS550 -> DCMotor.getBanebotsRs550(count);
            case BANEBOTS_RS775 -> DCMotor.getBanebotsRs775(count);
        };
    }

    private static double gearing(SimModel model) {
        if (model.gearRatio() == null) {
            return 1.0;
        }
        return 1.0 / Math.abs(model.gearRatio().ratio());
    }

    private static double momentOfInertia(SimModel model) {
        return Math.max(1.0e-6, model.momentOfInertia().map(inertia -> inertia.in(edu.wpi.first.units.Units.KilogramSquareMeters)).orElse(DEFAULT_MOI));
    }

    private static Range range(SimModel model, Range fallback) {
        return model.range() == null ? fallback : model.range();
    }

    private static List<EncoderDevice> linkedEncoders(SimModel model) {
        Set<EncoderDevice> encoders = new LinkedHashSet<>(model.encoders());
        for (MotorDevice motor : model.motors()) {
            encoders.add(motor.encoder());
        }
        return List.copyOf(encoders);
    }

    private static List<SimLimit> limits(SimModel model) {
        return model.limits();
    }

    private static void publish(
            SimModel model,
            SimulationSession session,
            double mechanismPositionRotations,
            double mechanismVelocityRotationsPerSecond) {
        double motorPositionRotations = mechanismPositionRotations * gearing(model);
        double motorVelocityRotationsPerSecond = mechanismVelocityRotationsPerSecond * gearing(model);
        for (MotorDevice motor : model.motors()) {
            session.motor(motor).state(motorPositionRotations, motorVelocityRotationsPerSecond);
        }
        for (EncoderDevice encoderDevice : linkedEncoders(model)) {
            boolean motorIntegrated = model.motors().stream()
                    .anyMatch(motor -> motor.encoder().equals(encoderDevice));
            SimEncoderHandle encoder = session.encoder(encoderDevice);
            double position = motorIntegrated ? motorPositionRotations : mechanismPositionRotations;
            double velocity = motorIntegrated
                    ? motorVelocityRotationsPerSecond
                    : mechanismVelocityRotationsPerSecond;
            encoder.positionRotations(position)
                    .absolutePositionRotations(position)
                    .velocityRotationsPerSecond(velocity);
        }
        for (SimLimit limit : limits(model)) {
            SimDigitalInputHandle input = session.digitalInput(limit.sensor());
            boolean active = Math.abs(mechanismPositionRotations - limit.position()) <= limit.tolerance();
            input.raw(limit.sensor().isInverted() ? !active : active);
        }
    }

    private static double rotationsToRadians(double rotations) {
        return Units.rotationsToRadians(rotations);
    }

    private static double radiansToRotations(double radians) {
        return Units.radiansToRotations(radians);
    }

    private static double clamp(double value, double minimum, double maximum) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        return Math.max(minimum, Math.min(maximum, value));
    }

    private interface ModelAdapter {
        double step(SimModel model, SimulationSession session, double seconds);
    }

    private static final class MotorAdapter implements ModelAdapter {
        private final DCMotorSim sim;

        private MotorAdapter(SimModel model) {
            DCMotor gearbox = gearbox(model);
            double gearing = gearing(model);
            sim = new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(gearbox, momentOfInertia(model), gearing),
                    gearbox);
        }

        @Override
        public double step(SimModel model, SimulationSession session, double seconds) {
            sim.setInput(commandVoltage(model, session, sim.getAngularPositionRotations()));
            sim.update(seconds);
            publish(model, session, sim.getAngularPositionRotations(), sim.getAngularVelocityRPM() / 60.0);
            return sim.getCurrentDrawAmps();
        }
    }

    private static final class FlywheelAdapter implements ModelAdapter {
        private final FlywheelSim sim;
        private double positionRotations;

        private FlywheelAdapter(SimModel model) {
            DCMotor gearbox = gearbox(model);
            double gearing = gearing(model);
            sim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(gearbox, momentOfInertia(model), gearing),
                    gearbox);
        }

        @Override
        public double step(SimModel model, SimulationSession session, double seconds) {
            sim.setInput(commandVoltage(model, session, positionRotations));
            sim.update(seconds);
            double velocity = sim.getAngularVelocityRPM() / 60.0;
            positionRotations += velocity * seconds;
            publish(model, session, positionRotations, velocity);
            return sim.getCurrentDrawAmps();
        }
    }

    private static final class ArmAdapter implements ModelAdapter {
        private final SingleJointedArmSim sim;

        private ArmAdapter(SimModel model) {
            Range range = range(model, Range.of(edu.wpi.first.units.Units.Rotations.of(-0.5), edu.wpi.first.units.Units.Rotations.of(0.5)));
            DCMotor gearbox = gearbox(model);
            sim = new SingleJointedArmSim(
                    gearbox,
                    gearing(model),
                    momentOfInertia(model),
                    model.length().map(length -> length.in(edu.wpi.first.units.Units.Meters)).orElse(DEFAULT_ARM_LENGTH_METERS),
                    rotationsToRadians(range.raw().minimumInUnit()),
                    rotationsToRadians(range.raw().maximumInUnit()),
                    model.simulatesGravity(),
                    rotationsToRadians(range.clamp(0.0)));
        }

        @Override
        public double step(SimModel model, SimulationSession session, double seconds) {
            double position = radiansToRotations(sim.getAngleRads());
            sim.setInput(commandVoltage(model, session, position));
            sim.update(seconds);
            publish(model, session, radiansToRotations(sim.getAngleRads()), radiansToRotations(sim.getVelocityRadPerSec()));
            return sim.getCurrentDrawAmps();
        }
    }

    private static final class ElevatorAdapter implements ModelAdapter {
        private final ElevatorSim sim;

        private ElevatorAdapter(SimModel model) {
            Range range = range(model, Range.of(edu.wpi.first.units.Units.Meters.of(0.0), edu.wpi.first.units.Units.Meters.of(1.0)));
            sim = new ElevatorSim(
                    gearbox(model),
                    gearing(model),
                    DEFAULT_ELEVATOR_MASS_KG,
                    DEFAULT_ELEVATOR_DRUM_RADIUS_METERS,
                    range.raw().minimumInUnit(),
                    range.raw().maximumInUnit(),
                    model.simulatesGravity(),
                    range.clamp(0.0));
        }

        @Override
        public double step(SimModel model, SimulationSession session, double seconds) {
            sim.setInput(commandVoltage(model, session, sim.getPositionMeters()));
            sim.update(seconds);
            publish(model, session, sim.getPositionMeters(), sim.getVelocityMetersPerSecond());
            return sim.getCurrentDrawAmps();
        }
    }

    /**
     * Simulates an identified plant. Static friction holds the mechanism at rest until the drive exceeds it,
     * then kinetic friction, kV, gravity and kA act on the motion. Volts become mechanism torque (rotary) or
     * force (linear) through the motor constants, so the inertia equals the one the characterization derives
     * from kA. Steps are split into fixed substeps so the stick-slip transitions stay stable.
     */
    private static final class PlantAdapter implements ModelAdapter {
        private static final double MAX_SUBSTEP_SECONDS = 0.001;
        private static final double MAX_VOLTAGE = 12.0;

        private final SimPlant plant;
        private final Range range;
        private final double outputPerVolt;
        private final double inertia;
        private final double staticVolts;
        private final double kineticVolts;
        private final double resistanceTotalOhms;
        private double position;
        private double velocity;

        private PlantAdapter(SimModel model, SimPlant plant) {
            this.plant = plant;
            double torquePerVolt = plant.outputPerVolt();
            if (plant.rotary()) {
                range = range(model, Range.of(edu.wpi.first.units.Units.Rotations.of(-0.5), edu.wpi.first.units.Units.Rotations.of(0.5)));
                outputPerVolt = torquePerVolt;
                // J = kA * torque per volt / (2 pi), from kA = 2 pi J / (torque per volt).
                inertia = plant.kAVoltsPerUnitPerSecondSquared() * torquePerVolt / (2.0 * Math.PI);
            } else {
                range = range(model, Range.of(edu.wpi.first.units.Units.Meters.of(0.0), edu.wpi.first.units.Units.Meters.of(1.0)));
                outputPerVolt = torquePerVolt / DEFAULT_ELEVATOR_DRUM_RADIUS_METERS;
                inertia = plant.kAVoltsPerUnitPerSecondSquared() * outputPerVolt;
            }
            // Static friction can never be below the kinetic value it breaks into.
            staticVolts = Math.max(plant.staticFrictionVolts(), plant.kineticFrictionVolts());
            kineticVolts = plant.kineticFrictionVolts();
            resistanceTotalOhms = plant.resistanceOhms() / plant.motorCount();
            position = range.clamp(0.0);
        }

        @Override
        public double step(SimModel model, SimulationSession session, double seconds) {
            double volts = clamp(commandVoltage(model, session, position), -MAX_VOLTAGE, MAX_VOLTAGE);
            double remaining = seconds;
            while (remaining > 0.0) {
                double substep = Math.min(MAX_SUBSTEP_SECONDS, remaining);
                remaining -= substep;
                advance(volts, substep);
            }
            double currentDraw = Math.abs(volts - plant.kVVoltsPerUnitPerSecond() * velocity) / resistanceTotalOhms;
            publish(model, session, position, velocity);
            return currentDraw;
        }

        private void advance(double volts, double seconds) {
            double drive = volts - gravityVolts();
            double previous = velocity;
            if (previous == 0.0) {
                if (Math.abs(drive) <= staticVolts) {
                    return;
                }
                // Breakaway: kinetic friction opposes the motion this substep starts.
                velocity = accelerationFor(drive - Math.signum(drive) * kineticVolts) * seconds;
            } else {
                double friction = Math.signum(previous) * kineticVolts;
                velocity = previous + accelerationFor(drive - friction - plant.kVVoltsPerUnitPerSecond() * previous)
                        * seconds;
                if (velocity * previous <= 0.0) {
                    // Crossed zero: the mechanism may stick here; the next substep decides.
                    velocity = 0.0;
                }
            }
            position += velocity * seconds;
            if (position <= range.raw().minimumInUnit() && velocity <= 0.0) {
                position = range.raw().minimumInUnit();
                velocity = 0.0;
            } else if (position >= range.raw().maximumInUnit() && velocity >= 0.0) {
                position = range.raw().maximumInUnit();
                velocity = 0.0;
            }
        }

        /** Acceleration (rotations or meters per second squared) produced by a net voltage. */
        private double accelerationFor(double netVolts) {
            double output = outputPerVolt * netVolts;
            return plant.rotary() ? output / inertia / (2.0 * Math.PI) : output / inertia;
        }

        /** Gravity voltage at the current position: kG cos(2 pi p + offset) for rotary, constant for linear. */
        private double gravityVolts() {
            return plant.rotary()
                    ? plant.gravityVolts() * Math.cos(2.0 * Math.PI * position + plant.gravityOffsetRadians())
                    : plant.gravityVolts();
        }
    }

    private record MotorCommand(SimMotorHandle.CommandKind kind, double value) {
    }
}
