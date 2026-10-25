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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
        double volts = 0.0;
        double velocity = Double.NaN;
        int count = 0;
        List<MotorDevice> motors = model.motors();
        for (int index = 0; index < motors.size(); index++) {
            MotorDevice motor = motors.get(index);
            MotorCommand command = command(motor, session);
            if (command == null) continue;
            if (command.kind() == SimMotorHandle.CommandKind.VELOCITY && Double.isNaN(velocity)) {
                velocity = averageVelocity(model, session);
            }
            // Percent and voltage commands are already applied to the shaft. Position and velocity targets are in
            // the controller's frame, phased with its inversion like the sensors, so compare them there and turn
            // the result onto the shaft. A follower without its own command keeps its leader's handling.
            boolean own = session.motor(motor).commandKind() != SimMotorHandle.CommandKind.NEUTRAL
                    || motor.follower() == null;
            double frame = own && motor.isInverted() ? -1.0 : 1.0;
            volts += switch (command.kind()) {
                case NEUTRAL -> 0.0;
                case PERCENT -> command.value() * 12.0;
                case VOLTAGE -> command.value();
                case POSITION -> frame * (command.value() - frame * position) * 12.0;
                case VELOCITY -> frame * (command.value() - frame * velocity) * 6.0;
            };
            count++;
        }
        return count == 0 ? 0.0 : clamp(volts / count, -12.0, 12.0);
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
            // Sensors are phased with the inversion; the plant runs on the shaft.
            sum += (motor.isInverted() ? -1.0 : 1.0) * session.motor(motor).integratedVelocityRotationsPerSecond();
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

    /** Resolves immutable declaration relationships once, rather than rebuilding sets and streams per tick. */
    private static final class Publication {
        private final List<MotorDevice> motors;
        private final List<EncoderDevice> encoders;
        private final boolean[] integratedEncoders;
        /** Sensor frame of each motor (-1 inverted) and of each integrated encoder (its motor's). */
        private final double[] motorFrames;
        private final double[] encoderFrames;
        private final List<SimLimit> limits;
        private final double gearing;

        private Publication(SimModel model) {
            motors = model.motors();
            encoders = linkedEncoders(model);
            integratedEncoders = new boolean[encoders.size()];
            Set<EncoderDevice> integrated = new LinkedHashSet<>();
            for (MotorDevice motor : motors) integrated.add(motor.encoder());
            motorFrames = new double[motors.size()];
            for (int index = 0; index < motors.size(); index++) {
                motorFrames[index] = motors.get(index).isInverted() ? -1.0 : 1.0;
            }
            encoderFrames = new double[encoders.size()];
            for (int index = 0; index < encoders.size(); index++) {
                integratedEncoders[index] = integrated.contains(encoders.get(index));
                encoderFrames[index] = 1.0;
                for (int motor = 0; motor < motors.size(); motor++) {
                    if (motors.get(motor).encoder().equals(encoders.get(index))) encoderFrames[index] = motorFrames[motor];
                }
            }
            limits = limits(model);
            gearing = gearing(model);
        }

        private void publish(SimulationSession session, double position, double velocity) {
            double motorPosition = position * gearing;
            double motorVelocity = velocity * gearing;
            for (int index = 0; index < motors.size(); index++) {
                // Like a SPARK or Talon, an inverted motor reports its built-in sensor inverted.
                session.motor(motors.get(index)).state(
                        motorFrames[index] * motorPosition, motorFrames[index] * motorVelocity);
            }
            for (int index = 0; index < encoders.size(); index++) {
                boolean integrated = integratedEncoders[index];
                SimEncoderHandle encoder = session.encoder(encoders.get(index));
                double encoderPosition = integrated ? encoderFrames[index] * motorPosition : position;
                encoder.positionRotations(encoderPosition)
                        .absolutePositionRotations(encoderPosition)
                        .velocityRotationsPerSecond(integrated ? encoderFrames[index] * motorVelocity : velocity);
            }
            for (int index = 0; index < limits.size(); index++) {
                SimLimit limit = limits.get(index);
                SimDigitalInputHandle input = session.digitalInput(limit.sensor());
                boolean active = Math.abs(position - limit.position()) <= limit.tolerance();
                input.raw(limit.sensor().isInverted() ? !active : active);
            }
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

    private abstract static class ModelAdapter {
        private final Publication publication;

        private ModelAdapter(SimModel model) {
            publication = new Publication(model);
        }

        protected final void publish(SimulationSession session, double position, double velocity) {
            publication.publish(session, position, velocity);
        }

        abstract double step(SimModel model, SimulationSession session, double seconds);
    }

    private static final class MotorAdapter extends ModelAdapter {
        private final DCMotorSim sim;

        private MotorAdapter(SimModel model) {
            super(model);
            DCMotor gearbox = gearbox(model);
            double gearing = gearing(model);
            sim = new CachedMotorSim(
                    LinearSystemId.createDCMotorSystem(gearbox, momentOfInertia(model), gearing),
                    gearbox);
        }

        @Override
        public double step(SimModel model, SimulationSession session, double seconds) {
            sim.setInput(0, commandVoltage(model, session, sim.getAngularPositionRotations()));
            sim.update(seconds);
            publish(session, sim.getAngularPositionRotations(), sim.getAngularVelocityRPM() / 60.0);
            return sim.getCurrentDrawAmps();
        }
    }

    private static final class FlywheelAdapter extends ModelAdapter {
        private final FlywheelSim sim;
        private double positionRotations;

        private FlywheelAdapter(SimModel model) {
            super(model);
            DCMotor gearbox = gearbox(model);
            double gearing = gearing(model);
            sim = new CachedFlywheelSim(
                    LinearSystemId.createFlywheelSystem(gearbox, momentOfInertia(model), gearing),
                    gearbox);
        }

        @Override
        public double step(SimModel model, SimulationSession session, double seconds) {
            sim.setInput(0, commandVoltage(model, session, positionRotations));
            sim.update(seconds);
            double velocity = sim.getAngularVelocityRPM() / 60.0;
            positionRotations += velocity * seconds;
            publish(session, positionRotations, velocity);
            return sim.getCurrentDrawAmps();
        }
    }

    private static final class ArmAdapter extends ModelAdapter {
        private final SingleJointedArmSim sim;

        private ArmAdapter(SimModel model) {
            super(model);
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
            sim.setInput(0, commandVoltage(model, session, position));
            sim.update(seconds);
            publish(session, radiansToRotations(sim.getAngleRads()), radiansToRotations(sim.getVelocityRadPerSec()));
            return sim.getCurrentDrawAmps();
        }
    }

    private static final class ElevatorAdapter extends ModelAdapter {
        private final ElevatorSim sim;

        private ElevatorAdapter(SimModel model) {
            super(model);
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
            sim.setInput(0, commandVoltage(model, session, sim.getPositionMeters()));
            sim.update(seconds);
            publish(session, sim.getPositionMeters(), sim.getVelocityMetersPerSecond());
            return sim.getCurrentDrawAmps();
        }
    }

    /**
     * Simulates an identified plant. Static friction holds the mechanism at rest until the drive exceeds it,
     * then kinetic friction, kV, gravity and kA act on the motion. Volts become mechanism torque (rotary) or
     * force (linear) through the motor constants, so the inertia equals the one the characterization derives
     * from kA. Steps are split into fixed substeps so the stick-slip transitions stay stable.
     */
    private static final class PlantAdapter extends ModelAdapter {
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
            super(model);
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
            publish(session, position, velocity);
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

    /**
     * The motor and flywheel plants are linear and noiseless. WPILib normally discretizes their unchanged
     * A/B matrices on every update. Keep the exact discretization for the last timestep and reuse private
     * state/output storage; a changed timestep always recomputes it, without quantization.
     */
    private static final class CachedLinearStep<S extends Num, I extends Num, O extends Num> {
        private final LinearSystem<S, I, O> plant;
        private final double[] nextState;
        private Matrix<S, S> discreteA;
        private Matrix<S, I> discreteB;
        private double lastSeconds = Double.NaN;

        private CachedLinearStep(LinearSystem<S, I, O> plant) {
            this.plant = plant;
            nextState = new double[plant.getA().getNumRows()];
        }

        private void update(Matrix<S, N1> state, Matrix<I, N1> input, Matrix<O, N1> output, double seconds) {
            if (seconds != lastSeconds) {
                var discretized = Discretization.discretizeAB(plant.getA(), plant.getB(), seconds);
                discreteA = discretized.getFirst();
                discreteB = discretized.getSecond();
                lastSeconds = seconds;
            }
            for (int row = 0; row < nextState.length; row++) {
                double next = 0.0;
                for (int col = 0; col < nextState.length; col++) next += discreteA.get(row, col) * state.get(col, 0);
                double driven = 0.0;
                for (int col = 0; col < input.getNumRows(); col++) driven += discreteB.get(row, col) * input.get(col, 0);
                nextState[row] = next + driven;
            }
            for (int row = 0; row < nextState.length; row++) state.set(row, 0, nextState[row]);
            for (int row = 0; row < output.getNumRows(); row++) {
                double observed = 0.0;
                for (int col = 0; col < nextState.length; col++) observed += plant.getC(row, col) * nextState[col];
                double direct = 0.0;
                for (int col = 0; col < input.getNumRows(); col++) direct += plant.getD(row, col) * input.get(col, 0);
                output.set(row, 0, observed + direct);
            }
        }
    }

    private static final class CachedMotorSim extends DCMotorSim {
        private final CachedLinearStep<N2, N1, N2> step;

        private CachedMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox) {
            super(plant, gearbox);
            step = new CachedLinearStep<>(plant);
        }

        @Override
        public void update(double seconds) {
            step.update(m_x, m_u, m_y, seconds);
        }
    }

    private static final class CachedFlywheelSim extends FlywheelSim {
        private final CachedLinearStep<N1, N1, N1> step;

        private CachedFlywheelSim(LinearSystem<N1, N1, N1> plant, DCMotor gearbox) {
            super(plant, gearbox);
            step = new CachedLinearStep<>(plant);
        }

        @Override
        public void update(double seconds) {
            step.update(m_x, m_u, m_y, seconds);
        }
    }

    private record MotorCommand(SimMotorHandle.CommandKind kind, double value) {
    }
}
