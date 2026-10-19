package ca.frc6390.athena.sim.runtime;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimLimit;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.sim.hardware.SimDigitalInputHandle;
import ca.frc6390.athena.sim.hardware.SimEncoderHandle;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Default physics for registered simulation models.
 *
 * <p>Each model is one rotary axis. Its motors are {@link DcMotorModel linear DC motors} with the physical
 * constants of their motor kind, coupled to the axis through the declared gear ratio (motor rotations per
 * mechanism rotation, {@code N}). The axis obeys {@code J alpha = N * sum(tau_motor)}, where {@code J} is the
 * declared moment of inertia, and the motors are driven by:</p>
 * <ul>
 *   <li>{@code PERCENT} / {@code VOLTAGE}: the applied voltage (clamped to the 12 V nominal);</li>
 *   <li>{@code POSITION} / {@code VELOCITY}: an onboard proportional loop producing that voltage;</li>
 *   <li>zero output: a motor whose neutral mode is {@code BRAKE} shorts its windings (electrical braking
 *       through the back EMF); {@code COAST} leaves them open so only friction slows it.</li>
 * </ul>
 * <p>A declared current limit clamps the motor current. Integration uses fixed 1 ms substeps, reduced when
 * the axis is stiff, so the result is deterministic and stable at the robot's 20 ms period.</p>
 */
final class SimModelRunner implements SimPhysicsEngine {
    private static final double MAX_SUBSTEP_SECONDS = 0.001;
    /** Substep is at most this fraction of the stiffest motor-side damping time constant. */
    private static final double STIFFNESS_FRACTION = 0.2;
    private static final int MAX_SUBSTEPS = 4000;
    private static final double POSITION_VOLTS_PER_ROTATION = 12.0;
    private static final double TWO_PI = 2.0 * Math.PI;

    private final Map<SimModel, ModelState> states = new IdentityHashMap<>();

    @Override
    public void step(Collection<SimModel> simulations, SimulationSession session, double dtSeconds) {
        if (dtSeconds <= 0.0 || simulations.isEmpty()) {
            return;
        }
        for (SimModel simulation : simulations) {
            step(simulation, session, dtSeconds);
        }
    }

    private void step(SimModel simulation, SimulationSession session, double dtSeconds) {
        List<EncoderDevice> linkedEncoders = linkedEncoders(simulation);
        if (simulation.motors().isEmpty() || linkedEncoders.isEmpty()) {
            return;
        }

        double motorPerMechanism = motorRotationsPerMechanismRotation(simulation);
        double inertia = momentOfInertia(simulation);
        List<MotorDrive> drives = new ArrayList<>();
        for (MotorDevice motor : simulation.motors()) {
            drives.add(drive(motor, session));
        }

        ModelState state = states.computeIfAbsent(
                simulation, ignored -> initialState(simulation, linkedEncoders, session, motorPerMechanism));
        integrate(state, drives, motorPerMechanism, inertia, simulation.range(), dtSeconds);

        for (MotorDevice motor : simulation.motors()) {
            session.motor(motor).state(
                    state.position * motorPerMechanism,
                    state.velocity * motorPerMechanism);
        }
        for (EncoderDevice encoderDevice : linkedEncoders) {
            // Motor encoders count motor rotations; declared mechanism encoders count mechanism rotations.
            double scale = motorSide(encoderDevice, simulation) ? motorPerMechanism : 1.0;
            session.encoder(encoderDevice)
                    .positionRotations(state.position * scale)
                    .absolutePositionRotations(state.position * scale)
                    .velocityRotationsPerSecond(state.velocity * scale);
        }

        for (SimLimit limit : simulation.limits()) {
            SimDigitalInputHandle input = session.digitalInput(limit.sensor());
            boolean active = Math.abs(state.position - limit.position()) <= limit.tolerance();
            input.raw(limit.sensor().isInverted() ? !active : active);
        }
    }

    /** Advances the axis by {@code seconds} in stable substeps. */
    private static void integrate(
            ModelState state,
            List<MotorDrive> drives,
            double motorPerMechanism,
            double inertia,
            Range range,
            double seconds) {
        double motorSideDamping = 0.0;
        for (MotorDrive drive : drives) {
            motorSideDamping += drive.model().motorSideDampingNmSPerRad();
        }
        // Motor-side damping seen at the axis scales with N^2 / J.
        double rate = motorPerMechanism * motorPerMechanism * motorSideDamping / inertia;
        double substep = rate > 0.0
                ? Math.min(MAX_SUBSTEP_SECONDS, STIFFNESS_FRACTION / rate)
                : MAX_SUBSTEP_SECONDS;
        int count = (int) Math.min(MAX_SUBSTEPS, Math.max(1, Math.ceil(seconds / substep)));
        double dt = seconds / count;

        for (int i = 0; i < count; i++) {
            double motorSpeed = motorPerMechanism * TWO_PI * state.velocity;
            double motorRotations = motorPerMechanism * state.position;
            double motorRps = motorPerMechanism * state.velocity;
            double torque = 0.0;
            for (MotorDrive drive : drives) {
                torque += drive.torque(motorSpeed, motorRotations, motorRps);
            }
            double accelerationRadPerSecondSquared = motorPerMechanism * torque / inertia;
            state.velocity += accelerationRadPerSecondSquared / TWO_PI * dt;
            state.position += state.velocity * dt;
            if (range != null) {
                double clamped = range.clamp(state.position);
                if (clamped != state.position) {
                    state.position = clamped;
                    state.velocity = 0.0;
                }
            }
        }
    }

    private static ModelState initialState(
            SimModel simulation,
            List<EncoderDevice> linkedEncoders,
            SimulationSession session,
            double motorPerMechanism) {
        for (EncoderDevice encoderDevice : linkedEncoders) {
            SimEncoderHandle encoder = session.encoder(encoderDevice);
            double scale = motorSide(encoderDevice, simulation) ? 1.0 / motorPerMechanism : 1.0;
            return new ModelState(
                    encoder.positionRotations() * scale,
                    encoder.velocityRotationsPerSecond() * scale);
        }
        return new ModelState(0.0, 0.0);
    }

    /** One motor's electrical drive state for the current control cycle. */
    private record MotorDrive(
            DcMotorModel model,
            Mode mode,
            double volts,
            double currentLimitAmps,
            MotorCommand closedLoop) {

        enum Mode {
            /** Voltage applied to the terminals. */
            DRIVE,
            /** Windings shorted (BRAKE at zero output). */
            SHORT,
            /** Windings open (COAST at zero output). */
            OPEN
        }

        /** Shaft torque of this motor at the given motor-side state. */
        double torque(double motorSpeedRadPerSecond, double motorRotations, double motorRps) {
            double amps = switch (mode) {
                case DRIVE -> model.current(drivenVolts(motorRotations, motorRps), motorSpeedRadPerSecond);
                case SHORT -> model.current(0.0, motorSpeedRadPerSecond);
                case OPEN -> 0.0;
            };
            if (currentLimitAmps > 0.0) {
                amps = Math.max(-currentLimitAmps, Math.min(currentLimitAmps, amps));
            }
            return model.torque(amps, motorSpeedRadPerSecond);
        }

        private double drivenVolts(double motorRotations, double motorRps) {
            if (closedLoop == null) {
                return volts;
            }
            double nominal = DcMotorModel.NOMINAL_VOLTAGE;
            double output = switch (closedLoop.kind()) {
                case POSITION -> (closedLoop.value() - motorRotations) * POSITION_VOLTS_PER_ROTATION;
                case VELOCITY -> (closedLoop.value() - motorRps) * nominal
                        / (model.freeSpeedRadPerSecond() / TWO_PI);
                default -> volts;
            };
            return Math.max(-nominal, Math.min(nominal, output));
        }
    }

    private static MotorDrive drive(MotorDevice motor, SimulationSession session) {
        DcMotorModel model = DcMotorModel.of(motor.kind().motorKind());
        SimMotorHandle runtime = session.motor(motor);
        MotorCommand command = command(motor, session);
        int limit = positiveMin(runtime.runtimeConfiguration().statorCurrentLimitAmps(),
                runtime.runtimeConfiguration().supplyCurrentLimitAmps());
        boolean brake = runtime.runtimeConfiguration().neutralMode() == MotorNeutralMode.BRAKE;
        if (command == null || isZeroOutput(command)) {
            return new MotorDrive(model, brake ? MotorDrive.Mode.SHORT : MotorDrive.Mode.OPEN, 0.0, limit, null);
        }
        return switch (command.kind()) {
            case PERCENT -> new MotorDrive(model, MotorDrive.Mode.DRIVE,
                    clamp(command.value(), -1.0, 1.0) * DcMotorModel.NOMINAL_VOLTAGE, limit, null);
            case VOLTAGE -> new MotorDrive(model, MotorDrive.Mode.DRIVE,
                    clamp(command.value(), -DcMotorModel.NOMINAL_VOLTAGE, DcMotorModel.NOMINAL_VOLTAGE),
                    limit, null);
            case POSITION, VELOCITY -> new MotorDrive(model, MotorDrive.Mode.DRIVE, 0.0, limit, command);
            case NEUTRAL -> new MotorDrive(model, brake ? MotorDrive.Mode.SHORT : MotorDrive.Mode.OPEN, 0.0, limit, null);
        };
    }

    private static MotorCommand command(MotorDevice motor, SimulationSession session) {
        Objects.requireNonNull(motor, "motor");
        SimMotorHandle runtime = session.motor(motor);
        if (runtime != null && runtime.commandKind() != SimMotorHandle.CommandKind.NEUTRAL) {
            return new MotorCommand(runtime.commandKind(), runtime.commandValue());
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
        if (runtime != null) {
            return new MotorCommand(runtime.commandKind(), runtime.commandValue());
        }
        return null;
    }

    private static boolean isZeroOutput(MotorCommand command) {
        return switch (command.kind()) {
            case NEUTRAL -> true;
            case PERCENT, VOLTAGE -> command.value() == 0.0;
            case POSITION, VELOCITY -> false;
        };
    }

    private static int positiveMin(int first, int second) {
        if (first <= 0) {
            return Math.max(0, second);
        }
        return second <= 0 ? first : Math.min(first, second);
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    /**
     * Moment of inertia of the axis in kg m^2. Declared values win; otherwise a default per model kind
     * approximates a typical FRC axis (a flywheel or a motor-driven shaft, an arm, a 25 mm drum elevator).
     */
    static double momentOfInertia(SimModel simulation) {
        double declared = simulation.momentOfInertia()
                .map(inertia -> inertia.in(edu.wpi.first.units.Units.KilogramSquareMeters))
                .orElse(defaultMomentOfInertia(simulation.kind()));
        return Math.max(1.0e-6, declared);
    }

    private static double defaultMomentOfInertia(SimModel.BuiltIn kind) {
        return switch (kind) {
            case MOTOR -> 0.01;
            case FLYWHEEL -> 0.01;
            case ARM -> 0.05;
            case ELEVATOR -> 0.0025;
        };
    }

    /** Motor rotations per mechanism rotation: 1 when no gear ratio is declared. */
    static double motorRotationsPerMechanismRotation(SimModel simulation) {
        return simulation.gearRatio() == null ? 1.0 : 1.0 / Math.abs(simulation.gearRatio().ratio());
    }

    /** Whether an encoder is the integrated encoder of one of the model's motors (counts motor rotations). */
    private static boolean motorSide(EncoderDevice encoder, SimModel simulation) {
        for (MotorDevice motor : simulation.motors()) {
            if (encoder.source().equals(motor.encoder().source())) {
                return true;
            }
        }
        return false;
    }

    private static List<EncoderDevice> linkedEncoders(SimModel simulation) {
        Set<EncoderDevice> encoders = new LinkedHashSet<>(simulation.encoders());
        for (MotorDevice motor : simulation.motors()) {
            encoders.add(motor.encoder());
        }
        return List.copyOf(encoders);
    }

    private record MotorCommand(SimMotorHandle.CommandKind kind, double value) {
    }

    private static final class ModelState {
        private double position;
        private double velocity;

        private ModelState(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }
}
