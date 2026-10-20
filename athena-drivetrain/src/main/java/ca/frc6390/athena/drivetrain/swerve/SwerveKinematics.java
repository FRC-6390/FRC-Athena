package ca.frc6390.athena.drivetrain.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import ca.frc6390.athena.runtime.control.VelocityFrame;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.concurrent.atomic.AtomicReference;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Production swerve kinematics shared by drive actions, odometry, and optional simulation.
 *
 * <p>The layout is selected explicitly when the robot is declared. Simulation never guesses a
 * drivetrain type from motor topology.</p>
 */
public final class SwerveKinematics implements SimModel.Source {
    private static final double TWO_PI = Math.PI * 2.0;

    private final List<Module> modules;
    private final double maxSpeedMetersPerSecond;
    private final AtomicReference<RobotVelocity> requestedSimulationVelocity = new AtomicReference<>();
    private SimModel simulationModel;
    // Simulation step only: per-module drive distance encoders and the drive motors they were built from.
    private final EncoderDevice[] simulationDistanceEncoders;
    private final MotorDevice[] simulationDriveMotors;

    public SwerveKinematics(LinearVelocity maxSpeed, Module... modules) {
        this(maxSpeed, modules == null ? List.of() : Arrays.asList(modules));
    }

    public SwerveKinematics(LinearVelocity maxSpeed, List<Module> modules) {
        double maxSpeedMetersPerSecond = Objects.requireNonNull(maxSpeed, "maxSpeed").in(MetersPerSecond);
        if (!Double.isFinite(maxSpeedMetersPerSecond) || maxSpeedMetersPerSecond <= 0.0) {
            throw new IllegalArgumentException("Max speed must be positive.");
        }
        this.modules = modules == null ? List.of() : List.copyOf(modules);
        if (this.modules.size() < 2) {
            throw new IllegalArgumentException("At least two positioned modules are required.");
        }
        this.modules.forEach(Objects::requireNonNull);
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        simulationDistanceEncoders = new EncoderDevice[this.modules.size()];
        simulationDriveMotors = new MotorDevice[this.modules.size()];
    }

    /** Creates the common four-module rectangular layout. */
    public static SwerveKinematics rectangular(
            Distance wheelBase,
            Distance trackWidth,
            LinearVelocity maxSpeed,
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight) {
        double wheelBaseMeters = Objects.requireNonNull(wheelBase, "wheelBase").in(Meters);
        double trackWidthMeters = Objects.requireNonNull(trackWidth, "trackWidth").in(Meters);
        requirePositive(wheelBaseMeters, "Wheelbase");
        requirePositive(trackWidthMeters, "Track width");
        Distance x = Meters.of(wheelBaseMeters / 2.0);
        Distance y = Meters.of(trackWidthMeters / 2.0);
        Distance negativeX = Meters.of(-wheelBaseMeters / 2.0);
        Distance negativeY = Meters.of(-trackWidthMeters / 2.0);
        return new SwerveKinematics(maxSpeed, List.of(
                new Module(frontLeft, x, y),
                new Module(frontRight, x, negativeY),
                new Module(backLeft, negativeX, y),
                new Module(backRight, negativeX, negativeY)));
    }

    /** Converts a requested robot-relative velocity into one composed module action. */
    public Action drive(RobotVelocity velocity) {
        return drive(velocity, 0.02);
    }

    private Action drive(RobotVelocity velocity, double dtSeconds) {
        RobotVelocity safeVelocity = discretize(
                velocity == null ? RobotVelocity.zero() : velocity,
                dtSeconds);
        requestedSimulationVelocity.set(safeVelocity);
        List<SwerveModuleTarget> targets = targets(safeVelocity);
        Action[] actions = new Action[modules.size()];
        for (int index = 0; index < modules.size(); index++) {
            actions[index] = modules.get(index).module().target(targets.get(index));
        }
        return Actions.parallel(actions);
    }

    /** Continuously recomputes module targets from a velocity supplier. */
    public Action drive(Supplier<RobotVelocity> velocity) {
        Supplier<RobotVelocity> safeVelocity = Objects.requireNonNull(velocity, "velocity");
        return Actions.compute(context -> drive(safeVelocity.get(), context.dtSeconds()), modules.stream()
                .map(Module::module)
                .toList());
    }

    /** Drives one pooled request after normalizing every channel into robot coordinates. */
    public Action drive(RobotVelocityPool velocities, Supplier<Rotation2d> heading) {
        RobotVelocityPool safeVelocities = Objects.requireNonNull(velocities, "velocities");
        Supplier<Rotation2d> safeHeading = Objects.requireNonNull(heading, "heading");
        return drive(() -> safeVelocities.robotRelative(safeHeading.get()));
    }

    /** Converts robot-relative velocity into ordered module targets. */
    public List<SwerveModuleTarget> targets(RobotVelocity velocity) {
        RobotVelocity safe = velocity == null ? RobotVelocity.zero() : velocity;
        if (safe.frame() != VelocityFrame.ROBOT) {
            throw new IllegalArgumentException("Swerve module targets require a robot-relative velocity.");
        }
        SwerveModuleTarget[] targets = new SwerveModuleTarget[modules.size()];
        double greatestSpeed = 0.0;
        for (int index = 0; index < targets.length; index++) {
            Module module = modules.get(index);
            double wheelX = safe.raw().xMetersPerSecond() - safe.raw().angularRadiansPerSecond() * module.yMeters();
            double wheelY = safe.raw().yMetersPerSecond() + safe.raw().angularRadiansPerSecond() * module.xMeters();
            double speed = Math.hypot(wheelX, wheelY);
            greatestSpeed = Math.max(greatestSpeed, speed);
            double angleRotations = speed <= 1.0e-12 ? 0.0 : Math.atan2(wheelY, wheelX) / TWO_PI;
            targets[index] = new SwerveModuleTarget(speed, angleRotations);
        }
        if (greatestSpeed > maxSpeedMetersPerSecond) {
            double scale = maxSpeedMetersPerSecond / greatestSpeed;
            for (int index = 0; index < targets.length; index++) {
                SwerveModuleTarget target = targets[index];
                targets[index] = new SwerveModuleTarget(target.speedMetersPerSecond() * scale, target.angleRotations());
            }
        }
        return List.of(targets);
    }

    /** Converts a finite-cycle request into module targets without translational skew. */
    public List<SwerveModuleTarget> targets(RobotVelocity velocity, double dtSeconds) {
        return targets(discretize(velocity == null ? RobotVelocity.zero() : velocity, dtSeconds));
    }

    /** Solves robot-relative velocity from ordered measured module states. */
    public RobotVelocity velocity(List<SwerveModuleTarget> states) {
        Objects.requireNonNull(states, "states");
        if (states.size() != modules.size()) {
            throw new IllegalArgumentException("Expected " + modules.size() + " module states, got " + states.size());
        }

        // Normal equations as one 3x4 row-major augmented matrix [A^T A | A^T b].
        double[] system = new double[12];
        for (int index = 0; index < modules.size(); index++) {
            Module module = modules.get(index);
            SwerveModuleTarget state = Objects.requireNonNull(states.get(index), "state");
            double angle = state.angleRotations() * TWO_PI;
            double wheelX = state.speedMetersPerSecond() * Math.cos(angle);
            double wheelY = state.speedMetersPerSecond() * Math.sin(angle);
            accumulate(system, 1.0, 0.0, -module.yMeters(), wheelX);
            accumulate(system, 0.0, 1.0, module.xMeters(), wheelY);
        }
        if (!solve3x3(system)) {
            return RobotVelocity.Raw.of(0.0, 0.0, 0.0, VelocityFrame.ROBOT);
        }
        return RobotVelocity.Raw.of(system[3], system[7], system[11], VelocityFrame.ROBOT);
    }

    public List<Module> modules() {
        return modules;
    }

    /** Wheel speed that the kinematics saturates to when a requested velocity is too fast. */
    public LinearVelocity maxSpeed() {
        return MetersPerSecond.of(maxSpeedMetersPerSecond);
    }

    /**
     * Whole-drivetrain drive characterization for this layout: all modules straight, the same voltage on every
     * drive motor, average drive velocity logged, kS/kV/kA fitted with no gravity term.
     */
    public SwerveDriveCharacterization characterization() {
        return new SwerveDriveCharacterization(modules);
    }

    /** Creates the path follower for this exact module layout. */
    public SwervePathFollower follow(
            FollowerBackend backend,
            Supplier<Pose2d> pose,
            Function<Pose2d, Action> resetPose,
            PidGains translationGains,
            PidGains headingGains) {
        return new SwervePathFollower(
                this, backend, pose, resetPose, translationGains, headingGains);
    }

    /** Creates odometry backed by this layout's measured wheel travel and an IMU heading. */
    public SwerveOdometry odometry(ImuSource imu) {
        return new SwerveOdometry(this, imu);
    }

    /**
     * Supplies the automatically discovered simulation for this real kinematic layout.
     */
    @Override
    public synchronized SimModel simulationModel() {
        if (simulationModel == null) {
            simulationModel = createSimulationModel();
        }
        return simulationModel;
    }

    private SimModel createSimulationModel() {
        SimModel.Builder model = SimModel.builder();
        for (Module positioned : modules) {
            SwerveModule module = positioned.module();
            model.include(SimModel.motor(module.drive.get())
                    .encoder(module.drive.get().encoder())
                    .gearRatio(GearRatio.reduction(module.model().driveReduction(), 1.0)));
            model.include(SimModel.motor(module.steer.get())
                    .encoder(module.angle.get())
                    .gearRatio(GearRatio.reduction(module.model().steerReduction(), 1.0)));
            model.motor(module.drive.get()).motor(module.steer.get()).encoder(module.angle.get());
        }
        model.runtime(SwerveSimulationRuntime::new);
        return model.build();
    }

    private void stepPose(SimModel.Context context, double seconds, double[] drivePositions) {
        RobotVelocity robot = requestedSimulationVelocity.getAndSet(null);
        if (robot != null) {
            List<SwerveModuleTarget> states = targets(robot);
            synchronizeModuleSensors(context, states, drivePositions, seconds);
            robot = velocity(states);
        } else {
            List<SwerveModuleTarget> states = new ArrayList<>(modules.size());
            for (Module positioned : modules) {
                SwerveModule module = positioned.module();
                states.add(new SwerveModuleTarget(moduleSpeed(module, context), moduleAngle(module, context)));
            }
            robot = velocity(states);
        }
        PoseSnapshot pose = context.pose();
        Pose2d current = new Pose2d(pose.xMeters(), pose.yMeters(),
                edu.wpi.first.math.geometry.Rotation2d.fromRadians(pose.headingRadians()));
        Pose2d next = current.exp(new Twist2d(
                robot.raw().xMetersPerSecond() * seconds,
                robot.raw().yMetersPerSecond() * seconds,
                robot.raw().angularRadiansPerSecond() * seconds));
        context.advancePose(
                Units.MetersPerSecond.of((next.getX() - current.getX()) / seconds),
                Units.MetersPerSecond.of((next.getY() - current.getY()) / seconds),
                Units.RadiansPerSecond.of(robot.raw().angularRadiansPerSecond()),
                Units.Seconds.of(seconds));
    }

    private void synchronizeModuleSensors(
            SimModel.Context context,
            List<SwerveModuleTarget> states,
            double[] drivePositions,
            double seconds) {
        for (int index = 0; index < modules.size(); index++) {
            SwerveModule module = modules.get(index).module();
            SwerveModuleTarget state = states.get(index);
            EncoderDevice driveDistance = simulationDistanceEncoder(index, module);
            double rawDriveVelocity = driveDistance.rotationsFromPosition(Meters.of(state.speedMetersPerSecond()))
                    - driveDistance.rotationsFromPosition(Meters.of(0.0));
            drivePositions[index] += rawDriveVelocity * seconds;
            context.motorState(module.drive.get(), Units.Rotations.of(drivePositions[index]),
                    Units.RotationsPerSecond.of(rawDriveVelocity));
            context.encoderState(
                    driveDistance,
                    Units.Rotations.of(drivePositions[index]),
                    Units.Rotations.of(drivePositions[index]),
                    Units.RotationsPerSecond.of(rawDriveVelocity));

            EncoderDevice angle = module.angle.get();
            double rawAngle = angle.rotationsFromPosition(Rotations.of(state.angleRotations()));
            double previousAngle = context.encoderPosition(angle).in(Units.Rotations);
            context.encoderState(angle, Units.Rotations.of(rawAngle), Units.Rotations.of(rawAngle),
                    Units.RotationsPerSecond.of((rawAngle - previousAngle) / seconds));
            double steerPosition = rawAngle * module.model().steerReduction();
            context.motorState(
                    module.steer.get(),
                    Units.Rotations.of(steerPosition),
                    Units.RotationsPerSecond.of((rawAngle - previousAngle) * module.model().steerReduction() / seconds));
        }
    }

    /**
     * The drive distance encoder of one module for the simulation step, rebuilt only when the module's drive motor
     * changes. The encoder is an immutable declaration derived from that motor and the module's fixed model.
     */
    private EncoderDevice simulationDistanceEncoder(int index, SwerveModule module) {
        MotorDevice drive = module.drive.get();
        if (simulationDriveMotors[index] != drive) {
            simulationDistanceEncoders[index] = distanceEncoder(module);
            simulationDriveMotors[index] = drive;
        }
        return simulationDistanceEncoders[index];
    }

    private static EncoderDevice distanceEncoder(SwerveModule module) {
        return module.drive.get().encoder()
                .reduction(module.model().driveReduction())
                .measures(Units.Meters)
                .travelPerRevolution(module.model().wheelCircumference());
    }

    private final class SwerveSimulationRuntime implements SimModel.Runtime {
        private final SimModel.Context context;
        private final double[] drivePositions = new double[modules.size()];

        private SwerveSimulationRuntime(SimModel.Context context) {
            this.context = context;
            for (int index = 0; index < modules.size(); index++) {
                drivePositions[index] = context.encoderPosition(distanceEncoder(modules.get(index).module())).in(Units.Rotations);
            }
        }

        @Override
        public void step(double seconds) {
            stepPose(context, seconds, drivePositions);
        }
    }

    private static RobotVelocity discretize(RobotVelocity velocity, double dtSeconds) {
        if (velocity.frame() != VelocityFrame.ROBOT) {
            throw new IllegalArgumentException("Swerve discretization requires a robot-relative velocity.");
        }
        double dt = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.02;
        ChassisSpeeds corrected = ChassisSpeeds.discretize(
                velocity.raw().xMetersPerSecond(),
                velocity.raw().yMetersPerSecond(),
                velocity.raw().angularRadiansPerSecond(),
                dt);
        return RobotVelocity.Raw.of(
                corrected.vxMetersPerSecond,
                corrected.vyMetersPerSecond,
                corrected.omegaRadiansPerSecond,
                VelocityFrame.ROBOT);
    }

    private double moduleSpeed(SwerveModule module, SimModel.Context context) {
        SimModel.MotorCommand command = context.command(module.drive.get());
        return switch (command.mode()) {
            case VELOCITY -> command.value();
            case PERCENT -> command.value() * maxSpeedMetersPerSecond;
            case VOLTAGE -> command.value() / 12.0 * maxSpeedMetersPerSecond;
            case POSITION, NEUTRAL -> context.motorVelocity(module.drive.get()).in(Units.RotationsPerSecond)
                    * Math.PI
                    * module.model().wheelDiameter().in(Meters);
        };
    }

    private static double moduleAngle(SwerveModule module, SimModel.Context context) {
        SimModel.MotorCommand command = context.command(module.steer.get());
        if (command.mode() == SimModel.CommandMode.POSITION) {
            return command.value();
        }
        return context.encoderAbsolutePosition(module.angle.get()).in(Units.Rotations);
    }

    /** Adds one observation row {@code (a, b, c)} to the 3x4 row-major augmented normal equations. */
    private static void accumulate(double[] system, double a, double b, double c, double observed) {
        accumulateRow(system, 0, a, a, b, c, observed);
        accumulateRow(system, 4, b, a, b, c, observed);
        accumulateRow(system, 8, c, a, b, c, observed);
    }

    private static void accumulateRow(
            double[] system, int offset, double weight, double a, double b, double c, double observed) {
        system[offset + 3] += weight * observed;
        system[offset] += weight * a;
        system[offset + 1] += weight * b;
        system[offset + 2] += weight * c;
    }

    /**
     * Solves a 3x4 row-major augmented system in place by Gauss-Jordan elimination with partial pivoting, leaving
     * the solution in the last column. Returns false when the system is singular.
     */
    private static boolean solve3x3(double[] augmented) {
        for (int column = 0; column < 3; column++) {
            int pivot = column;
            for (int row = column + 1; row < 3; row++) {
                if (Math.abs(augmented[row * 4 + column]) > Math.abs(augmented[pivot * 4 + column])) {
                    pivot = row;
                }
            }
            if (Math.abs(augmented[pivot * 4 + column]) < 1.0e-12) {
                return false;
            }
            if (pivot != column) {
                for (int item = 0; item < 4; item++) {
                    double swap = augmented[column * 4 + item];
                    augmented[column * 4 + item] = augmented[pivot * 4 + item];
                    augmented[pivot * 4 + item] = swap;
                }
            }
            double divisor = augmented[column * 4 + column];
            for (int item = column; item < 4; item++) {
                augmented[column * 4 + item] /= divisor;
            }
            for (int row = 0; row < 3; row++) {
                if (row == column) {
                    continue;
                }
                double factor = augmented[row * 4 + column];
                for (int item = column; item < 4; item++) {
                    augmented[row * 4 + item] -= factor * augmented[column * 4 + item];
                }
            }
        }
        return true;
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be positive.");
        }
    }

    /** A module and its real position relative to robot center: {@code x} forward, {@code y} left. */
    public record Module(SwerveModule module, Distance x, Distance y) {
        public Module {
            Objects.requireNonNull(module, "module");
            Objects.requireNonNull(x, "x");
            Objects.requireNonNull(y, "y");
            if (!module.drive.filled() || !module.steer.filled() || !module.angle.filled()) {
                throw new IllegalArgumentException("Swerve module slots must be filled before creating kinematics.");
            }
            if (!Double.isFinite(x.in(Meters)) || !Double.isFinite(y.in(Meters))) {
                throw new IllegalArgumentException("Module position must be finite.");
            }
        }

        /** Forward offset in meters. Per-cycle math reads primitives through this, so nothing allocates. */
        double xMeters() {
            return x.in(Meters);
        }

        /** Left offset in meters. */
        double yMeters() {
            return y.in(Meters);
        }
    }
}
