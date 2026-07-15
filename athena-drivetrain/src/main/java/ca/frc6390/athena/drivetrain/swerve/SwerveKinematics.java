package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.concurrent.atomic.AtomicReference;
import edu.wpi.first.math.geometry.Pose2d;

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

    public SwerveKinematics(double maxSpeedMetersPerSecond, Module... modules) {
        this(maxSpeedMetersPerSecond, modules == null ? List.of() : Arrays.asList(modules));
    }

    public SwerveKinematics(double maxSpeedMetersPerSecond, List<Module> modules) {
        if (!Double.isFinite(maxSpeedMetersPerSecond) || maxSpeedMetersPerSecond <= 0.0) {
            throw new IllegalArgumentException("Max speed must be positive.");
        }
        this.modules = modules == null ? List.of() : List.copyOf(modules);
        if (this.modules.size() < 2) {
            throw new IllegalArgumentException("At least two positioned modules are required.");
        }
        this.modules.forEach(Objects::requireNonNull);
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
    }

    /** Creates the common four-module rectangular layout. */
    public static SwerveKinematics rectangular(
            double wheelBaseMeters,
            double trackWidthMeters,
            double maxSpeedMetersPerSecond,
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight) {
        requirePositive(wheelBaseMeters, "Wheelbase");
        requirePositive(trackWidthMeters, "Track width");
        double x = wheelBaseMeters / 2.0;
        double y = trackWidthMeters / 2.0;
        return new SwerveKinematics(maxSpeedMetersPerSecond, List.of(
                new Module(frontLeft, x, y),
                new Module(frontRight, x, -y),
                new Module(backLeft, -x, y),
                new Module(backRight, -x, -y)));
    }

    /** Converts a requested robot-relative velocity into one composed module action. */
    public Action drive(RobotVelocity velocity) {
        RobotVelocity safeVelocity = velocity == null ? RobotVelocity.zero() : velocity;
        requestedSimulationVelocity.set(safeVelocity);
        List<SwerveModuleTarget> targets = targets(safeVelocity);
        Action[] actions = new Action[modules.size()];
        for (int index = 0; index < modules.size(); index++) {
            actions[index] = modules.get(index).module().target(targets.get(index));
        }
        return Actions.parallel(actions);
    }

    /** Converts robot-relative velocity into ordered module targets. */
    public List<SwerveModuleTarget> targets(RobotVelocity velocity) {
        RobotVelocity safe = velocity == null ? RobotVelocity.zero() : velocity;
        List<SwerveModuleTarget> targets = new ArrayList<>(modules.size());
        double greatestSpeed = 0.0;
        for (Module module : modules) {
            double wheelX = safe.xMetersPerSecond() - safe.angularRadiansPerSecond() * module.yMeters();
            double wheelY = safe.yMetersPerSecond() + safe.angularRadiansPerSecond() * module.xMeters();
            double speed = Math.hypot(wheelX, wheelY);
            greatestSpeed = Math.max(greatestSpeed, speed);
            double angleRotations = speed <= 1.0e-12 ? 0.0 : Math.atan2(wheelY, wheelX) / TWO_PI;
            targets.add(new SwerveModuleTarget(speed, angleRotations));
        }
        if (greatestSpeed <= maxSpeedMetersPerSecond) {
            return List.copyOf(targets);
        }
        double scale = maxSpeedMetersPerSecond / greatestSpeed;
        return targets.stream()
                .map(target -> new SwerveModuleTarget(target.speedMetersPerSecond() * scale, target.angleRotations()))
                .toList();
    }

    /** Solves robot-relative velocity from ordered measured module states. */
    public RobotVelocity velocity(List<SwerveModuleTarget> states) {
        Objects.requireNonNull(states, "states");
        if (states.size() != modules.size()) {
            throw new IllegalArgumentException("Expected " + modules.size() + " module states, got " + states.size());
        }

        double[][] normal = new double[3][3];
        double[] rhs = new double[3];
        for (int index = 0; index < modules.size(); index++) {
            Module module = modules.get(index);
            SwerveModuleTarget state = Objects.requireNonNull(states.get(index), "state");
            double angle = state.angleRotations() * TWO_PI;
            double wheelX = state.speedMetersPerSecond() * Math.cos(angle);
            double wheelY = state.speedMetersPerSecond() * Math.sin(angle);
            accumulate(normal, rhs, 1.0, 0.0, -module.yMeters(), wheelX);
            accumulate(normal, rhs, 0.0, 1.0, module.xMeters(), wheelY);
        }
        double[] solution = solve3x3(normal, rhs);
        return new RobotVelocity(solution[0], solution[1], solution[2]);
    }

    public List<Module> modules() {
        return modules;
    }

    public double maxSpeedMetersPerSecond() {
        return maxSpeedMetersPerSecond;
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
        model.runtime(context -> seconds -> stepPose(context, seconds));
        return model.build();
    }

    private void stepPose(SimModel.Context context, double seconds) {
        RobotVelocity robot = requestedSimulationVelocity.getAndSet(null);
        if (robot == null) {
            List<SwerveModuleTarget> states = new ArrayList<>(modules.size());
            for (Module positioned : modules) {
                SwerveModule module = positioned.module();
                states.add(new SwerveModuleTarget(moduleSpeed(module, context), moduleAngle(module, context)));
            }
            robot = velocity(states);
        }
        PoseSnapshot pose = context.pose();
        double cos = Math.cos(pose.headingRadians());
        double sin = Math.sin(pose.headingRadians());
        double fieldX = robot.xMetersPerSecond() * cos - robot.yMetersPerSecond() * sin;
        double fieldY = robot.xMetersPerSecond() * sin + robot.yMetersPerSecond() * cos;
        context.advancePose(fieldX, fieldY, robot.angularRadiansPerSecond(), seconds);
    }

    private double moduleSpeed(SwerveModule module, SimModel.Context context) {
        SimModel.MotorCommand command = context.command(module.drive.get());
        return switch (command.mode()) {
            case VELOCITY -> command.value();
            case PERCENT -> command.value() * maxSpeedMetersPerSecond;
            case VOLTAGE -> command.value() / 12.0 * maxSpeedMetersPerSecond;
            case POSITION, NEUTRAL -> context.motorVelocity(module.drive.get())
                    * Math.PI
                    * module.model().wheelDiameterMeters();
        };
    }

    private static double moduleAngle(SwerveModule module, SimModel.Context context) {
        SimModel.MotorCommand command = context.command(module.steer.get());
        if (command.mode() == SimModel.CommandMode.POSITION) {
            return command.value();
        }
        return context.encoderAbsolutePosition(module.angle.get());
    }

    private static void accumulate(
            double[][] normal,
            double[] rhs,
            double a,
            double b,
            double c,
            double observed) {
        double[] row = {a, b, c};
        for (int i = 0; i < 3; i++) {
            rhs[i] += row[i] * observed;
            for (int j = 0; j < 3; j++) {
                normal[i][j] += row[i] * row[j];
            }
        }
    }

    private static double[] solve3x3(double[][] matrix, double[] values) {
        double[][] augmented = new double[3][4];
        for (int row = 0; row < 3; row++) {
            System.arraycopy(matrix[row], 0, augmented[row], 0, 3);
            augmented[row][3] = values[row];
        }
        for (int column = 0; column < 3; column++) {
            int pivot = column;
            for (int row = column + 1; row < 3; row++) {
                if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) {
                    pivot = row;
                }
            }
            if (Math.abs(augmented[pivot][column]) < 1.0e-12) {
                return new double[3];
            }
            double[] swap = augmented[column];
            augmented[column] = augmented[pivot];
            augmented[pivot] = swap;
            double divisor = augmented[column][column];
            for (int item = column; item < 4; item++) {
                augmented[column][item] /= divisor;
            }
            for (int row = 0; row < 3; row++) {
                if (row == column) {
                    continue;
                }
                double factor = augmented[row][column];
                for (int item = column; item < 4; item++) {
                    augmented[row][item] -= factor * augmented[column][item];
                }
            }
        }
        return new double[] {augmented[0][3], augmented[1][3], augmented[2][3]};
    }

    private static void requirePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be positive.");
        }
    }

    /** A module and its real position relative to robot center. */
    public record Module(SwerveModule module, double xMeters, double yMeters) {
        public Module {
            Objects.requireNonNull(module, "module");
            if (!module.drive.filled() || !module.steer.filled() || !module.angle.filled()) {
                throw new IllegalArgumentException("Swerve module slots must be filled before creating kinematics.");
            }
            if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters)) {
                throw new IllegalArgumentException("Module position must be finite.");
            }
        }
    }
}
