package ca.frc6390.athena.wpilib.drivetrain;

import ca.frc6390.athena.drivetrain.spec.SwerveDrivetrainSpec;
import ca.frc6390.athena.drivetrain.spec.SwerveModuleSpec;
import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Applies Athena robot speed output to WPILib swerve module states.
 */
public final class WpilibSwerveDriveAdapter {
    private final RobotSpeeds speeds;
    private final SwerveDrivetrainSpec spec;
    private final SwerveDriveKinematics kinematics;
    private final List<ModuleOutput> outputs;
    private final double maxWheelMetersPerSecond;

    /**
     * Creates a swerve adapter.
     *
     * @param speeds Athena speed blender
     * @param spec swerve drivetrain spec
     * @param outputs module output bindings, in the same order as spec modules
     * @param maxWheelMetersPerSecond max wheel speed for desaturation
     */
    public WpilibSwerveDriveAdapter(
            RobotSpeeds speeds,
            SwerveDrivetrainSpec spec,
            List<ModuleOutput> outputs,
            double maxWheelMetersPerSecond) {
        this.speeds = Objects.requireNonNull(speeds, "speeds");
        this.spec = Objects.requireNonNull(spec, "spec");
        this.outputs = List.copyOf(outputs == null ? List.of() : outputs);
        if (this.outputs.size() != spec.modules().size()) {
            throw new IllegalArgumentException("Swerve output count must match module count.");
        }
        this.kinematics = new SwerveDriveKinematics(moduleTranslations(spec));
        this.maxWheelMetersPerSecond = finitePositive(maxWheelMetersPerSecond, 1.0);
    }

    /**
     * Creates a builder for module outputs keyed by module order.
     *
     * @param speeds Athena speed blender
     * @param spec swerve drivetrain spec
     * @param maxWheelMetersPerSecond max wheel speed for desaturation
     * @return output builder
     */
    public static Builder builder(
            RobotSpeeds speeds,
            SwerveDrivetrainSpec spec,
            double maxWheelMetersPerSecond) {
        return new Builder(speeds, spec, maxWheelMetersPerSecond);
    }

    /**
     * Applies the current robot-relative output.
     */
    public void periodic() {
        apply(speeds.calculate());
    }

    /**
     * Applies the current output after converting field-relative sources.
     *
     * @param headingRadians robot heading in radians
     */
    public void periodic(double headingRadians) {
        apply(speeds.calculate(headingRadians));
    }

    /**
     * Stops all module outputs.
     */
    public void stop() {
        for (ModuleOutput output : outputs) {
            output.stop();
        }
    }

    private void apply(RobotVelocity velocity) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(
                velocity.xMetersPerSecond(),
                velocity.yMetersPerSecond(),
                velocity.angularRadiansPerSecond()));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxWheelMetersPerSecond);
        for (int index = 0; index < states.length; index++) {
            SwerveModuleSpec module = spec.modules().get(index);
            SwerveModuleState state = states[index];
            double speed = module.driveInverted() ? -state.speedMetersPerSecond : state.speedMetersPerSecond;
            double angleRadians = module.steerInverted() ? -state.angle.getRadians() : state.angle.getRadians();
            outputs.get(index).apply(module, new SwerveModuleState(speed, edu.wpi.first.math.geometry.Rotation2d.fromRadians(angleRadians)));
        }
    }

    private static Translation2d[] moduleTranslations(SwerveDrivetrainSpec spec) {
        return spec.modules().stream()
                .map(module -> new Translation2d(module.xMeters(), module.yMeters()))
                .toArray(Translation2d[]::new);
    }

    private static double finitePositive(double value, double fallback) {
        return Double.isFinite(value) && value > 0.0 ? value : fallback;
    }

    /**
     * Receives one calculated WPILib swerve module state.
     */
    @FunctionalInterface
    public interface ModuleOutput {
        /**
         * Applies one module state.
         *
         * @param module module declaration
         * @param state WPILib module state
         */
        void apply(SwerveModuleSpec module, SwerveModuleState state);

        /**
         * Stops this module output.
         */
        default void stop() {
            apply(null, new SwerveModuleState());
        }
    }

    /**
     * Builder that collects module outputs in spec order.
     */
    public static final class Builder {
        private final RobotSpeeds speeds;
        private final SwerveDrivetrainSpec spec;
        private final double maxWheelMetersPerSecond;
        private final List<ModuleOutput> outputs = new ArrayList<>();

        private Builder(RobotSpeeds speeds, SwerveDrivetrainSpec spec, double maxWheelMetersPerSecond) {
            this.speeds = Objects.requireNonNull(speeds, "speeds");
            this.spec = Objects.requireNonNull(spec, "spec");
            this.maxWheelMetersPerSecond = maxWheelMetersPerSecond;
        }

        /**
         * Adds the next module output in spec order.
         *
         * @param output module output
         * @return this builder
         */
        public Builder module(ModuleOutput output) {
            outputs.add(Objects.requireNonNull(output, "output"));
            return this;
        }

        /**
         * Builds the adapter.
         *
         * @return swerve drive adapter
         */
        public WpilibSwerveDriveAdapter build() {
            return new WpilibSwerveDriveAdapter(speeds, spec, outputs, maxWheelMetersPerSecond);
        }
    }
}
