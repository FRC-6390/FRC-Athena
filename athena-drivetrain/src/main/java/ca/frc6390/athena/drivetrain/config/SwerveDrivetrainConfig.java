package ca.frc6390.athena.drivetrain.config;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import ca.frc6390.athena.drivetrain.spec.SwerveDrivetrainSpec;
import ca.frc6390.athena.drivetrain.spec.SwerveModuleSpec;
import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.drivetrain.spec.WheelBase;

/**
 * Student-facing swerve drivetrain declaration.
 */
public final class SwerveDrivetrainConfig {
    private final String name;
    private final List<SwerveModuleConfig> modules = new ArrayList<>();
    private TrackWidth trackWidth = TrackWidth.meters(0.6);
    private WheelBase wheelBase = WheelBase.meters(0.6);
    private boolean driveInverted;
    private boolean steerInverted;
    private boolean encoderInverted;

    SwerveDrivetrainConfig(String name) {
        this.name = name;
    }

    /**
     * Adds a named swerve module.
     *
     * @param name module name
     * @param configure module configuration
     * @return this config
     */
    public SwerveDrivetrainConfig module(String name, Consumer<SwerveModuleConfig> configure) {
        SwerveModuleConfig module = new SwerveModuleConfig(name);
        if (configure != null) {
            configure.accept(module);
        }
        modules.add(module);
        return this;
    }

    /**
     * Sets track width.
     *
     * @param trackWidth left-right wheel spacing
     * @return this config
     */
    public SwerveDrivetrainConfig trackWidth(TrackWidth trackWidth) {
        this.trackWidth = trackWidth;
        return this;
    }

    /**
     * Sets wheelbase.
     *
     * @param wheelBase front-back wheel spacing
     * @return this config
     */
    public SwerveDrivetrainConfig wheelBase(WheelBase wheelBase) {
        this.wheelBase = wheelBase;
        return this;
    }

    /**
     * Sets the drivetrain-level drive inversion default.
     *
     * @param inverted true to invert drive motors by default
     * @return this config
     */
    public SwerveDrivetrainConfig driveInverted(boolean inverted) {
        driveInverted = inverted;
        return this;
    }

    /**
     * Sets the drivetrain-level steer inversion default.
     *
     * @param inverted true to invert steer motors by default
     * @return this config
     */
    public SwerveDrivetrainConfig steerInverted(boolean inverted) {
        steerInverted = inverted;
        return this;
    }

    /**
     * Sets the drivetrain-level encoder inversion default.
     *
     * @param inverted true to invert module encoders by default
     * @return this config
     */
    public SwerveDrivetrainConfig encoderInverted(boolean inverted) {
        encoderInverted = inverted;
        return this;
    }

    /**
     * Lowers this declaration to an immutable spec.
     *
     * @return drivetrain spec
     */
    public SwerveDrivetrainSpec toSpec() {
        List<SwerveModuleSpec> specs = modules.stream()
                .map(module -> module.toSpec(name, driveInverted, steerInverted, encoderInverted))
                .toList();
        return new SwerveDrivetrainSpec(name, specs, trackWidth, wheelBase);
    }
}
