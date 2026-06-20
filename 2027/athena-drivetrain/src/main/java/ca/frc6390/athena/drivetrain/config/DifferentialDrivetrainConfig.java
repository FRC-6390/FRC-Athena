package ca.frc6390.athena.drivetrain.config;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import ca.frc6390.athena.drivetrain.spec.DifferentialDrivetrainSpec;
import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.hardware.config.MotorConfig;
import ca.frc6390.athena.hardware.spec.MotorSpec;

/**
 * Student-facing differential drivetrain declaration.
 */
public final class DifferentialDrivetrainConfig {
    private final String name;
    private final List<NamedMotor> left = new ArrayList<>();
    private final List<NamedMotor> right = new ArrayList<>();
    private TrackWidth trackWidth = TrackWidth.meters(0.6);

    DifferentialDrivetrainConfig(String name) {
        this.name = name;
    }

    /**
     * Adds a left-side motor.
     *
     * @param name motor name
     * @param configure motor configuration
     * @return this config
     */
    public DifferentialDrivetrainConfig leftMotor(String name, Consumer<MotorConfig> configure) {
        left.add(configureMotor(name, configure));
        return this;
    }

    /**
     * Adds a right-side motor.
     *
     * @param name motor name
     * @param configure motor configuration
     * @return this config
     */
    public DifferentialDrivetrainConfig rightMotor(String name, Consumer<MotorConfig> configure) {
        right.add(configureMotor(name, configure));
        return this;
    }

    /**
     * Sets track width.
     *
     * @param trackWidth track width
     * @return this config
     */
    public DifferentialDrivetrainConfig trackWidth(TrackWidth trackWidth) {
        this.trackWidth = trackWidth;
        return this;
    }

    /**
     * Lowers this declaration to an immutable spec.
     *
     * @return drivetrain spec
     */
    public DifferentialDrivetrainSpec toSpec() {
        return new DifferentialDrivetrainSpec(
                name,
                lower("left", left),
                lower("right", right),
                trackWidth);
    }

    private NamedMotor configureMotor(String name, Consumer<MotorConfig> configure) {
        MotorConfig config = MotorConfig.create();
        if (configure != null) {
            configure.accept(config);
        }
        return new NamedMotor(name, config);
    }

    private List<MotorSpec> lower(String side, List<NamedMotor> motors) {
        return motors.stream()
                .map(motor -> motor.config().toSpec(name + "." + side, motor.name()))
                .toList();
    }

    private record NamedMotor(String name, MotorConfig config) {
    }
}
