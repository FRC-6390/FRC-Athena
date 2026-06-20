package ca.frc6390.athena.drivetrain.config;

import java.util.function.Consumer;

import ca.frc6390.athena.drivetrain.spec.SwerveModuleControlSpec;
import ca.frc6390.athena.drivetrain.spec.SwerveModuleSpec;
import ca.frc6390.athena.hardware.config.MotorConfig;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;

/**
 * Student-facing swerve module declaration.
 */
public final class SwerveModuleConfig {
    private final String name;
    private MotorConfig driveMotor;
    private MotorConfig steerMotor;
    private EncoderConfig steerEncoder;
    private double xMeters;
    private double yMeters;
    private Boolean driveInverted;
    private Boolean steerInverted;
    private Boolean encoderInverted;
    private double steerP;
    private double steerI;
    private double steerD;
    private double driveKs;
    private double driveKv;
    private double driveKa;

    SwerveModuleConfig(String name) {
        this.name = name;
    }

    /**
     * Declares the drive motor.
     *
     * @param configure motor configuration
     * @return this config
     */
    public SwerveModuleConfig driveMotor(Consumer<MotorConfig> configure) {
        driveMotor = MotorConfig.create();
        if (configure != null) {
            configure.accept(driveMotor);
        }
        return this;
    }

    /**
     * Declares the steer motor.
     *
     * @param configure motor configuration
     * @return this config
     */
    public SwerveModuleConfig steerMotor(Consumer<MotorConfig> configure) {
        steerMotor = MotorConfig.create();
        if (configure != null) {
            configure.accept(steerMotor);
        }
        return this;
    }

    /**
     * Declares the absolute steer encoder.
     *
     * @param configure encoder configuration
     * @return this config
     */
    public SwerveModuleConfig steerEncoder(Consumer<EncoderConfig> configure) {
        steerEncoder = EncoderConfig.create();
        if (configure != null) {
            configure.accept(steerEncoder);
        }
        return this;
    }

    /**
     * Sets module location relative to robot center.
     *
     * @param xMeters forward-positive location
     * @param yMeters left-positive location
     * @return this config
     */
    public SwerveModuleConfig location(double xMeters, double yMeters) {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        return this;
    }

    /**
     * Overrides drivetrain-level drive inversion.
     *
     * @param inverted true to invert this drive motor
     * @return this config
     */
    public SwerveModuleConfig driveInverted(boolean inverted) {
        driveInverted = inverted;
        return this;
    }

    /**
     * Overrides drivetrain-level steer inversion.
     *
     * @param inverted true to invert this steer motor
     * @return this config
     */
    public SwerveModuleConfig steerInverted(boolean inverted) {
        steerInverted = inverted;
        return this;
    }

    /**
     * Overrides drivetrain-level encoder inversion.
     *
     * @param inverted true to invert this encoder
     * @return this config
     */
    public SwerveModuleConfig encoderInverted(boolean inverted) {
        encoderInverted = inverted;
        return this;
    }

    /**
     * Sets steer PID gains.
     *
     * @param p proportional gain
     * @param i integral gain
     * @param d derivative gain
     * @return this config
     */
    public SwerveModuleConfig steerPid(double p, double i, double d) {
        steerP = p;
        steerI = i;
        steerD = d;
        return this;
    }

    /**
     * Sets drive feedforward gains.
     *
     * @param staticGain static gain
     * @param velocity velocity gain
     * @param acceleration acceleration gain
     * @return this config
     */
    public SwerveModuleConfig driveFeedforward(double staticGain, double velocity, double acceleration) {
        driveKs = staticGain;
        driveKv = velocity;
        driveKa = acceleration;
        return this;
    }

    SwerveModuleSpec toSpec(
            String drivetrainName,
            boolean driveInvertedDefault,
            boolean steerInvertedDefault,
            boolean encoderInvertedDefault) {
        String ownerPath = drivetrainName + "." + name;
        ca.frc6390.athena.hardware.spec.MotorSpec driveMotorSpec =
                driveMotor == null ? null : driveMotor.toSpec(ownerPath, "drive");
        ca.frc6390.athena.hardware.spec.MotorSpec steerMotorSpec =
                steerMotor == null ? null : steerMotor.toSpec(ownerPath, "steer");
        ca.frc6390.athena.hardware.encoder.EncoderSpec steerEncoderSpec =
                steerEncoder == null ? null : steerEncoder.toSpec(ownerPath, "encoder");
        return new SwerveModuleSpec(
                name,
                driveMotorSpec,
                steerMotorSpec,
                steerEncoderSpec,
                xMeters,
                yMeters,
                driveInverted == null ? driveInvertedDefault : driveInverted,
                steerInverted == null ? steerInvertedDefault : steerInverted,
                encoderInverted == null ? encoderInvertedDefault : encoderInverted,
                new SwerveModuleControlSpec(steerP, steerI, steerD, driveKs, driveKv, driveKa));
    }
}
