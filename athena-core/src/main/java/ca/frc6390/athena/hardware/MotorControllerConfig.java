package ca.frc6390.athena.hardware;

import edu.wpi.first.math.controller.PIDController;

/**
 * Vendor-agnostic motor controller configuration.
 */
public class MotorControllerConfig {
    public MotorControllerType type;
    public int id;
    public String canbus = "rio";
    public double currentLimit = 40;
    public boolean inverted = false;
    public EncoderConfig encoderConfig;
    public MotorNeutralMode neutralMode = MotorNeutralMode.Coast;
    public PIDController pid;

    public MotorControllerConfig(MotorControllerType type, int id) {
        this.type = type;
        this.id = Math.abs(id);
        this.inverted = id < 0;
    }

    public MotorControllerConfig setCanbus(String canbus) {
        this.canbus = canbus;
        if (encoderConfig != null) {
            encoderConfig.setCanbus(canbus);
        }
        return this;
    }

    public MotorControllerConfig setCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public MotorControllerConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MotorControllerConfig setEncoderConfig(EncoderConfig encoderConfig) {
        this.encoderConfig = encoderConfig;
        if (encoderConfig != null) {
            encoderConfig.setCanbus(canbus);
        }
        return this;
    }

    public MotorControllerConfig setNeutralMode(MotorNeutralMode neutralMode) {
        this.neutralMode = neutralMode;
        return this;
    }

    public MotorControllerConfig setPid(PIDController pid) {
        this.pid = pid;
        return this;
    }
}
