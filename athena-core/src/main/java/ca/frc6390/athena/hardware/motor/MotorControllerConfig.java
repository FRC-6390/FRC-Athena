package ca.frc6390.athena.hardware.motor;

import java.util.function.Consumer;

import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
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

    /**
     */
    public MotorControllerConfig(MotorControllerType type, int id) {
        this.type = type;
        this.id = Math.abs(id);
        this.inverted = id < 0;
    }

    /**
     */
    public MotorControllerConfig() {
    }

    public static MotorControllerConfig create(MotorControllerType type, int id) {
        return new MotorControllerConfig(type, id);
    }

    public static MotorControllerConfig create(MotorControllerType type) {
        return new MotorControllerConfig(type, 0);
    }

    /**
     * Sectioned fluent API for identity/inversion/current settings.
     */
    public MotorControllerConfig hardware(Consumer<HardwareSection> section) {
        if (section != null) {
            section.accept(new HardwareSection(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API for nested encoder configuration.
     */
    public MotorControllerConfig encoder(Consumer<EncoderSection> section) {
        if (section != null) {
            section.accept(new EncoderSection(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API for closed-loop settings.
     */
    public MotorControllerConfig control(Consumer<ControlSection> section) {
        if (section != null) {
            section.accept(new ControlSection(this));
        }
        return this;
    }

    /**
     */
    public MotorControllerConfig setCanbus(String canbus) {
        this.canbus = canbus;
        if (encoderConfig != null) {
            encoderConfig.setCanbus(canbus);
        }
        return this;
    }

    /**
     */
    public MotorControllerConfig setCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    /**
     */
    public MotorControllerConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     */
    public MotorControllerConfig setEncoderConfig(EncoderConfig encoderConfig) {
        this.encoderConfig = encoderConfig;
        if (encoderConfig != null) {
            encoderConfig.setCanbus(canbus);
        }
        return this;
    }

    /**
     */
    public MotorControllerConfig setNeutralMode(MotorNeutralMode neutralMode) {
        this.neutralMode = neutralMode;
        return this;
    }

    /**
     */
    public MotorControllerConfig setPid(PIDController pid) {
        this.pid = pid;
        return this;
    }

    /**
     */
    public MotorControllerConfig setId(int id) {
        this.id = Math.abs(id);
        this.inverted = id < 0;
        return this;
    }

    public MotorControllerType type() {
        return type;
    }

    public int id() {
        return id;
    }

    public String canbus() {
        return canbus;
    }

    public double currentLimit() {
        return currentLimit;
    }

    public boolean inverted() {
        return inverted;
    }

    public EncoderConfig encoderConfig() {
        return encoderConfig;
    }

    public MotorNeutralMode neutralMode() {
        return neutralMode;
    }

    public PIDController pid() {
        return pid;
    }

    public static final class HardwareSection {
        private final MotorControllerConfig owner;

        private HardwareSection(MotorControllerConfig owner) {
            this.owner = owner;
        }

        public HardwareSection type(MotorControllerType type) {
            owner.type = type;
            return this;
        }

        public HardwareSection id(int id) {
            owner.setId(id);
            return this;
        }

        public HardwareSection canbus(String canbus) {
            owner.setCanbus(canbus);
            return this;
        }

        public HardwareSection currentLimit(double currentLimit) {
            owner.setCurrentLimit(currentLimit);
            return this;
        }

        public HardwareSection inverted(boolean inverted) {
            owner.setInverted(inverted);
            return this;
        }

        public HardwareSection neutralMode(MotorNeutralMode neutralMode) {
            owner.setNeutralMode(neutralMode);
            return this;
        }
    }

    public static final class EncoderSection {
        private final MotorControllerConfig owner;

        private EncoderSection(MotorControllerConfig owner) {
            this.owner = owner;
        }

        public EncoderSection config(EncoderConfig encoderConfig) {
            owner.setEncoderConfig(encoderConfig);
            return this;
        }

        public EncoderSection type(EncoderType type) {
            owner.setEncoderConfig(EncoderConfig.create(type));
            return this;
        }

        public EncoderSection type(EncoderType type, int id) {
            owner.setEncoderConfig(EncoderConfig.create(type, id));
            return this;
        }

        public EncoderSection configure(Consumer<EncoderConfig> section) {
            EncoderConfig cfg = owner.encoderConfig;
            if (cfg == null) {
                cfg = new EncoderConfig().setCanbus(owner.canbus);
                owner.setEncoderConfig(cfg);
            }
            if (section != null) {
                section.accept(cfg);
            }
            return this;
        }
    }

    public static final class ControlSection {
        private final MotorControllerConfig owner;

        private ControlSection(MotorControllerConfig owner) {
            this.owner = owner;
        }

        public ControlSection pid(PIDController pid) {
            owner.setPid(pid);
            return this;
        }
    }
}
