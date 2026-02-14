package ca.frc6390.athena.hardware.motor;

import java.util.function.Consumer;

import ca.frc6390.athena.hardware.config.DeviceIdentityConfig;
import ca.frc6390.athena.hardware.config.DeviceIdentitySection;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import edu.wpi.first.math.controller.PIDController;

/**
 * Vendor-agnostic motor controller configuration.
 */
public class MotorControllerConfig extends DeviceIdentityConfig<MotorControllerType> {
    private double currentLimit = 40;
    private EncoderConfig encoderConfig;
    private MotorNeutralMode neutralMode = MotorNeutralMode.Coast;
    private PIDController pid;

    /**
     */
    public MotorControllerConfig(MotorControllerType type, int id) {
        super(type, id);
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

    public HardwareSection hardware() {
        return new HardwareSection(this);
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

    public EncoderSection encoder() {
        return new EncoderSection(this);
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

    public ControlSection control() {
        return new ControlSection(this);
    }

    public double currentLimit() {
        return currentLimit;
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

    public static final class HardwareSection extends DeviceIdentitySection<HardwareSection, MotorControllerType> {
        private final MotorControllerConfig owner;

        private HardwareSection(MotorControllerConfig owner) {
            super(owner::applyType, owner::applyId, owner::applyCanbus, owner::applyInverted);
            this.owner = owner;
        }

        public HardwareSection currentLimit(double currentLimit) {
            owner.applyCurrentLimit(currentLimit);
            return this;
        }

        public HardwareSection neutralMode(MotorNeutralMode neutralMode) {
            owner.applyNeutralMode(neutralMode);
            return this;
        }

        @Override
        protected HardwareSection self() {
            return this;
        }
    }

    public static final class EncoderSection {
        private final MotorControllerConfig owner;

        private EncoderSection(MotorControllerConfig owner) {
            this.owner = owner;
        }

        public EncoderSection config(EncoderConfig encoderConfig) {
            owner.applyEncoderConfig(encoderConfig);
            return this;
        }

        public EncoderSection type(EncoderType type) {
            owner.applyEncoderConfig(EncoderConfig.create(type));
            return this;
        }

        public EncoderSection type(EncoderType type, int id) {
            owner.applyEncoderConfig(EncoderConfig.create(type, id));
            return this;
        }

        public EncoderSection configure(Consumer<EncoderConfig> section) {
            EncoderConfig cfg = owner.encoderConfig;
            if (cfg == null) {
                cfg = new EncoderConfig();
                cfg.hardware().canbus(owner.canbus());
                owner.applyEncoderConfig(cfg);
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
            owner.applyPid(pid);
            return this;
        }
    }

    @Override
    protected int normalizeId(int id) {
        return Math.abs(id);
    }

    @Override
    protected Boolean inferInvertedFromId(int id) {
        return id < 0;
    }

    @Override
    protected void applyCanbus(String canbus) {
        super.applyCanbus(canbus);
        if (encoderConfig != null) {
            encoderConfig.hardware().canbus(canbus);
        }
    }

    private void applyCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
    }

    private void applyEncoderConfig(EncoderConfig encoderConfig) {
        this.encoderConfig = encoderConfig;
        if (encoderConfig != null) {
            encoderConfig.hardware().canbus(canbus());
        }
    }

    private void applyNeutralMode(MotorNeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }

    private void applyPid(PIDController pid) {
        this.pid = pid;
    }
}
