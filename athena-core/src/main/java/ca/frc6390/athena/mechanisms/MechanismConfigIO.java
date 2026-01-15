package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderRegistry;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.hardware.motor.MotorRegistry;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class MechanismConfigIO {
    private static final ObjectMapper MAPPER = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

    private MechanismConfigIO() {
    }

    public static MechanismConfigSnapshot load(Path path) {
        try {
            return MAPPER.readValue(path.toFile(), MechanismConfigSnapshot.class);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to load mechanism config from " + path, e);
        }
    }

    public static void save(Path path, MechanismConfigSnapshot snapshot) {
        try {
            MAPPER.writeValue(path.toFile(), snapshot);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to save mechanism config to " + path, e);
        }
    }

    public static MechanismConfigSnapshot snapshot(MechanismConfig<?> config) {
        if (config == null) {
            return null;
        }
        List<MotorControllerConfigSnapshot> motors = new ArrayList<>();
        for (MotorControllerConfig motor : config.motors) {
            motors.add(MotorControllerConfigSnapshot.from(motor));
        }
        EncoderConfigSnapshot encoder = EncoderConfigSnapshot.from(config.encoder);
        PidConfig pid = PidConfig.from(config.pidController);
        ProfiledPidConfig profiled = ProfiledPidConfig.from(config.profiledPIDController);
        return new MechanismConfigSnapshot(
                motors,
                encoder,
                config.useAbsolute,
                config.useVoltage,
                config.useSetpointAsOutput,
                config.pidUseVelocity,
                config.customPIDCycle,
                config.pidContinous,
                config.motorCurrentLimit,
                config.motorNeutralMode,
                config.canbus,
                config.encoderGearRatio,
                config.encoderConversion,
                config.encoderConversionOffset,
                config.encoderOffset,
                config.tolerance,
                config.stateMachineDelay,
                config.pidPeriod,
                config.pidIZone,
                config.continousMin,
                config.continousMax,
                config.minBound,
                config.maxBound,
                config.motionLimits,
                pid,
                profiled,
                new ArrayList<>(config.limitSwitches));
    }

    public static MechanismConfigSnapshot snapshot(Mechanism mechanism) {
        if (mechanism == null) {
            return null;
        }
        List<MotorControllerConfigSnapshot> motors = new ArrayList<>();
        MotorController[] controllers = mechanism.getMotorGroup() != null
                ? mechanism.getMotorGroup().getControllers()
                : new MotorController[0];
        for (MotorController controller : controllers) {
            motors.add(MotorControllerConfigSnapshot.from(controller));
        }
        Encoder encoder = mechanism.getEncoder();
        EncoderConfigSnapshot encoderSnapshot = EncoderConfigSnapshot.from(encoder != null ? encoder.getConfig() : null);
        MotorNeutralMode neutralMode = controllers.length > 0 ? controllers[0].getNeutralMode() : MotorNeutralMode.Coast;
        double currentLimit = controllers.length > 0 ? controllers[0].getCurrentLimit() : Double.NaN;
        String canbus = controllers.length > 0 ? controllers[0].getCanbus() : null;
        double gearRatio = encoder != null ? encoder.getGearRatio() : 1.0;
        double conversion = encoder != null ? encoder.getConversion() : 1.0;
        double conversionOffset = encoder != null ? encoder.getConversionOffset() : 0.0;
        double offset = encoder != null ? encoder.getOffset() : 0.0;
        MotionLimits.AxisLimits limits = mechanism.resolveMotionLimits();
        return new MechanismConfigSnapshot(
                motors,
                encoderSnapshot,
                mechanism.isUseAbsolute(),
                mechanism.isUseVoltage(),
                mechanism.isSetpointAsOutput(),
                false,
                mechanism.isCustomPIDCycle(),
                false,
                currentLimit,
                neutralMode,
                canbus,
                gearRatio,
                conversion,
                conversionOffset,
                offset,
                0.0,
                0.0,
                mechanism.getPidPeriod(),
                0.0,
                0.0,
                0.0,
                Double.NaN,
                Double.NaN,
                limits,
                null,
                null,
                List.of());
    }

    public static void apply(MechanismConfig<?> config, MechanismConfigSnapshot snapshot) {
        if (config == null || snapshot == null) {
            return;
        }
        config.motors.clear();
        if (snapshot.motors != null) {
            for (MotorControllerConfigSnapshot motor : snapshot.motors) {
                config.motors.add(motor.toConfig());
            }
        }
        config.encoder = snapshot.encoder != null ? snapshot.encoder.toConfig() : null;
        config.useAbsolute = snapshot.useAbsolute;
        config.useVoltage = snapshot.useVoltage;
        config.useSetpointAsOutput = snapshot.useSetpointAsOutput;
        config.pidUseVelocity = snapshot.pidUseVelocity;
        config.customPIDCycle = snapshot.customPIDCycle;
        config.pidContinous = snapshot.pidContinous;
        config.motorCurrentLimit = snapshot.motorCurrentLimit;
        config.motorNeutralMode = snapshot.motorNeutralMode != null ? snapshot.motorNeutralMode : config.motorNeutralMode;
        if (snapshot.canbus != null) {
            config.canbus = snapshot.canbus;
        }
        config.encoderGearRatio = snapshot.encoderGearRatio;
        config.encoderConversion = snapshot.encoderConversion;
        config.encoderConversionOffset = snapshot.encoderConversionOffset;
        config.encoderOffset = snapshot.encoderOffset;
        config.tolerance = snapshot.tolerance;
        config.stateMachineDelay = snapshot.stateMachineDelay;
        config.pidPeriod = snapshot.pidPeriod;
        config.pidIZone = snapshot.pidIZone;
        config.continousMin = snapshot.continousMin;
        config.continousMax = snapshot.continousMax;
        config.minBound = snapshot.minBound;
        config.maxBound = snapshot.maxBound;
        config.motionLimits = snapshot.motionLimits;
        config.pidController = snapshot.pid != null ? snapshot.pid.toController() : null;
        config.profiledPIDController = snapshot.profiledPid != null ? snapshot.profiledPid.toController() : null;
        config.limitSwitches.clear();
        if (snapshot.limitSwitches != null) {
            config.limitSwitches.addAll(snapshot.limitSwitches);
        }
    }

    public static void apply(Mechanism mechanism, MechanismConfigSnapshot snapshot) {
        if (mechanism == null || snapshot == null) {
            return;
        }
        mechanism.setUseAbsolute(snapshot.useAbsolute);
        mechanism.setUseVoltage(snapshot.useVoltage);
        mechanism.setSetpointAsOutput(snapshot.useSetpointAsOutput);
        mechanism.setCustomPIDCycle(snapshot.customPIDCycle);
        mechanism.setPidPeriod(snapshot.pidPeriod);
        if (snapshot.motionLimits != null) {
            mechanism.setMotionLimits(snapshot.motionLimits);
        }
        if (Double.isFinite(snapshot.minBound) && Double.isFinite(snapshot.maxBound)) {
            mechanism.setBounds(snapshot.minBound, snapshot.maxBound);
        } else {
            mechanism.clearBounds();
        }
        if (snapshot.motorCurrentLimit > 0.0) {
            mechanism.setCurrentLimit(snapshot.motorCurrentLimit);
        }
        if (snapshot.motorNeutralMode != null) {
            mechanism.setMotorNeutralMode(snapshot.motorNeutralMode);
        }
        if (snapshot.encoder != null && mechanism.getEncoder() != null) {
            applyEncoderSnapshot(mechanism.getEncoder(), snapshot.encoder);
        }
        if (snapshot.motors == null || snapshot.motors.isEmpty()) {
            return;
        }
        Map<Integer, MotorControllerConfigSnapshot> byId = new HashMap<>();
        for (MotorControllerConfigSnapshot motor : snapshot.motors) {
            byId.put(motor.id, motor);
        }
        if (mechanism.getMotorGroup() != null) {
            for (MotorController controller : mechanism.getMotorGroup().getControllers()) {
                MotorControllerConfigSnapshot motorSnapshot = byId.get(controller.getId());
                if (motorSnapshot == null) {
                    continue;
                }
                if (motorSnapshot.currentLimit > 0.0) {
                    controller.setCurrentLimit(motorSnapshot.currentLimit);
                }
                controller.setInverted(motorSnapshot.inverted);
                if (motorSnapshot.neutralMode != null) {
                    controller.setNeutralMode(motorSnapshot.neutralMode);
                }
                if (motorSnapshot.pid != null) {
                    controller.setPid(motorSnapshot.pid.toController());
                }
                if (motorSnapshot.encoder != null && controller.getEncoder() != null) {
                    applyEncoderSnapshot(controller.getEncoder(), motorSnapshot.encoder);
                }
            }
        }
    }

    private static void applyEncoderSnapshot(Encoder encoder, EncoderConfigSnapshot snapshot) {
        encoder.setGearRatio(snapshot.gearRatio);
        encoder.setConversion(snapshot.conversion);
        encoder.setConversionOffset(snapshot.conversionOffset);
        encoder.setOffset(snapshot.offset);
        encoder.setInverted(snapshot.inverted);
    }

    public record MechanismConfigSnapshot(
            List<MotorControllerConfigSnapshot> motors,
            EncoderConfigSnapshot encoder,
            boolean useAbsolute,
            boolean useVoltage,
            boolean useSetpointAsOutput,
            boolean pidUseVelocity,
            boolean customPIDCycle,
            boolean pidContinous,
            double motorCurrentLimit,
            MotorNeutralMode motorNeutralMode,
            String canbus,
            double encoderGearRatio,
            double encoderConversion,
            double encoderConversionOffset,
            double encoderOffset,
            double tolerance,
            double stateMachineDelay,
            double pidPeriod,
            double pidIZone,
            double continousMin,
            double continousMax,
            double minBound,
            double maxBound,
            MotionLimits.AxisLimits motionLimits,
            PidConfig pid,
            ProfiledPidConfig profiledPid,
            List<GenericLimitSwitchConfig> limitSwitches) {
    }

    public record MotorControllerConfigSnapshot(
            String typeKey,
            int id,
            String canbus,
            double currentLimit,
            boolean inverted,
            MotorNeutralMode neutralMode,
            EncoderConfigSnapshot encoder,
            PidConfig pid) {

        public static MotorControllerConfigSnapshot from(MotorControllerConfig config) {
            if (config == null) {
                return null;
            }
            String key = config.type != null ? config.type.getKey() : null;
            return new MotorControllerConfigSnapshot(
                    key,
                    config.id,
                    config.canbus,
                    config.currentLimit,
                    config.inverted,
                    config.neutralMode,
                    EncoderConfigSnapshot.from(config.encoderConfig),
                    PidConfig.from(config.pid));
        }

        public static MotorControllerConfigSnapshot from(MotorController controller) {
            if (controller == null) {
                return null;
            }
            String key = controller.getType() != null ? controller.getType().getKey() : null;
            Encoder encoder = controller.getEncoder();
            EncoderConfigSnapshot encoderSnapshot = EncoderConfigSnapshot.from(encoder != null ? encoder.getConfig() : null);
            MotorControllerConfig cfg = controller.getConfig();
            PidConfig pid = cfg != null ? PidConfig.from(cfg.pid) : null;
            return new MotorControllerConfigSnapshot(
                    key,
                    controller.getId(),
                    controller.getCanbus(),
                    controller.getCurrentLimit(),
                    controller.isInverted(),
                    controller.getNeutralMode(),
                    encoderSnapshot,
                    pid);
        }

        public MotorControllerConfig toConfig() {
            MotorControllerType resolved = typeKey != null ? MotorRegistry.get().motor(typeKey) : null;
            MotorControllerConfig config = new MotorControllerConfig(resolved, id);
            if (canbus != null) {
                config.canbus = canbus;
            }
            config.currentLimit = currentLimit;
            config.inverted = inverted;
            config.neutralMode = neutralMode != null ? neutralMode : config.neutralMode;
            config.encoderConfig = encoder != null ? encoder.toConfig() : null;
            config.pid = pid != null ? pid.toController() : null;
            return config;
        }
    }

    public record EncoderConfigSnapshot(
            String typeKey,
            int id,
            String canbus,
            double gearRatio,
            double offset,
            double conversion,
            boolean inverted,
            double conversionOffset) {

        public static EncoderConfigSnapshot from(EncoderConfig config) {
            if (config == null) {
                return null;
            }
            String key = config.type != null ? config.type.getKey() : null;
            return new EncoderConfigSnapshot(
                    key,
                    config.id,
                    config.canbus,
                    config.gearRatio,
                    config.offset,
                    config.conversion,
                    config.inverted,
                    config.conversionOffset);
        }

        public EncoderConfig toConfig() {
            EncoderType resolved = typeKey != null ? EncoderRegistry.get().encoder(typeKey) : null;
            EncoderConfig config = new EncoderConfig();
            config.type = resolved;
            config.id = id;
            config.canbus = canbus;
            config.gearRatio = gearRatio;
            config.offset = offset;
            config.conversion = conversion;
            config.inverted = inverted;
            config.conversionOffset = conversionOffset;
            return config;
        }
    }

    public record PidConfig(double p, double i, double d) {
        public static PidConfig from(PIDController controller) {
            if (controller == null) {
                return null;
            }
            return new PidConfig(controller.getP(), controller.getI(), controller.getD());
        }

        public PIDController toController() {
            return new PIDController(p, i, d);
        }
    }

    public record ProfiledPidConfig(double p, double i, double d, double maxVelocity, double maxAcceleration) {
        public static ProfiledPidConfig from(ProfiledPIDController controller) {
            if (controller == null) {
                return null;
            }
            TrapezoidProfile.Constraints constraints = controller.getConstraints();
            return new ProfiledPidConfig(
                    controller.getP(),
                    controller.getI(),
                    controller.getD(),
                    constraints.maxVelocity,
                    constraints.maxAcceleration);
        }

        public ProfiledPIDController toController() {
            return new ProfiledPIDController(p, i, d, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        }
    }
}
