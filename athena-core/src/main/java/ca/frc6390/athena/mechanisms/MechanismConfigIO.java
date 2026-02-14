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
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonDeserializer;
import com.fasterxml.jackson.databind.JsonSerializer;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.module.SimpleModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public final class MechanismConfigIO {
    private static final ObjectMapper MAPPER = buildMapper();

    private MechanismConfigIO() {
    }

    public static MechanismConfigRecord load(Path path) {
        try {
            return MAPPER.readValue(path.toFile(), MechanismConfigRecord.class);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to load mechanism config from " + path, e);
        }
    }

    public static void save(Path path, MechanismConfigRecord record) {
        try {
            MAPPER.writeValue(path.toFile(), record);
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to save mechanism config to " + path, e);
        }
    }

    public static MechanismConfigRecord snapshot(MechanismConfig<?> config) {
        if (config == null) {
            return null;
        }
        return config.data();
    }

    public static MechanismConfigRecord snapshot(Mechanism mechanism) {
        if (mechanism == null) {
            return null;
        }
        List<MotorControllerConfig> motors = new ArrayList<>();
        MotorController[] controllers = mechanism.getMotorGroup() != null
                ? mechanism.getMotorGroup().getControllers()
                : new MotorController[0];
        for (MotorController controller : controllers) {
            MotorControllerConfig config = controller.getConfig();
            if (config != null) {
                motors.add(config);
            } else if (controller.getType() != null) {
                MotorControllerConfig fallback = MotorControllerConfig.create(controller.getType(), controller.getId());
                fallback.hardware()
                        .canbus(controller.getCanbus())
                        .currentLimit(controller.getCurrentLimit())
                        .inverted(controller.isInverted())
                        .neutralMode(controller.getNeutralMode());
                motors.add(fallback);
            }
        }
        Encoder encoder = mechanism.getEncoder();
        EncoderConfig encoderConfig = encoder != null ? encoder.getConfig() : null;
        MotorNeutralMode neutralMode = controllers.length > 0 ? controllers[0].getNeutralMode() : MotorNeutralMode.Coast;
        double currentLimit = controllers.length > 0 ? controllers[0].getCurrentLimit() : Double.NaN;
        String canbus = controllers.length > 0 ? controllers[0].getCanbus() : null;
        double gearRatio = encoder != null ? encoder.getGearRatio() : 1.0;
        double conversion = encoder != null ? encoder.getConversion() : 1.0;
        double conversionOffset = encoder != null ? encoder.getConversionOffset() : 0.0;
        double offset = encoder != null ? encoder.getOffset() : 0.0;
        MotionLimits.AxisLimits limits = mechanism.resolveMotionLimits();
        return MechanismConfigRecord.defaults().toBuilder()
                .motors(motors)
                .encoder(encoderConfig)
                .useAbsolute(mechanism.isUseAbsolute())
                .outputType(mechanism.getOutputType())
                .useSetpointAsOutput(mechanism.isSetpointAsOutput())
                .customPIDCycle(mechanism.isCustomPIDCycle())
                .pidPeriod(mechanism.getPidPeriod())
                .motorCurrentLimit(currentLimit)
                .motorNeutralMode(neutralMode)
                .canbus(canbus)
                .encoderGearRatio(gearRatio)
                .encoderConversion(conversion)
                .encoderConversionOffset(conversionOffset)
                .encoderOffset(offset)
                .motionLimits(limits)
                .build();
    }

    public static void apply(MechanismConfig<?> config, MechanismConfigRecord record) {
        if (config == null || record == null) {
            return;
        }
        config.data(record);
    }

    public static void apply(Mechanism mechanism, MechanismConfigRecord record) {
        if (mechanism == null || record == null) {
            return;
        }
        mechanism.setUseAbsolute(record.useAbsolute());
        if (record.outputType() != null) {
            mechanism.setOutputType(record.outputType());
        } else {
            mechanism.setOutputType(record.useVoltage() ? OutputType.VOLTAGE : OutputType.PERCENT);
        }
        mechanism.setSetpointAsOutput(record.useSetpointAsOutput());
        mechanism.setCustomPIDCycle(record.customPIDCycle());
        mechanism.setPidPeriod(record.pidPeriod());
        if (record.motionLimits() != null) {
            mechanism.setMotionLimits(record.motionLimits());
        }
        if (Double.isFinite(record.minBound()) && Double.isFinite(record.maxBound())) {
            mechanism.setBounds(record.minBound(), record.maxBound());
        } else {
            mechanism.clearBounds();
        }
        if (record.motorCurrentLimit() > 0.0) {
            mechanism.setCurrentLimit(record.motorCurrentLimit());
        }
        if (record.motorNeutralMode() != null) {
            mechanism.setMotorNeutralMode(record.motorNeutralMode());
        }
        if (record.encoder() != null && mechanism.getEncoder() != null) {
            applyEncoderConfig(mechanism.getEncoder(), record.encoder());
        }
        if (record.motors() == null || record.motors().isEmpty()) {
            return;
        }
        if (mechanism.getMotorGroup() != null) {
            for (MotorController controller : mechanism.getMotorGroup().getControllers()) {
                record.motors().stream()
                        .filter(motor -> motor.id() == controller.getId())
                        .findFirst()
                        .ifPresent(motor -> applyMotorConfig(controller, motor));
            }
        }
    }

    private static void applyMotorConfig(MotorController controller, MotorControllerConfig config) {
        if (config.currentLimit() > 0.0) {
            controller.setCurrentLimit(config.currentLimit());
        }
        controller.setInverted(config.inverted());
        if (config.neutralMode() != null) {
            controller.setNeutralMode(config.neutralMode());
        }
        if (config.pid() != null) {
            controller.setPid(config.pid());
        }
        if (config.encoderConfig() != null && controller.getEncoder() != null) {
            applyEncoderConfig(controller.getEncoder(), config.encoderConfig());
        }
    }

    private static void applyEncoderConfig(Encoder encoder, EncoderConfig config) {
        encoder.setGearRatio(config.gearRatio());
        encoder.setConversion(config.conversion());
        encoder.setConversionOffset(config.conversionOffset());
        encoder.setOffset(config.offset());
        encoder.setInverted(config.inverted());
    }

    private static ObjectMapper buildMapper() {
        ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
        SimpleModule module = new SimpleModule();
        module.addSerializer(MotorControllerType.class, new MotorControllerTypeSerializer());
        module.addDeserializer(MotorControllerType.class, new MotorControllerTypeDeserializer());
        module.addSerializer(EncoderType.class, new EncoderTypeSerializer());
        module.addDeserializer(EncoderType.class, new EncoderTypeDeserializer());
        module.addSerializer(PIDController.class, new PIDControllerSerializer());
        module.addDeserializer(PIDController.class, new PIDControllerDeserializer());
        module.addSerializer(ProfiledPIDController.class, new ProfiledPIDControllerSerializer());
        module.addDeserializer(ProfiledPIDController.class, new ProfiledPIDControllerDeserializer());
        mapper.registerModule(module);
        return mapper;
    }

    private static final class MotorControllerTypeSerializer extends JsonSerializer<MotorControllerType> {
        @Override
        public void serialize(MotorControllerType value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
            if (value == null) {
                gen.writeNull();
                return;
            }
            gen.writeString(value.getKey());
        }
    }

    private static final class MotorControllerTypeDeserializer extends JsonDeserializer<MotorControllerType> {
        @Override
        public MotorControllerType deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
            if (p.currentToken() == JsonToken.VALUE_NULL) {
                return null;
            }
            String key = p.getValueAsString();
            if (key == null || key.isBlank()) {
                return null;
            }
            return MotorRegistry.get().motor(key);
        }
    }

    private static final class EncoderTypeSerializer extends JsonSerializer<EncoderType> {
        @Override
        public void serialize(EncoderType value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
            if (value == null) {
                gen.writeNull();
                return;
            }
            gen.writeString(value.getKey());
        }
    }

    private static final class EncoderTypeDeserializer extends JsonDeserializer<EncoderType> {
        @Override
        public EncoderType deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
            if (p.currentToken() == JsonToken.VALUE_NULL) {
                return null;
            }
            String key = p.getValueAsString();
            if (key == null || key.isBlank()) {
                return null;
            }
            return EncoderRegistry.get().encoder(key);
        }
    }

    private static final class PIDControllerSerializer extends JsonSerializer<PIDController> {
        @Override
        public void serialize(PIDController value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
            if (value == null) {
                gen.writeNull();
                return;
            }
            gen.writeStartObject();
            gen.writeNumberField("p", value.getP());
            gen.writeNumberField("i", value.getI());
            gen.writeNumberField("d", value.getD());
            gen.writeEndObject();
        }
    }

    private static final class PIDControllerDeserializer extends JsonDeserializer<PIDController> {
        @Override
        public PIDController deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
            if (p.currentToken() == JsonToken.VALUE_NULL) {
                return null;
            }
            double pGain = 0.0;
            double iGain = 0.0;
            double dGain = 0.0;
            while (p.nextToken() != JsonToken.END_OBJECT) {
                String name = p.currentName();
                p.nextToken();
                if ("p".equals(name)) {
                    pGain = p.getDoubleValue();
                } else if ("i".equals(name)) {
                    iGain = p.getDoubleValue();
                } else if ("d".equals(name)) {
                    dGain = p.getDoubleValue();
                } else {
                    p.skipChildren();
                }
            }
            return new PIDController(pGain, iGain, dGain);
        }
    }

    private static final class ProfiledPIDControllerSerializer extends JsonSerializer<ProfiledPIDController> {
        @Override
        public void serialize(ProfiledPIDController value, JsonGenerator gen, SerializerProvider serializers) throws IOException {
            if (value == null) {
                gen.writeNull();
                return;
            }
            TrapezoidProfile.Constraints constraints = value.getConstraints();
            gen.writeStartObject();
            gen.writeNumberField("p", value.getP());
            gen.writeNumberField("i", value.getI());
            gen.writeNumberField("d", value.getD());
            gen.writeNumberField("maxVelocity", constraints.maxVelocity);
            gen.writeNumberField("maxAcceleration", constraints.maxAcceleration);
            gen.writeEndObject();
        }
    }

    private static final class ProfiledPIDControllerDeserializer extends JsonDeserializer<ProfiledPIDController> {
        @Override
        public ProfiledPIDController deserialize(JsonParser p, DeserializationContext ctxt) throws IOException {
            if (p.currentToken() == JsonToken.VALUE_NULL) {
                return null;
            }
            double pGain = 0.0;
            double iGain = 0.0;
            double dGain = 0.0;
            double maxVelocity = 0.0;
            double maxAcceleration = 0.0;
            while (p.nextToken() != JsonToken.END_OBJECT) {
                String name = p.currentName();
                p.nextToken();
                if ("p".equals(name)) {
                    pGain = p.getDoubleValue();
                } else if ("i".equals(name)) {
                    iGain = p.getDoubleValue();
                } else if ("d".equals(name)) {
                    dGain = p.getDoubleValue();
                } else if ("maxVelocity".equals(name)) {
                    maxVelocity = p.getDoubleValue();
                } else if ("maxAcceleration".equals(name)) {
                    maxAcceleration = p.getDoubleValue();
                } else {
                    p.skipChildren();
                }
            }
            return new ProfiledPIDController(pGain, iGain, dGain, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        }
    }
}
