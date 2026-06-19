package ca.frc6390.athena.mechanisms.config;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfigRecord;
import ca.frc6390.athena.mechanisms.MechanismInputSource;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Data-only export helpers for Athena mechanisms.
 *
 * <p>Exports are intended for inspection and copying into deploy files. Code-only concepts like
 * hooks, lambdas, and custom control loops are intentionally excluded.
 */
public final class MechanismConfigExport {
    private static final ObjectMapper MAPPER = buildMapper();

    private MechanismConfigExport() {}

    public static MechanismConfigFile export(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        MechanismConfigRecord data = ca.frc6390.athena.mechanisms.MechanismConfigIO.snapshot(mechanism);
        MechanismMotorsConfig motors = exportMotors(data);
        List<MechanismEncoderConfig> encoders = exportEncoders(data);
        MechanismConstraintsConfig constraints = exportConstraints(data);
        MechanismSensorsConfig sensors = exportSensors(data);
        MechanismControlConfig control = exportControl(mechanism, data, exportedEncoderNames(encoders));
        return new MechanismConfigFile(
                mechanism.getName(),
                mechanism.networkTables().typeName(),
                null,
                motors,
                encoders,
                constraints,
                sensors,
                control,
                null);
    }

    public static String toJson(MechanismConfigFile file) {
        Objects.requireNonNull(file, "file");
        try {
            return MAPPER.writeValueAsString(file);
        } catch (JsonProcessingException e) {
            throw new IllegalArgumentException("Failed to serialize mechanism config to JSON", e);
        }
    }

    public static String toToml(MechanismConfigFile file) {
        Objects.requireNonNull(file, "file");
        JsonNode node = MAPPER.valueToTree(file);
        return AthenaTomlWriter.write(node);
    }

    private static MechanismMotorsConfig exportMotors(MechanismConfigRecord data) {
        if (data == null) {
            return null;
        }
        List<MechanismMotorConfig> controllers = new ArrayList<>();
        if (data.motors() != null) {
            for (MotorControllerConfig m : data.motors()) {
                if (m == null || m.type() == null) {
                    continue;
                }
                String typeKey = m.type().getKey();
                controllers.add(new MechanismMotorConfig(
                        null,
                        typeKey,
                        m.id(),
                        m.inverted()));
            }
        }
        String canbus = data.canbus();
        String neutral = data.motorNeutralMode() != null ? data.motorNeutralMode().name() : null;
        Double currentLimit = Double.isFinite(data.motorCurrentLimit()) ? data.motorCurrentLimit() : null;
        return new MechanismMotorsConfig(canbus, neutral, currentLimit, controllers.isEmpty() ? null : controllers);
    }

    private static List<MechanismEncoderConfig> exportEncoders(MechanismConfigRecord data) {
        if (data == null) {
            return null;
        }
        if (data.encoders() == null || data.encoders().isEmpty()) {
            return null;
        }
        List<MechanismEncoderConfig> candidates = new ArrayList<>();
        for (MechanismEncoderConfig enc : data.encoders()) {
            if (enc == null || enc.name() == null || enc.name().isBlank()) {
                continue;
            }
            if (enc.source() != null && enc.source().equalsIgnoreCase("virtual")) {
                continue;
            }
            candidates.add(enc);
        }
        if (candidates.isEmpty()) {
            return null;
        }
        Set<String> exportedNames = new LinkedHashSet<>();
        boolean progress;
        do {
            progress = false;
            for (MechanismEncoderConfig enc : candidates) {
                if (exportedNames.contains(enc.name())) {
                    continue;
                }
                if (encoderInputsResolved(enc, exportedNames)) {
                    exportedNames.add(enc.name());
                    progress = true;
                }
            }
        } while (progress);

        List<MechanismEncoderConfig> exportable = new ArrayList<>();
        for (MechanismEncoderConfig enc : candidates) {
            if (exportedNames.contains(enc.name()) && encoderInputsResolved(enc, exportedNames)) {
                exportable.add(enc);
            }
        }
        return exportable.isEmpty() ? null : List.copyOf(exportable);
    }

    private static boolean encoderInputsResolved(MechanismEncoderConfig enc, Set<String> exportedNames) {
        if (enc.inputs() == null || enc.inputs().isEmpty()) {
            return true;
        }
        for (MechanismEncoderInputConfig input : enc.inputs()) {
            if (input == null || input.source() == null || input.source().isBlank()) {
                return false;
            }
            if (!exportedNames.contains(input.source())) {
                return false;
            }
        }
        return true;
    }

    private static Set<String> exportedEncoderNames(List<MechanismEncoderConfig> encoders) {
        LinkedHashSet<String> names = new LinkedHashSet<>();
        if (encoders != null) {
            for (MechanismEncoderConfig enc : encoders) {
                if (enc != null && enc.name() != null && !enc.name().isBlank()) {
                    names.add(enc.name());
                }
            }
        }
        return names;
    }

    private static MechanismConstraintsConfig exportConstraints(MechanismConfigRecord data) {
        if (data == null) {
            return null;
        }
        Double min = Double.isFinite(data.minBound()) ? data.minBound() : null;
        Double max = Double.isFinite(data.maxBound()) ? data.maxBound() : null;
        MotionLimits.AxisLimits limits = data.motionLimits();
        MechanismMotionLimitsConfig motion = null;
        if (limits != null) {
            Double v = limits.maxVelocity() > 0.0 ? limits.maxVelocity() : null;
            Double a = limits.maxAcceleration() > 0.0 ? limits.maxAcceleration() : null;
            if (v != null || a != null) {
                motion = new MechanismMotionLimitsConfig(v, a);
            }
        }
        if (min == null && max == null && motion == null) {
            return null;
        }
        return new MechanismConstraintsConfig(min, max, null, motion);
    }

    private static MechanismSensorsConfig exportSensors(MechanismConfigRecord data) {
        if (data == null) {
            return null;
        }
        Double hardwareUpdatePeriodMs = null;
        if (Double.isFinite(data.hardwareUpdatePeriodSeconds())
                && data.hardwareUpdatePeriodSeconds() > 0.0
                && Math.abs(data.hardwareUpdatePeriodSeconds() - 0.02) > 1e-9) {
            hardwareUpdatePeriodMs = data.hardwareUpdatePeriodSeconds() * 1000.0;
        }
        List<MechanismLimitSwitchConfig> switches = new ArrayList<>();
        if (data.limitSwitches() != null) {
            for (GenericLimitSwitchConfig sw : data.limitSwitches()) {
                if (sw == null) {
                    continue;
                }
                switches.add(new MechanismLimitSwitchConfig(
                        sw.id(),
                        sw.inverted(),
                        Double.isFinite(sw.position()) ? sw.position() : null,
                        sw.isHardstop(),
                        sw.blockDirection() != null ? sw.blockDirection().name() : null,
                        sw.name(),
                        sw.delaySeconds()));
            }
        }
        if (switches.isEmpty() && hardwareUpdatePeriodMs == null) {
            return null;
        }
        return new MechanismSensorsConfig(switches.isEmpty() ? null : switches, hardwareUpdatePeriodMs);
    }

    private static MechanismControlConfig exportControl(
            Mechanism mechanism,
            MechanismConfigRecord data,
            Set<String> exportedEncoderNames) {
        if (mechanism == null || data == null) {
            return null;
        }
        String output = data.outputType() != null ? data.outputType().name() : null;
        Boolean setpointAsOutput = data.useSetpointAsOutput();
        Boolean pidContinuous = data.pidContinous();
        Double pidContinuousMin = Double.isFinite(data.continousMin()) ? data.continousMin() : null;
        Double pidContinuousMax = Double.isFinite(data.continousMax()) ? data.continousMax() : null;
        Double tolerance = Double.isFinite(data.tolerance()) ? data.tolerance() : null;

        List<MechanismPidConfig> pidProfiles = null;
        if (!mechanism.getControlLoopPidProfiles().isEmpty()) {
            pidProfiles = new ArrayList<>();
            for (Map.Entry<String, ca.frc6390.athena.mechanisms.MechanismRuntimeConfig.PidProfile> e
                    : mechanism.getControlLoopPidProfiles().entrySet()) {
                if (e == null || e.getKey() == null || e.getKey().isBlank() || e.getValue() == null) {
                    continue;
                }
                var p = e.getValue();
                Double iZone = Double.isFinite(p.iZone()) ? p.iZone() : null;
                Double toleranceProfile = Double.isFinite(p.tolerance()) ? p.tolerance() : null;
                Double maxVelocity = Double.isFinite(p.maxVelocity()) ? p.maxVelocity() : null;
                Double maxAcceleration = Double.isFinite(p.maxAcceleration()) ? p.maxAcceleration() : null;
                pidProfiles.add(new MechanismPidConfig(
                        e.getKey(),
                        p.kP(),
                        p.kI(),
                        p.kD(),
                        iZone,
                        null,
                        toleranceProfile,
                        maxVelocity,
                        maxAcceleration,
                        formatMeasurementSource(p.inputSource(), exportedEncoderNames)));
            }
            if (pidProfiles.isEmpty()) {
                pidProfiles = null;
            }
        }

        List<MechanismFeedforwardConfig> ffProfiles = null;
        if (!mechanism.getControlLoopFeedforwardProfiles().isEmpty()) {
            ffProfiles = new ArrayList<>();
            for (Map.Entry<String, ca.frc6390.athena.mechanisms.MechanismRuntimeConfig.FeedforwardProfile> e
                    : mechanism.getControlLoopFeedforwardProfiles().entrySet()) {
                if (e == null || e.getKey() == null || e.getKey().isBlank() || e.getValue() == null) {
                    continue;
                }
                var ff = e.getValue();
                Double toleranceProfile = Double.isFinite(ff.tolerance()) ? ff.tolerance() : null;
                ffProfiles.add(new MechanismFeedforwardConfig(
                        e.getKey(),
                        ff.type() != null ? ff.type().name().toLowerCase(Locale.ROOT) : "simple",
                        ff.kS(),
                        ff.kG(),
                        ff.kV(),
                        ff.kA(),
                        toleranceProfile,
                        formatSetpointSource(ff.setpointSource(), exportedEncoderNames)));
            }
            if (ffProfiles.isEmpty()) {
                ffProfiles = null;
            }
        }

        List<MechanismBangBangConfig> bangBangProfiles = null;
        if (!mechanism.getControlLoopBangBangProfiles().isEmpty()) {
            bangBangProfiles = new ArrayList<>();
            for (Map.Entry<String, ca.frc6390.athena.mechanisms.MechanismRuntimeConfig.BangBangProfile> e
                    : mechanism.getControlLoopBangBangProfiles().entrySet()) {
                if (e == null || e.getKey() == null || e.getKey().isBlank() || e.getValue() == null) {
                    continue;
                }
                var profile = e.getValue();
                String outputProfile = profile.outputType() != null ? profile.outputType().name() : null;
                bangBangProfiles.add(new MechanismBangBangConfig(
                        e.getKey(),
                        outputProfile,
                        profile.highOutput(),
                        profile.lowOutput(),
                        profile.tolerance(),
                        formatMeasurementSource(profile.inputSource(), exportedEncoderNames)));
            }
            if (bangBangProfiles.isEmpty()) {
                bangBangProfiles = null;
            }
        }

        return new MechanismControlConfig(
                output,
                exportSourceName(data.positionSource(), exportedEncoderNames),
                exportSourceName(data.velocitySource(), exportedEncoderNames),
                exportSourceName(data.absoluteSource(), exportedEncoderNames),
                setpointAsOutput,
                pidContinuous,
                pidContinuousMin,
                pidContinuousMax,
                tolerance,
                pidProfiles,
                bangBangProfiles,
                ffProfiles);
    }

    private static String formatMeasurementSource(MechanismInputSource source, Set<String> exportedEncoderNames) {
        if (source == null) {
            return null;
        }
        return switch (source.kind()) {
            case POSITION -> formatNamedMeasurement("position", source.encoderId(), exportedEncoderNames);
            case VELOCITY -> formatNamedMeasurement("velocity", source.encoderId(), exportedEncoderNames);
            case ABSOLUTE -> formatNamedMeasurement("absolute", source.encoderId(), exportedEncoderNames);
            case INPUT -> source.inputKey() != null && !source.inputKey().isBlank()
                    ? "input:" + source.inputKey()
                    : null;
        };
    }

    private static String formatSetpointSource(MechanismSetpointSource source, Set<String> exportedEncoderNames) {
        if (source == null) {
            return null;
        }
        return switch (source.kind()) {
            case SETPOINT -> "setpoint";
            case INPUT -> source.inputKey() != null && !source.inputKey().isBlank()
                    ? "input:" + source.inputKey()
                    : null;
        };
    }

    private static String formatNamedMeasurement(String prefix, String encoderId, Set<String> exportedEncoderNames) {
        if (encoderId == null || encoderId.isBlank()) {
            return prefix;
        }
        return exportSourceName(encoderId, exportedEncoderNames) != null ? prefix + ":" + encoderId : null;
    }

    private static String exportSourceName(String sourceName, Set<String> exportedEncoderNames) {
        if (sourceName == null || sourceName.isBlank()) {
            return null;
        }
        return exportedEncoderNames != null && exportedEncoderNames.contains(sourceName) ? sourceName : null;
    }

    private static ObjectMapper buildMapper() {
        ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
        mapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        return mapper;
    }
}
