package ca.frc6390.athena.mechanisms.config;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.MechanismConfigRecord;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

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
        MechanismConfig<?> cfg = mechanism.getSourceConfig();
        if (cfg == null) {
            return new MechanismConfigFile(
                    mechanism.getName(),
                    mechanism.getNetworkTablesTypeName(),
                    null,
                    null,
                    null,
                    null,
                    null,
                    null,
                    null);
        }
        MechanismConfigFile file = export(cfg, mechanism.getNetworkTablesTypeName());
        // Prefer the runtime mechanism name (RobotCore enforces uniqueness and may suffix unnamed mechs).
        return new MechanismConfigFile(
                mechanism.getName(),
                file.mechanismType(),
                file.units(),
                file.motors(),
                file.encoder(),
                file.constraints(),
                file.sensors(),
                file.control(),
                file.sim());
    }

    public static MechanismConfigFile export(MechanismConfig<?> cfg, String mechanismType) {
        Objects.requireNonNull(cfg, "cfg");
        MechanismConfigRecord data = cfg.data();

        MechanismMotorsConfig motors = exportMotors(data);
        MechanismEncoderConfig encoder = exportEncoder(data);
        MechanismConstraintsConfig constraints = exportConstraints(data);
        MechanismSensorsConfig sensors = exportSensors(data);
        MechanismControlConfig control = exportControl(cfg, data);
        MechanismSimConfig sim = exportSim(cfg);

        return new MechanismConfigFile(
                cfg.name(),
                mechanismType,
                null,
                motors,
                encoder,
                constraints,
                sensors,
                control,
                sim);
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
                if (m == null || m.type == null) {
                    continue;
                }
                String typeKey = m.type.getKey();
                controllers.add(new MechanismMotorConfig(
                        null,
                        typeKey,
                        m.id,
                        m.inverted));
            }
        }
        String canbus = data.canbus();
        String neutral = data.motorNeutralMode() != null ? data.motorNeutralMode().name() : null;
        Double currentLimit = Double.isFinite(data.motorCurrentLimit()) ? data.motorCurrentLimit() : null;
        return new MechanismMotorsConfig(canbus, neutral, currentLimit, controllers.isEmpty() ? null : controllers);
    }

    private static MechanismEncoderConfig exportEncoder(MechanismConfigRecord data) {
        if (data == null) {
            return null;
        }
        EncoderConfig enc = data.encoder();
        if (enc == null) {
            return null;
        }
        String source = enc.type != null ? enc.type.getKey() : null;
        return new MechanismEncoderConfig(
                source,
                enc.id,
                null,
                data.useAbsolute(),
                enc.inverted,
                enc.gearRatio,
                enc.conversion,
                enc.conversionOffset,
                enc.offset,
                Double.isFinite(enc.discontinuityPoint) ? enc.discontinuityPoint : null,
                Double.isFinite(enc.discontinuityRange) ? enc.discontinuityRange : null);
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

    private static MechanismControlConfig exportControl(MechanismConfig<?> cfg, MechanismConfigRecord data) {
        if (cfg == null || data == null) {
            return null;
        }
        String output = data.outputType() != null ? data.outputType().name() : null;
        Boolean setpointAsOutput = data.useSetpointAsOutput();
        Boolean pidContinuous = data.pidContinous();
        Double pidContinuousMin = Double.isFinite(data.continousMin()) ? data.continousMin() : null;
        Double pidContinuousMax = Double.isFinite(data.continousMax()) ? data.continousMax() : null;
        Double tolerance = Double.isFinite(data.tolerance()) ? data.tolerance() : null;

        // Export named loop profiles (data-only) so teams can inspect/round-trip them later.
        List<MechanismPidConfig> pidProfiles = null;
        if (cfg.controlLoopPidProfiles != null && !cfg.controlLoopPidProfiles.isEmpty()) {
            pidProfiles = new ArrayList<>();
            for (Map.Entry<String, MechanismConfig.PidProfile> e : cfg.controlLoopPidProfiles.entrySet()) {
                if (e == null || e.getKey() == null || e.getKey().isBlank() || e.getValue() == null) {
                    continue;
                }
                MechanismConfig.PidProfile p = e.getValue();
                Double iZone = Double.isFinite(p.iZone()) ? p.iZone() : null;
                Double toleranceProfile = Double.isFinite(p.tolerance()) ? p.tolerance() : null;
                pidProfiles.add(new MechanismPidConfig(e.getKey(), p.kP(), p.kI(), p.kD(), iZone, null, toleranceProfile));
            }
            if (pidProfiles.isEmpty()) {
                pidProfiles = null;
            }
        }

        List<MechanismFeedforwardConfig> ffProfiles = null;
        if (cfg.controlLoopFeedforwardProfiles != null && !cfg.controlLoopFeedforwardProfiles.isEmpty()) {
            ffProfiles = new ArrayList<>();
            for (Map.Entry<String, MechanismConfig.FeedforwardProfile> e : cfg.controlLoopFeedforwardProfiles.entrySet()) {
                if (e == null || e.getKey() == null || e.getKey().isBlank() || e.getValue() == null) {
                    continue;
                }
                var ff = e.getValue().feedforward();
                ffProfiles.add(new MechanismFeedforwardConfig(
                        e.getKey(),
                        "simple_motor",
                        ff.getKs(),
                        null,
                        ff.getKv(),
                        ff.getKa()));
            }
            if (ffProfiles.isEmpty()) {
                ffProfiles = null;
            }
        }

        List<MechanismBangBangConfig> bangBangProfiles = null;
        if (cfg.controlLoopBangBangProfiles != null && !cfg.controlLoopBangBangProfiles.isEmpty()) {
            bangBangProfiles = new ArrayList<>();
            for (Map.Entry<String, MechanismConfig.BangBangProfile> e : cfg.controlLoopBangBangProfiles.entrySet()) {
                if (e == null || e.getKey() == null || e.getKey().isBlank() || e.getValue() == null) {
                    continue;
                }
                MechanismConfig.BangBangProfile profile = e.getValue();
                String outputProfile = profile.outputType() != null ? profile.outputType().name() : null;
                bangBangProfiles.add(new MechanismBangBangConfig(
                        e.getKey(),
                        outputProfile,
                        profile.highOutput(),
                        profile.lowOutput(),
                        profile.tolerance()));
            }
            if (bangBangProfiles.isEmpty()) {
                bangBangProfiles = null;
            }
        }

        return new MechanismControlConfig(
                output,
                setpointAsOutput,
                pidContinuous,
                pidContinuousMin,
                pidContinuousMax,
                tolerance,
                pidProfiles,
                bangBangProfiles,
                ffProfiles);
    }

    private static MechanismSimConfig exportSim(MechanismConfig<?> cfg) {
        if (cfg == null) {
            return null;
        }
        // Keep this conservative: export only the explicit sim parameters set on the config.
        if (cfg.simpleMotorSimulationParameters != null) {
            var sm = cfg.simpleMotorSimulationParameters;
            return new MechanismSimConfig(
                    new MechanismSimSimpleMotorConfig(
                            Double.isFinite(sm.momentOfInertia) ? sm.momentOfInertia : null,
                            Double.isFinite(sm.nominalVoltage) ? sm.nominalVoltage : null,
                            Double.isFinite(sm.unitsPerRadianOverride) ? sm.unitsPerRadianOverride : null),
                    null,
                    null);
        }
        if (cfg.armSimulationParameters != null) {
            var a = cfg.armSimulationParameters;
            return new MechanismSimConfig(
                    null,
                    new MechanismSimArmConfig(
                            Double.isFinite(a.armLengthMeters) ? a.armLengthMeters : null,
                            Double.isFinite(a.motorReduction) ? a.motorReduction : null,
                            Double.isFinite(a.minAngleRadians) ? Math.toDegrees(a.minAngleRadians) : null,
                            Double.isFinite(a.maxAngleRadians) ? Math.toDegrees(a.maxAngleRadians) : null,
                            Double.isFinite(a.startingAngleRadians) ? Math.toDegrees(a.startingAngleRadians) : null,
                            Double.isFinite(a.unitsPerRadianOverride) ? a.unitsPerRadianOverride : null,
                            a.simulateGravity,
                            Double.isFinite(a.nominalVoltage) ? a.nominalVoltage : null,
                            Double.isFinite(a.momentOfInertia) ? a.momentOfInertia : null),
                    null);
        }
        if (cfg.elevatorSimulationParameters != null) {
            var e = cfg.elevatorSimulationParameters;
            return new MechanismSimConfig(
                    null,
                    null,
                    new MechanismSimElevatorConfig(
                            Double.isFinite(e.drumRadiusMeters) ? e.drumRadiusMeters : null,
                            Double.isFinite(e.carriageMassKg) ? e.carriageMassKg : null,
                            Double.isFinite(e.minHeightMeters) ? e.minHeightMeters : null,
                            Double.isFinite(e.maxHeightMeters) ? e.maxHeightMeters : null,
                            Double.isFinite(e.startingHeightMeters) ? e.startingHeightMeters : null,
                            e.simulateGravity,
                            Double.isFinite(e.nominalVoltage) ? e.nominalVoltage : null,
                            Double.isFinite(e.unitsPerMeterOverride) ? e.unitsPerMeterOverride : null));
        }
        return null;
    }

    private static ObjectMapper buildMapper() {
        ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
        mapper.setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE);
        return mapper;
    }
}
