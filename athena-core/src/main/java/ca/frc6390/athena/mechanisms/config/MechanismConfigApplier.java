package ca.frc6390.athena.mechanisms.config;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorRegistry;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.MechanismEncoderSourceDsl;
import ca.frc6390.athena.mechanisms.MechanismEncoderUnit;
import ca.frc6390.athena.mechanisms.MechanismInputSource;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.BlockDirection;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Locale;
import java.util.Objects;
import java.util.Set;

/**
 * Applies a data-only {@link MechanismConfigFile} to Athena's fluent {@link MechanismConfig} builder.
 *
 * <p>This keeps deploy-file parsing isolated from the runtime builder.
 */
public final class MechanismConfigApplier {
    private MechanismConfigApplier() {
    }

    public static void apply(MechanismConfig<?> target, MechanismConfigFile file) {
        Objects.requireNonNull(target, "target");
        Objects.requireNonNull(file, "file");

        if (file.name() != null && !file.name().isBlank()) {
            target.named(file.name());
        }

        if (file.motors() != null) {
            applyMotors(target, file.motors());
        }
        if (file.encoders() != null && !file.encoders().isEmpty()) {
            applyEncoders(target, file.encoders());
        }
        if (file.constraints() != null) {
            applyConstraints(target, file.constraints());
        }
        if (file.sensors() != null) {
            applySensors(target, file.sensors());
        }
        if (file.control() != null) {
            applyControl(target, file.control());
        }
        if (file.sim() != null) {
            applySim(target, file.sim());
        }
    }

    private static void applyMotors(MechanismConfig<?> target, MechanismMotorsConfig motors) {
        target.motors(section -> {
            if (motors.canbus() != null && !motors.canbus().isBlank()) {
                section.canbus(motors.canbus());
            }
            if (motors.neutralMode() != null && !motors.neutralMode().isBlank()) {
                section.neutralMode(parseNeutralMode(motors.neutralMode()));
            }
            if (motors.currentLimit() != null && motors.currentLimit() > 0.0) {
                section.currentLimit(motors.currentLimit());
            }
            if (motors.controllers() == null) {
                return;
            }
            for (MechanismMotorConfig motor : motors.controllers()) {
                if (motor == null || motor.id() == null) {
                    continue;
                }
                String motorName = motor.motor() != null ? motor.motor().trim() : "";
                String typeKey = motor.type() != null ? motor.type().trim() : "";
                int id = motor.id();
                boolean inverted = motor.inverted() != null ? motor.inverted() : (id < 0);
                int signed = inverted ? -Math.abs(id) : Math.abs(id);
                if (!motorName.isBlank()) {
                    AthenaMotor am = AthenaMotor.valueOf(motorName.toUpperCase(Locale.ROOT));
                    section.add(am, signed);
                } else if (!typeKey.isBlank()) {
                    MotorControllerType type = MotorRegistry.get().motor(typeKey.toLowerCase(Locale.ROOT));
                    section.add(type, signed);
                }
            }
        });
    }

    private static void applyEncoders(MechanismConfig<?> target, java.util.List<MechanismEncoderConfig> encoders) {
        ArrayList<MechanismEncoderConfig> pending = new ArrayList<>();
        for (MechanismEncoderConfig enc : encoders) {
            if (enc != null) {
                pending.add(enc);
            }
        }
        Set<String> applied = new HashSet<>();
        while (!pending.isEmpty()) {
            boolean progress = false;
            for (int i = 0; i < pending.size(); i++) {
                MechanismEncoderConfig enc = pending.get(i);
                if (!canApplyEncoder(enc, applied)) {
                    continue;
                }
                applyEncoder(target, enc);
                applied.add(normalizeEncoderName(enc.name()));
                pending.remove(i);
                i--;
                progress = true;
            }
            if (!progress) {
                throw new IllegalArgumentException("encoder source graph contains unresolved derived-source dependencies");
            }
        }
    }

    private static boolean canApplyEncoder(MechanismEncoderConfig enc, Set<String> applied) {
        if (enc.inputs() == null || enc.inputs().isEmpty()) {
            return true;
        }
        for (MechanismEncoderInputConfig input : enc.inputs()) {
            if (input == null || input.source() == null || input.source().isBlank()) {
                throw new IllegalArgumentException(
                        "encoder source '" + enc.name() + "' has a blank input source");
            }
            if (!applied.contains(normalizeEncoderName(input.source()))) {
                return false;
            }
        }
        return true;
    }

    private static void applyEncoder(MechanismConfig<?> target, MechanismEncoderConfig enc) {
        String name = normalizeEncoderName(enc.name());
        String source = enc.source() != null ? enc.source().trim().toLowerCase(Locale.ROOT) : "";
        target.encoders(section -> section.add(name, builder -> {
            if ("virtual".equals(source)) {
                throw new IllegalArgumentException("encoder source '" + name + "' uses code-only source=virtual");
            }
            switch (source) {
                case "crt" -> builder.crt(crt -> {
                    applyCrtInputs(name, enc, crt);
                    if (enc.validMin() != null && enc.validMax() != null) {
                        crt.validRange(enc.validMin(), enc.validMax());
                    }
                    return crt;
                });
                case "filter" -> builder.filter(filter -> {
                    applyDerivedInputs(name, enc.inputs(), MechanismConfig.InputSource.Position, filter::input);
                    applyFilter(name, enc, filter);
                    return filter;
                });
                case "differentiate" -> builder.differentiate(diff -> {
                    applyDerivedInputs(name, enc.inputs(), MechanismConfig.InputSource.Position, diff::input);
                    applyFilter(name, enc, diff);
                    return diff;
                });
                case "average" -> builder.average(avg -> {
                    applyDerivedInputs(name, enc.inputs(), MechanismConfig.InputSource.Position, avg::input);
                    return avg;
                });
                case "difference" -> builder.difference(diff -> {
                    applyDerivedInputs(name, enc.inputs(), MechanismConfig.InputSource.Position, diff::input);
                    return diff;
                });
                case "calibration_map", "calibration-map", "calibrationmap" -> builder.calibrationMap(map -> {
                    applyDerivedInputs(name, enc.inputs(), MechanismConfig.InputSource.Position, map::input);
                    applyCalibrationPoints(name, enc, map);
                    return map;
                });
                default -> {
                    int signedId = signedEncoderId(enc);
                    if ("internal".equals(source)) {
                        builder.encoder(AthenaEncoder.INTERNAL, signedId);
                    } else {
                        try {
                            AthenaEncoder ae = AthenaEncoder.valueOf(source.toUpperCase(Locale.ROOT));
                            builder.encoder(ae, signedId);
                        } catch (IllegalArgumentException ignored) {
                            builder.encoder(
                                    ca.frc6390.athena.hardware.encoder.EncoderRegistry.get().encoder(source),
                                    signedId);
                        }
                    }
                }
            }
            applyEncoderSettings(builder, enc);
            return builder;
        }));
    }

    @FunctionalInterface
    private interface DerivedInputConsumer {
        void accept(String sourceName, MechanismConfig.InputSource signal);
    }

    private static void applyDerivedInputs(
            String ownerName,
            java.util.List<MechanismEncoderInputConfig> inputs,
            MechanismConfig.InputSource defaultSource,
            DerivedInputConsumer consumer) {
        if (inputs == null || inputs.isEmpty()) {
            throw new IllegalArgumentException(
                    "encoder source '" + ownerName + "' requires at least one input");
        }
        for (MechanismEncoderInputConfig input : inputs) {
            if (input == null || input.source() == null || input.source().isBlank()) {
                throw new IllegalArgumentException(
                        "encoder source '" + ownerName + "' has a blank input source");
            }
            consumer.accept(input.source(), parseDerivedInputSource(input.signal(), defaultSource));
        }
    }

    private static void applyCrtInputs(
            String ownerName,
            MechanismEncoderConfig enc,
            MechanismConfig.CrtSourceBuilder crt) {
        if (enc.inputs() == null || enc.inputs().isEmpty()) {
            throw new IllegalArgumentException(
                    "encoder source '" + ownerName + "' requires inputs when source=crt");
        }
        for (MechanismEncoderInputConfig input : enc.inputs()) {
            if (input == null || input.source() == null || input.source().isBlank()) {
                throw new IllegalArgumentException(
                        "encoder source '" + ownerName + "' has a blank input source");
            }
            crt.input(
                    input.source(),
                    parseDerivedInputSource(input.signal(), MechanismConfig.InputSource.Absolute),
                    input.modulus() != null ? input.modulus() : 0);
        }
    }

    private static void applyFilter(
            String ownerName,
            MechanismEncoderConfig enc,
            MechanismConfig.FilterSourceBuilder builder) {
        String filter = enc.filter() != null ? enc.filter().trim().toLowerCase(Locale.ROOT) : "";
        switch (filter) {
            case "low_pass", "low-pass", "lowpass" -> builder.lowPass(
                    enc.filterAlpha() != null ? enc.filterAlpha() : Double.NaN);
            case "median" -> builder.median(enc.filterWindow() != null ? enc.filterWindow() : 0);
            case "moving_average", "moving-average", "movingaverage" -> builder.movingAverage(
                    enc.filterWindow() != null ? enc.filterWindow() : 0);
            case "" -> throw new IllegalArgumentException(
                    "encoder source '" + ownerName + "' requires a filter type");
            default -> throw new IllegalArgumentException(
                    "encoder source '" + ownerName + "' has an unknown filter type: " + enc.filter());
        }
    }

    private static void applyFilter(
            String ownerName,
            MechanismEncoderConfig enc,
            MechanismConfig.DifferentiateSourceBuilder builder) {
        String filter = enc.filter() != null ? enc.filter().trim().toLowerCase(Locale.ROOT) : "";
        switch (filter) {
            case "" -> {
                return;
            }
            case "low_pass", "low-pass", "lowpass" -> builder.lowPass(
                    enc.filterAlpha() != null ? enc.filterAlpha() : Double.NaN);
            case "median" -> builder.median(enc.filterWindow() != null ? enc.filterWindow() : 0);
            case "moving_average", "moving-average", "movingaverage" -> builder.movingAverage(
                    enc.filterWindow() != null ? enc.filterWindow() : 0);
            default -> throw new IllegalArgumentException(
                    "encoder source '" + ownerName + "' has an unknown filter type: " + enc.filter());
        }
    }

    private static void applyCalibrationPoints(
            String ownerName,
            MechanismEncoderConfig enc,
            MechanismConfig.CalibrationMapSourceBuilder builder) {
        if (enc.points() == null || enc.points().isEmpty()) {
            throw new IllegalArgumentException(
                    "encoder source '" + ownerName + "' requires points when source=calibration_map");
        }
        for (MechanismEncoderCalibrationPointConfig point : enc.points()) {
            if (point == null || point.input() == null || point.output() == null) {
                throw new IllegalArgumentException(
                        "encoder source '" + ownerName + "' has a calibration point with a null input/output");
            }
            builder.point(point.input(), point.output());
        }
    }

    private static String normalizeEncoderName(String name) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("encoder source name cannot be blank");
        }
        return name.trim();
    }

    private static int signedEncoderId(MechanismEncoderConfig enc) {
        if (enc.id() == null) {
            throw new IllegalArgumentException("encoder source '" + enc.name() + "' requires an id");
        }
        int id = Math.abs(enc.id());
        return Boolean.TRUE.equals(enc.inverted()) ? -id : id;
    }

    private static void applyEncoderSettings(MechanismEncoderSourceDsl builder, MechanismEncoderConfig enc) {
        if (enc.canbus() != null && !enc.canbus().isBlank()) {
            builder.canbus(enc.canbus());
        }
        if (enc.gearRatio() != null) {
            builder.gearRatio(enc.gearRatio());
        }
        if (enc.conversion() != null) {
            builder.conversion(enc.conversion());
        }
        if (enc.offset() != null) {
            builder.offset(enc.offset());
        }
        if (enc.unit() != null && !enc.unit().isBlank()) {
            builder.unit(parseEncoderUnit(enc.unit()));
        }
        if (enc.wrapsEvery() != null) {
            builder.wrapsEvery(enc.wrapsEvery());
        }
    }

    private static void applyConstraints(MechanismConfig<?> target, MechanismConstraintsConfig constraints) {
        target.constraints(section -> {
            if (constraints.min() != null && constraints.max() != null) {
                section.bounds(constraints.min(), constraints.max());
            }
            if (constraints.motion() != null) {
                MechanismMotionLimitsConfig motion = constraints.motion();
                if (motion.maxVelocity() != null || motion.maxAcceleration() != null) {
                    double maxVel = motion.maxVelocity() != null ? motion.maxVelocity() : 0.0;
                    double maxAccel = motion.maxAcceleration() != null ? motion.maxAcceleration() : 0.0;
                    section.motionLimits(new MotionLimits.AxisLimits(maxVel, maxAccel));
                }
            }
        });
    }

    private static void applySensors(MechanismConfig<?> target, MechanismSensorsConfig sensors) {
        if (sensors == null) {
            return;
        }
        target.sensors(section -> {
            if (sensors.hardwareUpdatePeriodMs() != null && sensors.hardwareUpdatePeriodMs() > 0.0) {
                section.hardwareUpdatePeriodMs(sensors.hardwareUpdatePeriodMs());
            }
            if (sensors.limitSwitches() == null) {
                return;
            }
            for (MechanismLimitSwitchConfig sw : sensors.limitSwitches()) {
                if (sw == null || sw.id() == null) {
                    continue;
                }
                boolean inverted = sw.inverted() != null ? sw.inverted() : false;
                GenericLimitSwitchConfig cfg = new GenericLimitSwitchConfig(
                        sw.id(),
                        inverted,
                        sw.position() != null ? sw.position() : Double.NaN,
                        sw.hardstop() != null ? sw.hardstop() : false,
                        parseBlockDirection(sw.blockDirection()),
                        sw.name(),
                        sw.delaySeconds() != null ? sw.delaySeconds() : 0.0
                );
                section.limitSwitch(cfg);
            }
        });
    }

    private static void applyControl(MechanismConfig<?> target, MechanismControlConfig control) {
        target.control(section -> {
            if (control.output() != null && !control.output().isBlank()) {
                section.output(parseOutput(control.output()));
            }
            if (control.positionSource() != null && !control.positionSource().isBlank()) {
                section.positionSource(control.positionSource());
            }
            if (control.velocitySource() != null && !control.velocitySource().isBlank()) {
                section.velocitySource(control.velocitySource());
            }
            if (control.absoluteSource() != null && !control.absoluteSource().isBlank()) {
                section.absoluteSource(control.absoluteSource());
            }
            if (control.setpointAsOutput() != null) {
                section.setpointAsOutput(control.setpointAsOutput());
            }
            if (control.pidContinuous() != null) {
                if (control.pidContinuous()) {
                    double min = control.pidContinuousMin() != null ? control.pidContinuousMin() : 0.0;
                    double max = control.pidContinuousMax() != null ? control.pidContinuousMax() : 0.0;
                    section.pidContinuous(min, max);
                } else {
                    section.pidContinuousDisabled();
                }
            }
        });

        OutputType pidOutputTypeHint = OutputType.PERCENT;
        if (control.output() != null && !control.output().isBlank()) {
            OutputType parsed = parseOutput(control.output());
            if (parsed == OutputType.VOLTAGE || parsed == OutputType.PERCENT) {
                pidOutputTypeHint = parsed;
            }
        }
        final OutputType pidOutputTypeHintFinal = pidOutputTypeHint;

        // Register PID profiles usable by periodic/custom loops.
        if (control.pidProfiles() != null) {
            for (MechanismPidConfig profile : control.pidProfiles()) {
                if (profile == null || profile.name() == null || profile.name().isBlank()) {
                    continue;
                }
                double kp = profile.kP() != null ? profile.kP() : 0.0;
                double ki = profile.kI() != null ? profile.kI() : 0.0;
                double kd = profile.kD() != null ? profile.kD() : 0.0;
                double iZone = profile.iZone() != null ? profile.iZone() : Double.NaN;
                double maxVelocity = profile.maxVelocity() != null ? profile.maxVelocity() : Double.NaN;
                double maxAcceleration = profile.maxAcceleration() != null ? profile.maxAcceleration() : Double.NaN;
                double tolerance = profile.tolerance() != null
                        ? profile.tolerance()
                        : (control.tolerance() != null ? control.tolerance() : Double.NaN);
                MechanismInputSource source = parseMeasurementInputSource(profile.source());
                MechanismSetpointSource setpointSource = parseSetpointSource(profile.source());
                target.control(c -> c.pid(profile.name(), builder -> {
                    builder.output(pidOutputTypeHintFinal)
                            .kp(kp)
                            .ki(ki)
                            .kd(kd);
                    if (Double.isFinite(iZone)) {
                        builder.iZone(iZone);
                    }
                    if (Double.isFinite(maxVelocity) || Double.isFinite(maxAcceleration)) {
                        builder.profiled(maxVelocity, maxAcceleration);
                    }
                    if (Double.isFinite(tolerance)) {
                        builder.tolerance(tolerance);
                    }
                    if (source != null) {
                        builder.inputSource(source);
                    }
                    if (setpointSource != null) {
                        builder.setpointSource(setpointSource);
                    }
                }));
            }
        }

        OutputType bangBangOutputTypeHint = OutputType.PERCENT;
        if (control.output() != null && !control.output().isBlank()) {
            OutputType parsed = parseOutput(control.output());
            if (parsed == OutputType.VOLTAGE || parsed == OutputType.PERCENT) {
                bangBangOutputTypeHint = parsed;
            }
        }
        final OutputType bangBangOutputTypeHintFinal = bangBangOutputTypeHint;

        if (control.bangBangProfiles() != null) {
            for (MechanismBangBangConfig profile : control.bangBangProfiles()) {
                if (profile == null || profile.name() == null || profile.name().isBlank()) {
                    continue;
                }
                OutputType profileOutput = parseBangBangOutput(profile.output(), bangBangOutputTypeHintFinal);
                double highOutput = profile.highOutput() != null ? profile.highOutput() : 1.0;
                double lowOutput = profile.lowOutput() != null ? profile.lowOutput() : -highOutput;
                double tolerance = profile.tolerance() != null ? profile.tolerance() : 0.0;
                MechanismInputSource source = parseMeasurementInputSource(profile.source());
                MechanismSetpointSource setpointSource = parseSetpointSource(profile.source());
                target.control(c -> c.bangBang(profile.name(), builder -> builder
                        .output(profileOutput)
                        .high(highOutput)
                        .low(lowOutput)
                        .tolerance(tolerance)
                        .inputSource(source)
                        .setpointSource(setpointSource)));
            }
        }

        // Feedforward profiles are used by periodic/custom loops.
        if (control.ffProfiles() != null) {
            for (MechanismFeedforwardConfig ff : control.ffProfiles()) {
                if (ff == null || ff.name() == null || ff.name().isBlank()) {
                    continue;
                }
                MechanismConfig.FeedforwardType type = MechanismConfig.FeedforwardType.fromConfig(ff.type());
                double ks = ff.kS() != null ? ff.kS() : 0.0;
                double kg = ff.kG() != null ? ff.kG() : 0.0;
                double kv = ff.kV() != null ? ff.kV() : 0.0;
                double ka = ff.kA() != null ? ff.kA() : 0.0;
                double tolerance = ff.tolerance() != null
                        ? ff.tolerance()
                        : (control.tolerance() != null ? control.tolerance() : Double.NaN);
                MechanismSetpointSource source = parseSetpointSource(ff.source());
                target.control(c -> c.ff(ff.name(), builder -> {
                    builder.output(OutputType.VOLTAGE);
                    switch (type) {
                        case ARM -> builder.arm(ks, kg, kv, ka);
                        case ELEVATOR -> builder.elevator(ks, kg, kv, ka);
                        case SIMPLE -> builder.simple(ks, kv, ka);
                    }
                    if (Double.isFinite(tolerance)) {
                        builder.tolerance(tolerance);
                    }
                    if (source != null) {
                        builder.setpointSource(source);
                    }
                }));
            }
        }
    }

    private static void applySim(MechanismConfig<?> target, MechanismSimConfig sim) {
        if (sim.simpleMotor() != null) {
            MechanismSimSimpleMotorConfig sm = sim.simpleMotor();
            MechanismConfig.SimpleMotorSimulationParameters params = new MechanismConfig.SimpleMotorSimulationParameters();
            if (sm.momentOfInertia() != null) {
                params.momentOfInertia(sm.momentOfInertia());
            }
            if (sm.nominalVoltage() != null) {
                params.nominalVoltage(sm.nominalVoltage());
            }
            if (sm.unitsPerRadian() != null) {
                params.unitsPerRadian(sm.unitsPerRadian());
            }
            target.sim(s -> s.simpleMotor(params));
        }
        if (sim.arm() != null) {
            MechanismSimArmConfig arm = sim.arm();
            MechanismConfig.ArmSimulationParameters params = new MechanismConfig.ArmSimulationParameters();
            if (arm.armLengthM() != null) {
                params.armLengthMeters(arm.armLengthM());
            }
            if (arm.motorReduction() != null) {
                params.motorReduction(arm.motorReduction());
            }
            if (arm.minDeg() != null && arm.maxDeg() != null) {
                params.angleRangeRadians(Units.degreesToRadians(arm.minDeg()), Units.degreesToRadians(arm.maxDeg()));
            }
            if (arm.startingDeg() != null) {
                params.startingAngleRadians(Units.degreesToRadians(arm.startingDeg()));
            }
            if (arm.unitsPerRadian() != null) {
                params.unitsPerRadian(arm.unitsPerRadian());
            }
            if (arm.simulateGravity() != null) {
                params.simulateGravity(arm.simulateGravity());
            }
            if (arm.nominalVoltage() != null) {
                params.nominalVoltage(arm.nominalVoltage());
            }
            if (arm.momentOfInertia() != null) {
                params.momentOfInertia(arm.momentOfInertia());
            }
            target.sim(s -> s.arm(params));
        }
        if (sim.elevator() != null) {
            MechanismSimElevatorConfig elev = sim.elevator();
            MechanismConfig.ElevatorSimulationParameters params = new MechanismConfig.ElevatorSimulationParameters();
            if (elev.drumRadiusM() != null) {
                params.drumRadiusMeters(elev.drumRadiusM());
            }
            if (elev.carriageMassKg() != null) {
                params.carriageMassKg(elev.carriageMassKg());
            }
            if (elev.minHeightM() != null && elev.maxHeightM() != null) {
                params.rangeMeters(elev.minHeightM(), elev.maxHeightM());
            }
            if (elev.startingHeightM() != null) {
                params.startingHeightMeters(elev.startingHeightM());
            }
            if (elev.simulateGravity() != null) {
                params.simulateGravity(elev.simulateGravity());
            }
            if (elev.nominalVoltage() != null) {
                params.nominalVoltage(elev.nominalVoltage());
            }
            if (elev.unitsPerMeter() != null) {
                params.unitsPerMeter(elev.unitsPerMeter());
            }
            target.sim(s -> s.elevator(params));
        }
    }

    private static MotorNeutralMode parseNeutralMode(String value) {
        String v = value == null ? "" : value.trim().toUpperCase(Locale.ROOT);
        return switch (v) {
            case "BRAKE" -> MotorNeutralMode.Brake;
            case "COAST" -> MotorNeutralMode.Coast;
            default -> throw new IllegalArgumentException("Unknown motor neutral mode: " + value);
        };
    }

    private static OutputType parseOutput(String value) {
        String v = value.trim().toUpperCase(Locale.ROOT);
        return OutputType.valueOf(v);
    }

    private static MechanismEncoderUnit parseEncoderUnit(String value) {
        String v = value == null ? "" : value.trim().toLowerCase(Locale.ROOT);
        return switch (v) {
            case "", "encoder_units", "encoder-units", "encoderunits" -> MechanismEncoderUnit.ENCODER_UNITS;
            case "rotations" -> MechanismEncoderUnit.ROTATIONS;
            case "radians" -> MechanismEncoderUnit.RADIANS;
            case "degrees" -> MechanismEncoderUnit.DEGREES;
            default -> throw new IllegalArgumentException("Unknown encoder unit: " + value);
        };
    }

    private static MechanismConfig.InputSource parseDerivedInputSource(
            String value,
            MechanismConfig.InputSource fallback) {
        if (value == null || value.isBlank()) {
            return fallback;
        }
        return switch (value.trim().toLowerCase(Locale.ROOT)) {
            case "position" -> MechanismConfig.InputSource.Position;
            case "velocity" -> MechanismConfig.InputSource.Velocity;
            case "absolute" -> MechanismConfig.InputSource.Absolute;
            default -> throw new IllegalArgumentException("Unknown encoder input signal: " + value);
        };
    }

    private static OutputType parseBangBangOutput(String value, OutputType fallback) {
        OutputType parsed = fallback != null ? fallback : OutputType.PERCENT;
        if (value != null && !value.isBlank()) {
            parsed = parseOutput(value);
        }
        if (parsed != OutputType.PERCENT && parsed != OutputType.VOLTAGE) {
            throw new IllegalArgumentException("Bang-bang profile output must be PERCENT or VOLTAGE");
        }
        return parsed;
    }

    private static MechanismInputSource parseMeasurementInputSource(String value) {
        if (value == null || value.isBlank()) {
            return null;
        }
        String trimmed = value.trim();
        String normalized = trimmed.toLowerCase(Locale.ROOT);
        return switch (normalized) {
            case "position" -> MechanismInputSource.Position;
            case "velocity" -> MechanismInputSource.Velocity;
            case "absolute" -> MechanismInputSource.Absolute;
            default -> {
                if (normalized.startsWith("position:")) {
                    yield MechanismInputSource.position(trimmed.substring(trimmed.indexOf(':') + 1).trim());
                }
                if (normalized.startsWith("velocity:")) {
                    yield MechanismInputSource.velocity(trimmed.substring(trimmed.indexOf(':') + 1).trim());
                }
                if (normalized.startsWith("absolute:")) {
                    yield MechanismInputSource.absolute(trimmed.substring(trimmed.indexOf(':') + 1).trim());
                }
                if (normalized.startsWith("input:") || normalized.startsWith("inputs:")) {
                    int separator = trimmed.indexOf(':');
                    String key = separator >= 0 ? trimmed.substring(separator + 1).trim() : "";
                    if (key.isBlank()) {
                        throw new IllegalArgumentException("Input source key cannot be blank: " + value);
                    }
                    yield MechanismInputSource.input(key);
                }
                if ("setpoint".equals(normalized)) {
                    yield null;
                }
                throw new IllegalArgumentException("Unknown control input source: " + value);
            }
        };
    }

    private static MechanismSetpointSource parseSetpointSource(String value) {
        if (value == null || value.isBlank()) {
            return null;
        }
        String trimmed = value.trim();
        String normalized = trimmed.toLowerCase(Locale.ROOT);
        return switch (normalized) {
            case "setpoint" -> MechanismSetpointSource.Setpoint;
            default -> {
                if (normalized.startsWith("input:") || normalized.startsWith("inputs:")) {
                    int separator = trimmed.indexOf(':');
                    String key = separator >= 0 ? trimmed.substring(separator + 1).trim() : "";
                    if (key.isBlank()) {
                        throw new IllegalArgumentException("Setpoint source key cannot be blank: " + value);
                    }
                    yield MechanismSetpointSource.input(key);
                }
                yield null;
            }
        };
    }

    private static BlockDirection parseBlockDirection(String value) {
        if (value == null || value.isBlank()) {
            return BlockDirection.None;
        }
        String v = value.trim().toUpperCase(Locale.ROOT);
        return switch (v) {
            case "POSITIVE", "POSITIVE_INPUT", "PLUS" -> BlockDirection.PositiveInput;
            case "NEGATIVE", "NEGATIVE_INPUT", "MINUS" -> BlockDirection.NegativeInput;
            case "NONE" -> BlockDirection.None;
            default -> BlockDirection.None;
        };
    }
}
