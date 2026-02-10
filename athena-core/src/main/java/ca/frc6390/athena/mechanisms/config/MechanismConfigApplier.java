package ca.frc6390.athena.mechanisms.config;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorRegistry;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.BlockDirection;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import edu.wpi.first.math.util.Units;
import java.util.Locale;
import java.util.Objects;

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
        if (file.encoder() != null) {
            applyEncoder(target, file.encoder());
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

    private static void applyEncoder(MechanismConfig<?> target, MechanismEncoderConfig enc) {
        String source = enc.source() != null ? enc.source().trim().toLowerCase(Locale.ROOT) : "";
        target.encoder(section -> {
            if (source.equals("motor") || source.equals("integrated")) {
                if (enc.motorId() != null) {
                    section.fromMotor(enc.motorId());
                }
            } else if (source.equals("cancoder")) {
                if (enc.id() != null) {
                    EncoderConfig cfg = EncoderConfig.type(AthenaEncoder.CANCODER.resolve(), enc.id());
                    applyEncoderNumbers(cfg, enc);
                    section.config(cfg);
                }
            } else if (!source.isBlank() && !source.equals("custom")) {
                // Best-effort: accept AthenaEncoder enum names in source.
                try {
                    AthenaEncoder ae = AthenaEncoder.valueOf(source.toUpperCase(Locale.ROOT));
                    if (enc.id() != null) {
                        EncoderConfig cfg = EncoderConfig.type(ae.resolve(), enc.id());
                        applyEncoderNumbers(cfg, enc);
                        section.config(cfg);
                    }
                } catch (IllegalArgumentException ignored) {
                    // If not an AthenaEncoder enum, try treating it as a registry key (EncoderType).
                    try {
                        var et = ca.frc6390.athena.hardware.encoder.EncoderRegistry.get().encoder(source);
                        if (enc.id() != null) {
                            EncoderConfig cfg = EncoderConfig.type(et, enc.id());
                            applyEncoderNumbers(cfg, enc);
                            section.config(cfg);
                        }
                    } catch (Exception ignored2) {
                        // Leave unresolved. Custom sensors remain code-only.
                    }
                }
            }

            if (enc.absolute() != null) {
                section.absolute(enc.absolute());
            }
        });
    }

    private static void applyEncoderNumbers(EncoderConfig cfg, MechanismEncoderConfig enc) {
        if (cfg == null || enc == null) {
            return;
        }
        if (enc.gearRatio() != null) {
            cfg.setGearRatio(enc.gearRatio());
        }
        if (enc.conversion() != null) {
            cfg.setConversion(enc.conversion());
        }
        if (enc.conversionOffset() != null) {
            cfg.setConversionOffset(enc.conversionOffset());
        }
        if (enc.offset() != null) {
            cfg.setOffset(enc.offset());
        }
        if (enc.discontinuityPoint() != null && enc.discontinuityRange() != null) {
            cfg.setDiscontinuity(enc.discontinuityPoint(), enc.discontinuityRange());
        } else if (enc.discontinuityPoint() != null) {
            cfg.setDiscontinuityPoint(enc.discontinuityPoint());
        } else if (enc.discontinuityRange() != null) {
            cfg.setDiscontinuityRange(enc.discontinuityRange());
        }
        if (enc.inverted() != null) {
            cfg.setInverted(enc.inverted());
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
        if (sensors.limitSwitches() == null) {
            return;
        }
        target.sensors(section -> {
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
            if (control.setpointAsOutput() != null) {
                section.setpointAsOutput(control.setpointAsOutput());
            }
            if (control.pidUseVelocity() != null) {
                section.pidUseVelocity(control.pidUseVelocity());
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

        // Register PID profiles (usable by custom loops) and optionally select a default.
        if (control.pidProfiles() != null) {
            for (MechanismPidConfig profile : control.pidProfiles()) {
                if (profile == null || profile.name() == null || profile.name().isBlank()) {
                    continue;
                }
                double kp = profile.kP() != null ? profile.kP() : 0.0;
                double ki = profile.kI() != null ? profile.kI() : 0.0;
                double kd = profile.kD() != null ? profile.kD() : 0.0;
                double iZone = profile.iZone() != null ? profile.iZone() : Double.NaN;
                double tolerance = profile.tolerance() != null
                        ? profile.tolerance()
                        : (control.tolerance() != null ? control.tolerance() : Double.NaN);
                target.control(c -> c.pidProfile(profile.name(), pidOutputTypeHintFinal, kp, ki, kd, iZone, tolerance));
            }
        }

        // Main PID selection from named profiles.
        if (control.pidProfile() != null && !control.pidProfile().isBlank()) {
            String selected = control.pidProfile().trim();
            target.control(c -> c.mainPid(selected));
            if (control.pidProfiles() != null) {
                for (MechanismPidConfig profile : control.pidProfiles()) {
                    if (profile == null || profile.name() == null) {
                        continue;
                    }
                    if (!profile.name().trim().equals(selected)) {
                        continue;
                    }
                    if (profile.period() != null) {
                        target.control(c -> c.pidPeriod(profile.period()));
                    }
                    break;
                }
            }
        }

        // Feedforward profiles are used by custom loops (simple motor) and can optionally be selected
        // as a mechanism's main feedforward for mechanisms that support built-in FF.
        if (control.ffProfiles() != null) {
            for (MechanismFeedforwardConfig ff : control.ffProfiles()) {
                if (ff == null || ff.name() == null || ff.name().isBlank()) {
                    continue;
                }
                String type = ff.type() != null ? ff.type().trim().toLowerCase(Locale.ROOT) : "simple_motor";
                double ks = ff.kS() != null ? ff.kS() : 0.0;
                double kg = ff.kG() != null ? ff.kG() : 0.0;
                double kv = ff.kV() != null ? ff.kV() : 0.0;
                double ka = ff.kA() != null ? ff.kA() : 0.0;
                switch (type) {
                    case "arm" -> target.control(c -> c.armFeedforwardProfile(ff.name(), ks, kg, kv, ka));
                    case "elevator" -> target.control(c -> c.elevatorFeedforwardProfile(ff.name(), ks, kg, kv, ka));
                    case "simple_motor" -> target.control(c -> c.feedforwardProfile(ff.name(), OutputType.VOLTAGE, ks, kv, ka));
                    default -> target.control(c -> c.feedforwardProfile(ff.name(), OutputType.VOLTAGE, ks, kv, ka));
                }
            }
        }

        // Optional: select a main feedforward profile for mechanisms with built-in FF.
        if (control.ffProfile() != null && !control.ffProfile().isBlank() && control.ffProfiles() != null) {
            String selected = control.ffProfile().trim();
            for (MechanismFeedforwardConfig ff : control.ffProfiles()) {
                if (ff == null || ff.name() == null) {
                    continue;
                }
                if (!ff.name().trim().equals(selected)) {
                    continue;
                }
                String type = ff.type() != null ? ff.type().trim().toLowerCase(Locale.ROOT) : "simple_motor";
                switch (type) {
                    case "arm" -> target.control(c -> c.mainArmFeedforward(selected));
                    case "elevator" -> target.control(c -> c.mainElevatorFeedforward(selected));
                    case "simple_motor" -> target.control(c -> c.mainSimpleFeedforward(selected));
                    default -> target.control(c -> c.mainSimpleFeedforward(selected));
                }
                break;
            }
        }
    }

    private static void applySim(MechanismConfig<?> target, MechanismSimConfig sim) {
        if (sim.simpleMotor() != null) {
            MechanismSimSimpleMotorConfig sm = sim.simpleMotor();
            MechanismConfig.SimpleMotorSimulationParameters params = new MechanismConfig.SimpleMotorSimulationParameters();
            if (sm.momentOfInertia() != null) {
                params.setMomentOfInertia(sm.momentOfInertia());
            }
            if (sm.nominalVoltage() != null) {
                params.setNominalVoltage(sm.nominalVoltage());
            }
            if (sm.unitsPerRadian() != null) {
                params.setUnitsPerRadian(sm.unitsPerRadian());
            }
            target.sim(s -> s.simpleMotor(params));
        }
        if (sim.arm() != null) {
            MechanismSimArmConfig arm = sim.arm();
            MechanismConfig.ArmSimulationParameters params = new MechanismConfig.ArmSimulationParameters();
            if (arm.armLengthM() != null) {
                params.setArmLengthMeters(arm.armLengthM());
            }
            if (arm.motorReduction() != null) {
                params.setMotorReduction(arm.motorReduction());
            }
            if (arm.minDeg() != null && arm.maxDeg() != null) {
                params.setAngleRangeRadians(Units.degreesToRadians(arm.minDeg()), Units.degreesToRadians(arm.maxDeg()));
            }
            if (arm.startingDeg() != null) {
                params.setStartingAngleRadians(Units.degreesToRadians(arm.startingDeg()));
            }
            if (arm.unitsPerRadian() != null) {
                params.setUnitsPerRadian(arm.unitsPerRadian());
            }
            if (arm.simulateGravity() != null) {
                params.setSimulateGravity(arm.simulateGravity());
            }
            if (arm.nominalVoltage() != null) {
                params.setNominalVoltage(arm.nominalVoltage());
            }
            if (arm.momentOfInertia() != null) {
                params.setMomentOfInertia(arm.momentOfInertia());
            }
            target.sim(s -> s.arm(params));
        }
        if (sim.elevator() != null) {
            MechanismSimElevatorConfig elev = sim.elevator();
            MechanismConfig.ElevatorSimulationParameters params = new MechanismConfig.ElevatorSimulationParameters();
            if (elev.drumRadiusM() != null) {
                params.setDrumRadiusMeters(elev.drumRadiusM());
            }
            if (elev.carriageMassKg() != null) {
                params.setCarriageMassKg(elev.carriageMassKg());
            }
            if (elev.minHeightM() != null && elev.maxHeightM() != null) {
                params.setRangeMeters(elev.minHeightM(), elev.maxHeightM());
            }
            if (elev.startingHeightM() != null) {
                params.setStartingHeightMeters(elev.startingHeightM());
            }
            if (elev.simulateGravity() != null) {
                params.setSimulateGravity(elev.simulateGravity());
            }
            if (elev.nominalVoltage() != null) {
                params.setNominalVoltage(elev.nominalVoltage());
            }
            if (elev.unitsPerMeter() != null) {
                params.setUnitsPerMeter(elev.unitsPerMeter());
            }
            target.sim(s -> s.elevator(params));
        }
    }

    private static MotorNeutralMode parseNeutralMode(String value) {
        String v = value.trim().toUpperCase(Locale.ROOT);
        return MotorNeutralMode.valueOf(v);
    }

    private static OutputType parseOutput(String value) {
        String v = value.trim().toUpperCase(Locale.ROOT);
        return OutputType.valueOf(v);
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
