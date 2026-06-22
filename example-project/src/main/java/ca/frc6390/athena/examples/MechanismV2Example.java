package ca.frc6390.athena.examples;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.mechanism.config.MechanismConfig;
import ca.frc6390.athena.mechanism.config.Mechanisms;
import ca.frc6390.athena.mechanism.runtime.MechanismController;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Fluent equivalents for the legacy v2 mechanism authoring examples.
 */
public final class MechanismV2Example {
    private MechanismV2Example() {
    }

    /**
     * Simple roller mechanism using percent output.
     *
     * @return mechanism config
     */
    public static MechanismConfig simpleRoller() {
        return Mechanisms.simple("simpleRoller")
                .motor("roller", motor -> motor
                        .hardware(AthenaMotor.SIM, 10)
                        .brake()
                        .currentLimit(30))
                .control(control -> control.percentOutput())
                .state("off", state -> state.target(0.0))
                .state("intake", state -> state.target(0.65))
                .state("reverse", state -> state.target(-0.35));
    }

    /**
     * Angular arm mechanism with absolute encoder and home switch.
     *
     * @return mechanism config
     */
    public static MechanismConfig armPivot() {
        return Mechanisms.simple("armPivot")
                .motor("armMotor", motor -> motor
                        .hardware(AthenaMotor.SIM, 30)
                        .brake()
                        .currentLimit(45))
                .encoder("armAbsolute", encoder -> encoder
                        .hardware(AthenaEncoder.SIM, 31)
                        .absolutePosition()
                        .offset(0.0))
                .positionSource("armAbsolute")
                .input("homeSwitch", input -> input.digital(4))
                .control(control -> control
                        .position(pid -> pid
                                .p(0.16)
                                .i(0.0)
                                .d(0.0))
                        .feedforward(ff -> ff.gravity(0.08)))
                .state("home", state -> state.target(0.0))
                .state("intake", state -> state.target(18.0))
                .state("score", state -> state.target(72.0));
    }

    /**
     * Linear elevator mechanism with bottom switch and measured-height input.
     *
     * @return mechanism config
     */
    public static MechanismConfig elevator() {
        return Mechanisms.simple("elevator")
                .motor("elevatorMotor", motor -> motor
                        .hardware(AthenaMotor.SIM, 40)
                        .brake()
                        .currentLimit(50))
                .encoder("carriageEncoder", encoder -> encoder
                        .hardware(AthenaEncoder.SIM, 41)
                        .relativePosition()
                        .gearRatio(1.0))
                .positionSource("carriageEncoder")
                .input("bottomHome", input -> input.digital(5))
                .input("measuredHeightMeters", input -> input.runtimeNumber("elevator/heightMeters"))
                .control(control -> control
                        .position(pid -> pid
                                .p(1.4)
                                .i(0.0)
                                .d(0.04))
                        .feedforward(ff -> ff
                                .staticGain(0.12)
                                .gravity(0.18)))
                .state("home", state -> state.target(0.0))
                .state("mid", state -> state.target(0.65))
                .state("high", state -> state.target(1.25));
    }

    /**
     * Flywheel mechanism with velocity source and dashboard setpoint input.
     *
     * @return mechanism config
     */
    public static MechanismConfig flywheel() {
        return Mechanisms.flywheel("shooterFlywheel")
                .motor("shooterMotor", motor -> motor
                        .hardware(AthenaMotor.SIM, 20)
                        .coast()
                        .currentLimit(60)
                        .integratedEncoder())
                .encoder("flywheelEncoder", encoder -> encoder
                        .hardware(AthenaEncoder.SIM, 21)
                        .velocity())
                .velocitySource("flywheelEncoder")
                .input("measuredVelocityRadPerSec", input -> input.runtimeNumber("shooter/velocity"))
                .control(control -> control
                        .velocity(pid -> pid
                                .p(0.11)
                                .i(0.0)
                                .d(0.001))
                        .feedforward(ff -> ff
                                .staticGain(0.18)
                                .velocity(0.12)))
                .state("off", state -> state.target(0.0))
                .state("spinup", state -> state.target(366.5))
                .state("fire", state -> state.target(544.5));
    }

    /**
     * Turret mechanism with angular range style states and runtime targeting inputs.
     *
     * @return mechanism config
     */
    public static MechanismConfig turret() {
        return Mechanisms.simple("turret")
                .motor("turretMotor", motor -> motor
                        .hardware(AthenaMotor.SIM, 50)
                        .brake()
                        .currentLimit(35))
                .encoder("turretAbsolute", encoder -> encoder
                        .hardware(AthenaEncoder.SIM, 51)
                        .absolutePosition())
                .positionSource("turretAbsolute")
                .input("turretHeadingDeg", input -> input.runtimeNumber("turret/headingDeg"))
                .input("targetVisible", input -> input.runtimeBoolean("turret/targetVisible"))
                .control(control -> control
                        .position(pid -> pid
                                .p(0.08)
                                .i(0.0)
                                .d(0.002)))
                .state("off", state -> state.target(0.0))
                .state("safe", state -> state.target(0.0))
                .state("aim", state -> state.target(24.0));
    }

    /**
     * Returns all v2-style fluent mechanism declarations.
     *
     * @return mechanism configs
     */
    public static List<MechanismConfig> all() {
        return List.of(simpleRoller(), armPivot(), elevator(), flywheel(), turret());
    }

    /**
     * Applies named mechanism states through the runtime motor boundary.
     *
     * @return recorded state targets by mechanism
     */
    public static Map<String, Double> runtimeStateTargets() {
        Map<String, Double> targets = new LinkedHashMap<>();
        var roller = simpleRoller().toSpec();
        var arm = armPivot().toSpec();
        var flywheel = flywheel().toSpec();
        var rollerMotor = new RecordingMotor(roller.motors().get(0), targets, "roller");
        var armMotor = new RecordingMotor(arm.motors().get(0), targets, "arm");
        var flywheelMotor = new RecordingMotor(flywheel.motors().get(0), targets, "flywheel");

        MechanismController.of(roller, List.of(rollerMotor)).applyState("intake");
        MechanismController.of(arm, List.of(armMotor)).applyState("score");
        MechanismController.of(flywheel, List.of(flywheelMotor)).applyState("fire");

        return targets;
    }

    private record RecordingMotor(MotorSpec spec, Map<String, Double> targets, String key) implements MotorDevice {
        @Override
        public void setPercentOutput(double percent) {
            targets.put(key, percent);
        }

        @Override
        public void setPositionTargetRotations(double rotations) {
            targets.put(key, rotations);
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            targets.put(key, rotationsPerSecond);
        }
    }
}
