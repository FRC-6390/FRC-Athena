package ca.frc6390.athena.examples;

import ca.frc6390.athena.mechanism.config.MechanismConfig;
import ca.frc6390.athena.mechanism.config.Mechanisms;

/**
 * Flywheel-style example demonstrating velocity control and integrated encoder
 * requirements.
 */
public final class ShooterExample {
    /**
     * Shooter mechanism declaration.
     */
    public static final MechanismConfig CONFIG = Mechanisms.flywheel("shooter")
            .motor("leader", motor -> motor
                    .hardware(RobotHardware.SHOOTER_LEADER)
                    .coast()
                    .currentLimit(60)
                    .integratedEncoder())
            // Named encoders make the control source explicit and testable.
            .encoder("flywheelEncoder", encoder -> encoder
                    .hardware(RobotHardware.SHOOTER_ENCODER)
                    .velocity()
                    .gearRatio(1.0))
            .velocitySource("flywheelEncoder")
            .input("targetRpm", input -> input.runtimeNumber("dashboard/shooterTargetRpm"))
            .control(control -> control
                    .velocity(pid -> pid
                            .p(0.14)
                            .i(0.0)
                            .d(0.001))
                    .feedforward(ff -> ff
                            .staticGain(0.18)
                            .velocity(0.12)))
            .state("idle", state -> state.target(0.0))
            .state("speaker", state -> state.target(4600.0))
            .state("amp", state -> state.target(1800.0));

    private ShooterExample() {
    }
}
