package ca.frc6390.athena.examples;

import ca.frc6390.athena.mechanism.config.MechanismConfig;
import ca.frc6390.athena.mechanism.config.Mechanisms;

/**
 * Minimal mechanism example using the V3 fluent API.
 */
public final class IntakeExample {
    /**
     * Intake mechanism declaration.
     */
    public static final MechanismConfig CONFIG = Mechanisms.simple("intake")
            .motor("roller", motor -> motor
                    .hardware(RobotHardware.INTAKE_ROLLER)
                    .brake()
                    .currentLimit(35))
            .input("beamBreak", input -> input.digital(0))
            .control(control -> control.percentOutput())
            .state("stopped", state -> state.target(0.0))
            .state("feed", state -> state.target(0.65));

    private IntakeExample() {
    }
}
