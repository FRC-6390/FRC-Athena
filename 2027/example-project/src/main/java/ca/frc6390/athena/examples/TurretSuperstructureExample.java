package ca.frc6390.athena.examples;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.mechanism.config.MechanismConfig;
import ca.frc6390.athena.mechanism.config.Mechanisms;
import ca.frc6390.athena.superstructure.config.SuperstructureConfig;
import ca.frc6390.athena.superstructure.config.Superstructures;
import ca.frc6390.athena.superstructure.runtime.SuperstructurePlanner;
import ca.frc6390.athena.superstructure.runtime.SuperstructureTransitionGuard;
import ca.frc6390.athena.superstructure.runtime.SuperstructureTransitionPlan;

/**
 * Turret assembly example with separate turret, hood, and shooter targets.
 */
public final class TurretSuperstructureExample {
    /**
     * Position-controlled turret mechanism.
     */
    public static final MechanismConfig TURRET = Mechanisms.simple("turret")
            .motor("rotate", motor -> motor
                    .hardware(AthenaMotor.SIM, 61)
                    .brake()
                    .currentLimit(30))
            .encoder("azimuth", encoder -> encoder
                    .hardware(AthenaEncoder.SIM, 61)
                    .absolutePosition()
                    .offset(0.0))
            .positionSource("azimuth")
            .control(control -> control
                    .position(pid -> pid
                            .p(0.09)
                            .d(0.002)))
            .state("stowed", state -> state.target(0.0))
            .state("speaker", state -> state.target(28.0))
            .state("amp", state -> state.target(-42.0));

    /**
     * Position-controlled hood mechanism.
     */
    public static final MechanismConfig HOOD = Mechanisms.simple("hood")
            .motor("tilt", motor -> motor
                    .hardware(AthenaMotor.SIM, 62)
                    .brake()
                    .currentLimit(25))
            .encoder("angle", encoder -> encoder
                    .hardware(AthenaEncoder.SIM, 62)
                    .absolutePosition())
            .positionSource("angle")
            .control(control -> control
                    .position(pid -> pid
                            .p(0.07)
                            .d(0.001)))
            .state("stowed", state -> state.target(8.0))
            .state("speaker", state -> state.target(34.0))
            .state("amp", state -> state.target(61.0));

    /**
     * Turret assembly declaration that coordinates turret, hood, and shooter.
     */
    public static final SuperstructureConfig ASSEMBLY = Superstructures.create("turretAssembly")
            .part("turret", TURRET)
            .part("hood", HOOD)
            .part("shooter", ShooterExample.CONFIG)
            .state("stowed", state -> state
                    .part("turret", "stowed")
                    .part("hood", "stowed")
                    .part("shooter", "idle"))
            .state("speaker", state -> state
                    .part("turret", "speaker")
                    .part("hood", "speaker")
                    .part("shooter", "speaker"))
            .state("amp", state -> state
                    .part("turret", "amp")
                    .part("hood", "amp")
                    .part("shooter", "amp"));

    /**
     * Plans turret scoring targets while preserving a safety interlock hook.
     *
     * @param guard transition guard supplied by robot code
     * @return resolved transition plan
     */
    public static SuperstructureTransitionPlan speakerPlan(SuperstructureTransitionGuard guard) {
        return new SuperstructurePlanner(ASSEMBLY.toSpec(), guard).plan("stowed", "speaker");
    }

    private TurretSuperstructureExample() {
    }
}
