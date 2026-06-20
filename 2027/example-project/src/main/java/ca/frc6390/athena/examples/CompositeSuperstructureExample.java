package ca.frc6390.athena.examples;

import ca.frc6390.athena.commands.CommandSpec;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.mechanism.runtime.MechanismController;
import ca.frc6390.athena.superstructure.runtime.SuperstructureCommands;
import ca.frc6390.athena.superstructure.runtime.SuperstructureController;
import ca.frc6390.athena.superstructure.config.SuperstructureConfig;
import ca.frc6390.athena.superstructure.config.Superstructures;
import ca.frc6390.athena.superstructure.runtime.SuperstructurePlanner;
import ca.frc6390.athena.superstructure.runtime.SuperstructureTransitionPlan;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Nested superstructure example for robots that split scoring into assemblies.
 */
public final class CompositeSuperstructureExample {
    /**
     * Assembly that owns the intake-to-shooter handoff sequence.
     */
    public static final SuperstructureConfig HANDOFF = Superstructures.create("handoff")
            .part("intake", IntakeExample.CONFIG)
            .part("shooter", ShooterExample.CONFIG)
            .state("stowed", state -> state
                    .part("intake", "stopped")
                    .part("shooter", "idle"))
            .state("feeding", state -> state
                    .part("intake", "feed")
                    .part("shooter", "speaker"));

    /**
     * Top-level robot superstructure that targets a nested assembly state.
     */
    public static final SuperstructureConfig ROBOT = Superstructures.create("robotComposite")
            .part("handoff", HANDOFF)
            .state("idle", state -> state
                    .part("handoff", "stowed"))
            .state("scoreSpeaker", state -> state
                    .part("handoff", "feeding"));

    /**
     * Plans the nested mechanism targets needed to score from the idle state.
     *
     * @return resolved transition plan
     */
    public static SuperstructureTransitionPlan scoreSpeakerPlan() {
        return new SuperstructurePlanner(ROBOT.toSpec()).plan("idle", "scoreSpeaker");
    }

    /**
     * Applies the score state through mechanism runtime controllers.
     *
     * @return applied targets by leaf mechanism path
     */
    public static Map<String, Double> applyScoreSpeaker() {
        Map<String, Double> targets = new LinkedHashMap<>();
        SuperstructureController controller = createController(targets);

        controller.applyState("scoreSpeaker");

        return targets;
    }

    /**
     * Creates a command that applies the score-speaker state.
     *
     * @param targets applied targets by leaf mechanism path
     * @return command spec
     */
    public static CommandSpec scoreSpeakerCommand(Map<String, Double> targets) {
        return SuperstructureCommands.applyState(createController(targets), "scoreSpeaker", "superstructure");
    }

    private CompositeSuperstructureExample() {
    }

    private static SuperstructureController createController(Map<String, Double> targets) {
        var robot = ROBOT.toSpec();
        var handoff = HANDOFF.toSpec();
        var intake = handoff.parts().get(0).mechanism();
        var shooter = handoff.parts().get(1).mechanism();
        var intakeMotor = new RecordingMotor(intake.motors().get(0), targets, "handoff.intake");
        var shooterMotor = new RecordingMotor(shooter.motors().get(0), targets, "handoff.shooter");

        return SuperstructureController.builder(robot)
                .mechanism("handoff.intake", MechanismController.of(intake, List.of(intakeMotor)))
                .mechanism("handoff.shooter", MechanismController.of(shooter, List.of(shooterMotor)))
                .build();
    }

    private record RecordingMotor(MotorSpec spec, Map<String, Double> targets, String path) implements MotorDevice {
        @Override
        public void setPercentOutput(double percent) {
            targets.put(path, percent);
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            targets.put(path, rotationsPerSecond);
        }
    }
}
