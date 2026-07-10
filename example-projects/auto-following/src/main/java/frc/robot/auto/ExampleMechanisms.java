package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Replace these print-only commands with the real superstructure command factories. */
public final class ExampleMechanisms extends SubsystemBase {
    private final ExampleRobotState state;

    public ExampleMechanisms(ExampleRobotState state) {
        this.state = state;
    }

    public Command prepareToScore() {
        return Commands.runOnce(() -> System.out.println("prepare score"), this)
                .withName("PrepareToScore");
    }

    public Command score() {
        return Commands.sequence(
                        Commands.runOnce(() -> System.out.println("score"), this),
                        Commands.waitSeconds(0.20),
                        Commands.runOnce(() -> state.setHasGamePiece(false), this))
                .withName("Score");
    }

    public Command intakeUntilCaptured() {
        return Commands.startEnd(
                        () -> System.out.println("intake on"),
                        () -> System.out.println("intake off"),
                        this)
                .until(state::hasGamePiece)
                .withTimeout(1.5)
                .withName("IntakeUntilCaptured");
    }

    public Command stow() {
        return Commands.runOnce(() -> System.out.println("stow"), this).withName("Stow");
    }
}
