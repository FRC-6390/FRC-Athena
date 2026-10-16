package frc.robot;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.commands.WpilibCommands;
import ca.frc6390.athena.wpilib.controls.ClickSequence;
import ca.frc6390.athena.wpilib.controls.ControlSignal;
import ca.frc6390.athena.wpilib.controls.ControlSignals;
import ca.frc6390.athena.wpilib.controls.Controllers;
import ca.frc6390.athena.wpilib.controls.Gamepad;
import ca.frc6390.athena.wpilib.controls.ToggleSignal;
import edu.wpi.first.wpilibj2.command.Commands;
import java.time.Duration;

/** Gesture, timing, chord, sequence, threshold, and command-interoperability examples. */
public final class AdvancedControls implements Mechanism {
    public final Gamepad operator = Controllers.xbox(Constants.Operator.PORT);
    private double trim;
    private int diagnosticCaptures;
    private boolean serviceMode;

    public final Action wpilibCommand = WpilibCommands.action(
            Commands.runOnce(() -> System.out.println("WPILib command ran through Athena")));
    public final Action diagnosticCommand = Actions.lifecycle("capture-diagnostics")
            .onStart(() -> diagnosticCaptures++)
            .until(() -> true)
            .requires("diagnostics")
            .build();

    public AdvancedControls(Robot robot) {
        ClickSequence clicks = operator.a().debounce(Duration.ofMillis(20))
                .clicks(Duration.ofMillis(300));
        clicks.exactly(1).onTrue(robot.intake.collect);
        clicks.exactly(2).onTrue(robot.intake.eject);
        clicks.exactly(3).onTrue(robot.intake.stop);
        clicks.between(4, 6).onTrue(() -> robot.athena().request(diagnosticCommand));
        clicks.atLeast(7).onTrue(() -> diagnosticCaptures = 0);

        operator.b().heldFor(Duration.ofMillis(500))
                .whileTrue(robot.intake.collect)
                .onFalse(robot.intake.stop);
        operator.x().shortPress(Duration.ofMillis(250)).onTrue(robot.driveTrain.stop);
        operator.y().repeated(Duration.ofMillis(400), Duration.ofMillis(100))
                .onTrue(() -> trim = Math.min(1.0, trim + 0.01));
        operator.start().holdStarted(Duration.ofSeconds(1)).onTrue(() -> serviceMode = true);

        ToggleSignal enabled = operator.rightStick().toggle(false)
                .setWhen(operator.x().pressed())
                .clearWhen(operator.b().pressed())
                .resetWhen(operator.back().pressed());
        enabled.whileTrue(robot.intake.collect).onFalse(robot.intake.stop);

        ControlSignal shoot = operator.rightTrigger().above(0.55, 0.45);
        ControlSignal safe = operator.leftTrigger().inside(0.20);
        ControlSignals.allOf(shoot, safe).whileTrue(robot.intake.eject);
        ControlSignals.anyOf(operator.leftBumper(), operator.rightBumper())
                .onTrue(robot.driveTrain.stop);
        ControlSignals.noneOf(operator.leftBumper(), operator.rightBumper())
                .onTrue(() -> serviceMode = false);

        // Binding set: these bindings exist only while service mode is on. Outside service mode the POV buttons do
        // nothing here, and bindings made directly on the operator controller are never gated by the set.
        operator.in(() -> serviceMode, pad -> {
            pad.povLeft().whileTrue(robot.intake.collect).onFalse(robot.intake.stop);
            pad.povRight().whileTrue(robot.intake.eject).onFalse(robot.intake.stop);
        });

        ControlSignals.chord(operator.back(), operator.start())
                .within(Duration.ofMillis(200))
                .onTrue(() -> robot.athena().request(wpilibCommand));
        ControlSignals.sequence(
                operator.povUp().pressed(),
                operator.povUp().released(),
                operator.povDown().pressed())
                .within(Duration.ofSeconds(2))
                .onTrue(() -> serviceMode = !serviceMode);
    }
}
