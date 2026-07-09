package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.runtime.control.ModifiedAxis;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * WPILib controller and command adapters.
 */
public final class Controls {
    private Controls() {
    }

    public static Controller xbox(int port) {
        return new Controller(new XboxController(port));
    }

    public static CommandAction run(Runnable action) {
        return run("run", action);
    }

    public static CommandAction run(String name, Runnable action) {
        Objects.requireNonNull(action, "action");
        return CommandAction.create(name)
                .onInitialize(action::run)
                .until(() -> true)
                .build();
    }

    public static CommandAction loop(String name, Runnable action) {
        Objects.requireNonNull(action, "action");
        return CommandAction.create(name)
                .onExecute(action::run)
                .build();
    }

    public static CommandAction wpilib(Command command) {
        return wpilib(command.getName(), command);
    }

    public static CommandAction wpilib(String name, Command command) {
        Objects.requireNonNull(command, "command");
        CommandScheduler scheduler = CommandScheduler.getInstance();
        return CommandAction.create(name)
                .onInitialize(() -> scheduler.schedule(command))
                .until(() -> !scheduler.isScheduled(command))
                .onEnd(() -> scheduler.cancel(command))
                .build();
    }

    static Command adapt(CommandAction Action) {
        Objects.requireNonNull(Action, "Action");
        return new Command() {
            {
                setName(Action.name());
            }

            @Override
            public void initialize() {
                Action.onInitialize().run();
            }

            @Override
            public void execute() {
                Action.onExecute().run();
            }

            @Override
            public boolean isFinished() {
                return Action.isFinished().getAsBoolean();
            }

            @Override
            public void end(boolean interrupted) {
                Action.onEnd().run();
            }
        };
    }

    public static final class Controller {
        private final XboxController controller;
        private final ModifiedAxis leftX;
        private final ModifiedAxis leftY;
        private final ModifiedAxis rightX;
        private final ModifiedAxis rightY;

        private Controller(XboxController controller) {
            this.controller = controller;
            leftX = axis(XboxController.Axis.kLeftX);
            leftY = axis(XboxController.Axis.kLeftY);
            rightX = axis(XboxController.Axis.kRightX);
            rightY = axis(XboxController.Axis.kRightY);
        }

        public Controller deadband(double deadband) {
            leftX.deadzone(deadband);
            leftY.deadzone(deadband);
            rightX.deadzone(deadband);
            rightY.deadzone(deadband);
            return this;
        }

        public Controller squareAxes() {
            leftX.squared(true);
            leftY.squared(true);
            rightX.squared(true);
            rightY.squared(true);
            return this;
        }

        public Controller invertLeftY() {
            leftY.inverted(true);
            return this;
        }

        public Controller invertRightY() {
            rightY.inverted(true);
            return this;
        }

        public double leftX() {
            return leftX.getAsDouble();
        }

        public double leftY() {
            return leftY.getAsDouble();
        }

        public double rightX() {
            return rightX.getAsDouble();
        }

        public double rightY() {
            return rightY.getAsDouble();
        }

        public DoubleSupplier leftXSupplier() {
            return this::leftX;
        }

        public DoubleSupplier leftYSupplier() {
            return this::leftY;
        }

        public DoubleSupplier rightXSupplier() {
            return this::rightX;
        }

        public DoubleSupplier rightYSupplier() {
            return this::rightY;
        }

        public Button a() {
            return button(XboxController.Button.kA);
        }

        public Button b() {
            return button(XboxController.Button.kB);
        }

        public Button x() {
            return button(XboxController.Button.kX);
        }

        public Button y() {
            return button(XboxController.Button.kY);
        }

        public Button leftBumper() {
            return button(XboxController.Button.kLeftBumper);
        }

        public Button rightBumper() {
            return button(XboxController.Button.kRightBumper);
        }

        public Button back() {
            return button(XboxController.Button.kBack);
        }

        public Button start() {
            return button(XboxController.Button.kStart);
        }

        public Button leftStick() {
            return button(XboxController.Button.kLeftStick);
        }

        public Button rightStick() {
            return button(XboxController.Button.kRightStick);
        }

        public Button povUp() {
            return pov(0);
        }

        public Button povRight() {
            return pov(90);
        }

        public Button povDown() {
            return pov(180);
        }

        public Button povLeft() {
            return pov(270);
        }

        public Button leftTrigger(double threshold) {
            return when(() -> controller.getLeftTriggerAxis() > threshold);
        }

        public Button rightTrigger(double threshold) {
            return when(() -> controller.getRightTriggerAxis() > threshold);
        }

        public Button when(BooleanSupplier condition) {
            return new Button(new Trigger(condition));
        }

        private Button button(XboxController.Button button) {
            return when(() -> controller.getRawButton(button.value));
        }

        private Button pov(int angle) {
            return when(() -> controller.getPOV() == angle);
        }

        private ModifiedAxis axis(XboxController.Axis axis) {
            return new ModifiedAxis(() -> controller.getRawAxis(axis.value), 0.0);
        }
    }

    public static final class Button {
        private final Trigger trigger;

        private Button(Trigger trigger) {
            this.trigger = trigger;
        }

        public Button onTrue(CommandAction Action) {
            trigger.onTrue(adapt(Action));
            return this;
        }

        public Button whileTrue(CommandAction Action) {
            trigger.whileTrue(adapt(Action));
            return this;
        }

        public Button toggleOnTrue(CommandAction Action) {
            trigger.toggleOnTrue(adapt(Action));
            return this;
        }

        public Button onFalse(CommandAction Action) {
            trigger.onFalse(adapt(Action));
            return this;
        }
    }
}
