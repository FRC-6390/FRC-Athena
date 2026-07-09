package frc.robot;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlLoops;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;

public final class Robot extends AthenaRobot {
    public final ExperimentalArm arm = new ExperimentalArm();
    public final Flywheel flywheel = new Flywheel();

    @SuppressWarnings("unused")
    public final HookBinding runControls = Events.teleopPeriodic().whileActive(() -> {
        arm.motionProfiled.request();
        flywheel.withArbitraryFeedforward.request();
    });

    @Override
    protected void configure() {
        register(arm);
        register(flywheel);
    }

    public static final class ExperimentalArm implements Mechanism {
        private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 20);
        private final ControlBinding arm = Controls.position(motor)
                .feedback(motor.encoder())
                .loop(ControlLoops.software(binding -> new SlewLimitedPositionLoop(0.45, 0.08)));

        public final Action home = arm.position(0.0);
        public final Action motionProfiled = arm.position(75.0);
        public final Action holdOperatorSetpoint = arm.position(this::operatorSetpointDegrees);

        private double operatorSetpointDegrees() {
            return 35.0;
        }
    }

    public static final class Flywheel implements Mechanism {
        private final MotorDevice motor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 21);
        private final ControlBinding wheel = Controls.velocity(motor)
                .feedback(motor.encoder())
                .slot(1)
                .pid(0.08, 0.0, 0.0)
                .loop(ControlLoops.arbitraryFeedforward(binding -> context ->
                        ControlOutput.voltage(0.4 + 0.02 * Math.abs(context.target()))));

        public final Action off = motor.percent(0.0);
        public final Action withDevicePid = wheel.velocity(55.0);
        public final Action withArbitraryFeedforward = wheel.velocity(75.0);
    }

    private static final class SlewLimitedPositionLoop implements ControlLoopRuntime {
        private final double maxStepPerSecond;
        private final double kP;
        private double profiledTarget;

        private SlewLimitedPositionLoop(double maxStepPerSecond, double kP) {
            this.maxStepPerSecond = maxStepPerSecond;
            this.kP = kP;
        }

        @Override
        public void reset(ControlLoopContext context) {
            profiledTarget = context.position();
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            double target = context.target();
            double maxStep = maxStepPerSecond * context.dtSeconds();
            profiledTarget += clamp(target - profiledTarget, -maxStep, maxStep);
            return ControlOutput.percent(clamp((profiledTarget - context.position()) * kP, -0.8, 0.8));
        }

        private static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }
}
