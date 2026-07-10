package frc.robot.controls;

import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlOutput;

public final class SlewLimitedPositionLoop implements ControlLoopRuntime {
    private final double maxStepPerSecond;
    private final double kP;
    private double profiledTarget;

    public SlewLimitedPositionLoop(double maxStepPerSecond, double kP) {
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
