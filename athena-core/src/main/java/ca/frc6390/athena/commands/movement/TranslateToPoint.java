package ca.frc6390.athena.commands.movement;

import java.util.Objects;

import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TranslateToPoint extends Command{

    private final RobotSpeeds speeds;
    private final double targetXMeters;
    private final double targetYMeters;
    private PIDController xController;
    private PIDController yController;
    private double estimatedX;
    private double estimatedY;
    private double lastTimestampSeconds;
    private double lastCommandedVx;
    private double lastCommandedVy;

    public TranslateToPoint(RobotDrivetrain<?> drivetrain, double x, double y){
        this.speeds = drivetrain.robotSpeeds();
        this.targetXMeters = Double.isFinite(x) ? x : 0.0;
        this.targetYMeters = Double.isFinite(y) ? y : 0.0;
        PIDController base = new PIDController(1.0, 0.0, 0.0);
        base.setTolerance(0.05);
        applyController(base);
    }

    public TranslateToPoint setPID(PIDController translationPID){
        applyController(Objects.requireNonNull(translationPID, "translationPID"));
        return this;
    }

    private void applyController(PIDController template) {
        PIDController x = new PIDController(template.getP(), template.getI(), template.getD());
        PIDController y = new PIDController(template.getP(), template.getI(), template.getD());
        x.setTolerance(template.getErrorTolerance(), template.getErrorDerivativeTolerance());
        y.setTolerance(template.getErrorTolerance(), template.getErrorDerivativeTolerance());
        this.xController = x;
        this.yController = y;
    }

    @Override
    public void initialize() {
        estimatedX = 0.0;
        estimatedY = 0.0;
        lastCommandedVx = 0.0;
        lastCommandedVy = 0.0;
        lastTimestampSeconds = Double.NaN;
        xController.reset();
        yController.reset();
        xController.setSetpoint(targetXMeters);
        yController.setSetpoint(targetYMeters);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double dt = 0.02;
        if (Double.isFinite(lastTimestampSeconds)) {
            double measured = now - lastTimestampSeconds;
            if (Double.isFinite(measured) && measured >= 1e-3) {
                dt = measured;
            }
        }
        lastTimestampSeconds = now;

        estimatedX += lastCommandedVx * dt;
        estimatedY += lastCommandedVy * dt;

        double vxMetersPerSecond = xController.calculate(estimatedX);
        double vyMetersPerSecond = yController.calculate(estimatedY);
        double maxVelocity = speeds.getMaxVelocity();
        if (Double.isFinite(maxVelocity) && maxVelocity > 0.0) {
            vxMetersPerSecond = MathUtil.clamp(vxMetersPerSecond, -maxVelocity, maxVelocity);
            vyMetersPerSecond = MathUtil.clamp(vyMetersPerSecond, -maxVelocity, maxVelocity);
        }

        speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, vxMetersPerSecond, vyMetersPerSecond, 0.0);
        lastCommandedVx = vxMetersPerSecond;
        lastCommandedVy = vyMetersPerSecond;
    }

    @Override
    public void end(boolean interrupted) {
        speeds.stopSpeeds(RobotSpeeds.FEEDBACK_SOURCE);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint();
    }
}
