package frc.robot;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import java.util.function.DoubleSupplier;

public final class DriveTrain implements Mechanism {
    public final MotorDevice leftLeader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 1)
            .brake()
            .currentLimit(Constants.Drive.CURRENT_LIMIT_AMPS);
    public final MotorDevice leftFollower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 2)
            .follow(leftLeader)
            .brake()
            .currentLimit(Constants.Drive.CURRENT_LIMIT_AMPS);
    public final MotorDevice rightLeader = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 3)
            .inverted()
            .brake()
            .currentLimit(Constants.Drive.CURRENT_LIMIT_AMPS);
    public final MotorDevice rightFollower = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 4)
            .follow(rightLeader)
            .brake()
            .currentLimit(Constants.Drive.CURRENT_LIMIT_AMPS);

    private final SimModel leftSimulation = SimModel.motor(leftLeader, leftFollower)
            .encoder(leftLeader.encoder());
    private final SimModel rightSimulation = SimModel.motor(rightLeader, rightFollower)
            .encoder(rightLeader.encoder());

    public Action arcadeDrive(DoubleSupplier forward, DoubleSupplier turn) {
        return Actions.parallel(
                leftLeader.percent(() -> clamp(forward.getAsDouble() + turn.getAsDouble())),
                rightLeader.percent(() -> clamp(forward.getAsDouble() - turn.getAsDouble())));
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
