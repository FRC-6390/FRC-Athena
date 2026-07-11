package frc.robot;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import java.util.function.DoubleSupplier;

public final class DriveTrain implements Mechanism {
    private final MotorDevice left = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 1);
    private final MotorDevice right = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 2).inverted();

    public final Action stop = Actions.parallel(left.percent(0.0), right.percent(0.0));

    public Action arcade(DoubleSupplier forward, DoubleSupplier turn) {
        return Actions.parallel(
                left.percent(() -> clamp(forward.getAsDouble() + turn.getAsDouble())),
                right.percent(() -> clamp(forward.getAsDouble() - turn.getAsDouble())));
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}
