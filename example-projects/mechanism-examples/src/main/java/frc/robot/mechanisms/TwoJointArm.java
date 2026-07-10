package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.DigitalInputs;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public final class TwoJointArm implements Mechanism {
    private final MotorDevice shoulderMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 3).currentLimit(35);
    private final MotorDevice wristMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X44, 4).currentLimit(20);
    private final EncoderDevice shoulderEncoder = Constants.RIO.encoder(EncoderKinds.CANCODER, 3);
    private final EncoderDevice wristEncoder = Constants.RIO.encoder(EncoderKinds.CANCODER, 4);
    private final DigitalInputDevice homeSwitch = DigitalInputs.rio(0).inverted();
    private final Range shoulderTravel = Range.degrees(-25.0, 95.0);
    private final Range wristTravel = Range.degrees(-70.0, 80.0);
    @SuppressWarnings("unused")
    private final SimModel shoulderSimulation = SimModels.arm(shoulderMotor)
            .encoder(shoulderEncoder)
            .range(shoulderTravel)
            .lengthMeters(Units.inchesToMeters(22.0))
            .gearRatio(GearRatio.reduction(60.0, 1.0));
    @SuppressWarnings("unused")
    private final SimModel wristSimulation = SimModels.arm(wristMotor)
            .encoder(wristEncoder)
            .range(wristTravel)
            .lengthMeters(Units.inchesToMeters(12.0))
            .gearRatio(GearRatio.reduction(30.0, 1.0));
    private final ControlBinding shoulder = Controls.position(shoulderMotor)
            .feedback(shoulderEncoder)
            .pid(0.08, 0.0, 0.002);
    private final ControlBinding wrist = Controls.position(wristMotor)
            .feedback(wristEncoder)
            .pid(0.06, 0.0, 0.001);

    public final Action stow = Actions.sequence()
            .until(homeSwitch::active, shoulderMotor.percent(() -> homeSwitch.active() ? 0.0 : -0.15))
            .then(shoulderEncoder.setPosition(0.0).then(pose(0.0, 0.0)));
    public final Action floorPickup = pose(-18.0, -45.0);
    public final Action ampScore = pose(65.0, 40.0);
    public final Action trapScore = pose(92.0, 70.0);

    private Action pose(double shoulderDegrees, double wristDegrees) {
        return Actions.parallel(
                shoulder.position(shoulderDegrees).clamp(shoulderTravel),
                wrist.position(wristDegrees).clamp(wristTravel));
    }
}
