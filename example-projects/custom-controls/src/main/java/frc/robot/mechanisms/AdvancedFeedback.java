package frc.robot.mechanisms;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.FeedbackBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import frc.robot.Constants;
import frc.robot.controls.FeedbackSignals;

/** Demonstrates ordinary, modular/fused, and redundant feedback bindings. */
public final class AdvancedFeedback implements Mechanism {
    private final MotorDevice modularMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 22);

    // Both are single-turn absolute sensors. The second sensor must be geared so
    // it turns 16 times while the mechanism turns 17 times.
    private final EncoderDevice directAbsolute = Constants.RIO.encoder(EncoderKinds.CANCODER, 30);
    private final EncoderDevice vernierAbsolute = Constants.RIO.encoder(EncoderKinds.CANCODER, 31);

    private final PositionSignal modularAbsolute = FeedbackSignals.modular(
            Range.rotations(0.0, 17.0),
            0.025,
            FeedbackSignals.input(FeedbackSignals.absolutePhase(directAbsolute), 1.0),
            FeedbackSignals.input(FeedbackSignals.absolutePhase(vernierAbsolute), 16.0 / 17.0));

    // CRT/modular reconstruction establishes the turn count. The integrated
    // encoder then supplies smooth motion between gated absolute corrections.
    private final PositionSignal trackedPosition = FeedbackSignals.absoluteRelative(
            modularAbsolute,
            modularMotor.encoder(),
            0.25,
            0.02);
    private final VelocitySignal filteredVelocity = FeedbackSignals.lowPass(
            modularMotor.encoder(),
            0.15);

    // The ordinary case remains terse and pays no FeedbackBinding API cost.
    private final ControlBinding ordinary = Controls.position(modularMotor)
            .feedback(modularMotor.encoder())
            .pid(0.12, 0.0, 0.002);

    // Only the mixed case constructs FeedbackBinding explicitly.
    private final ControlBinding modular = Controls.position(modularMotor)
            .feedback(new FeedbackBinding(trackedPosition, filteredVelocity))
            .pid(1.44, 0.0, 0.024);

    private final MotorDevice redundantMotor = Constants.RIO.motor(MotorKinds.KRAKEN_X60, 23);
    private final EncoderDevice redundantA = Constants.RIO.encoder(EncoderKinds.CANCODER, 32);
    private final EncoderDevice redundantB = Constants.RIO.encoder(EncoderKinds.CANCODER, 33);
    private final PositionSignal medianPosition = FeedbackSignals.median(
            redundantA,
            redundantB,
            redundantMotor.encoder());
    private final ControlBinding redundant = Controls.position(redundantMotor)
            .feedback(new FeedbackBinding(
                    medianPosition,
                    FeedbackSignals.lowPass(redundantMotor.encoder(), 0.2)))
            .pid(1.2, 0.0, 0.012);

    public final Action ordinaryRelativeMove = ordinary.position(2.0);
    public final Action modularAbsoluteMove = modular.position(12.0);
    public final Action redundantMove = redundant.position(3.0);
    public final Action stop = Actions.parallel(
            modularMotor.percent(0.0),
            redundantMotor.percent(0.0));
}
