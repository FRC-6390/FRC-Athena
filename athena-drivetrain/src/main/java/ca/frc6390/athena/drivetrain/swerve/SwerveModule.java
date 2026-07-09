package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.EncoderSlot;
import ca.frc6390.athena.mechanism.core.MechanismTemplate;
import ca.frc6390.athena.mechanism.core.MotorSlot;
import ca.frc6390.athena.mechanism.core.Slots;
import java.util.Objects;

/**
 * Base type for a swerve module declared as a specialized mechanism.
 */
public abstract class SwerveModule implements MechanismTemplate {
    private final SwerveModuleModel model;
    public final MotorSlot<SwerveModule> drive = Slots.motor(this, "drive", this::configureIfReady);
    public final MotorSlot<SwerveModule> steer = Slots.motor(this, "steer", this::configureIfReady);
    public final EncoderSlot<SwerveModule> angle = Slots.encoder(this, "angle", this::configureIfReady);
    public ControlBinding driveVelocity;
    public ControlBinding steerPosition;

    protected SwerveModule(SwerveModuleModel model) {
        this.model = Objects.requireNonNull(model, "model");
    }

    public SwerveModuleModel model() {
        return model;
    }

    private void configureIfReady() {
        if (!drive.filled() || !steer.filled() || !angle.filled()) {
            return;
        }

        EncoderDevice driveDistance = drive.get().encoder()
                .gearRatio(model.driveReduction())
                .wheelDiameterMeters(model.wheelDiameterMeters())
                .units(EncoderUnit.METERS);

        driveVelocity = Controls.velocity(drive.get())
                .feedback(driveDistance);
        steerPosition = Controls.position(steer.get())
                .feedback(angle.get());
    }

    public Action target(SwerveModuleTarget target) {
        Objects.requireNonNull(target, "target");
        if (driveVelocity == null || steerPosition == null) {
            throw new IllegalStateException("Swerve module controls have not been configured.");
        }
        return Actions.parallel(
                driveVelocity.velocity(target.speedMetersPerSecond()),
                steerPosition.position(target.angleRotations()));
    }
}
