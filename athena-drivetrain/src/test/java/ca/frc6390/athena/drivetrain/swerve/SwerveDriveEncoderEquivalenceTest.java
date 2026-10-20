package ca.frc6390.athena.drivetrain.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Pins that the typed drive-distance encoder used by swerve modules reports the same position and
 * velocity as the untyped declaration it replaced (gear 1/reduction, conversion PI * wheel diameter,
 * meters), for real presets.
 */
class SwerveDriveEncoderEquivalenceTest {

    @Test
    void typedDriveDistanceMatchesLegacyDeclarationForPresets() {
        List<SwerveModule> modules = List.of(
                new SwerveModules.SDS.MK4N.L2_PLUS(),
                new SwerveModules.SDS.MK4I.L2(),
                new SwerveModules.SDS.MK5N.R1(),
                new SwerveModules.REV.MAXSWERVE_3IN.MEDIUM(),
                new SwerveModules.REV.EASYSWERVE_4IN.STANDARD());
        for (SwerveModule module : modules) {
            SwerveModuleModel model = module.model();
            MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
            EncoderDevice legacy = new EncoderDevice(motor.encoder().source(), false,
                    1.0 / model.driveReduction(), Math.PI * model.wheelDiameter().in(Units.Meters), 0.0,
                    FailurePolicy.DISABLE_MECHANISM, false, Units.Meters);
            EncoderDevice typed = motor.encoder()
                    .reduction(model.driveReduction())
                    .measures(Units.Meters)
                    .travelPerRevolution(Units.Meters.of(Math.PI * model.wheelDiameter().in(Units.Meters)));
            for (double rotations : new double[] {0.0, 1.0, -2.5, 13.75}) {
                assertEquals(legacy.raw().positionFromRotations(rotations), typed.raw().positionFromRotations(rotations), 0.0,
                        model.name() + " position");
            }
            for (double rate : new double[] {0.0, 3.0, -7.5}) {
                assertEquals(legacy.raw().velocityFromRotationsPerSecond(rate), typed.raw().velocityFromRotationsPerSecond(rate),
                        0.0, model.name() + " velocity");
            }
        }
    }
}
