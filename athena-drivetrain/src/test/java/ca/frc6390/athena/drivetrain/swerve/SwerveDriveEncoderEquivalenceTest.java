package ca.frc6390.athena.drivetrain.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Pins that the typed drive-distance encoder used by swerve modules reports the same position and
 * velocity as the untyped declaration it replaces, for real presets.
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
            EncoderDevice legacy = motor.encoder()
                    .gearRatio(GearRatio.reduction(model.driveReduction(), 1.0))
                    .wheelDiameterMeters(model.wheelDiameterMeters())
                    .units(EncoderUnit.METERS);
            EncoderDevice typed = motor.encoder()
                    .reduction(model.driveReduction())
                    .measures(Units.Meters)
                    .travelPerRevolution(Units.Meters.of(Math.PI * model.wheelDiameterMeters()));
            for (double rotations : new double[] {0.0, 1.0, -2.5, 13.75}) {
                assertEquals(legacy.positionFromRotations(rotations), typed.positionFromRotations(rotations), 0.0,
                        model.name() + " position");
            }
            for (double rate : new double[] {0.0, 3.0, -7.5}) {
                assertEquals(legacy.velocityFromRotationsPerSecond(rate), typed.velocityFromRotationsPerSecond(rate),
                        0.0, model.name() + " velocity");
            }
        }
    }
}
