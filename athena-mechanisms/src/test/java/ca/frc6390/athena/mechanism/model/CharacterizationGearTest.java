package ca.frc6390.athena.mechanism.model;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import org.junit.jupiter.api.Test;

/** The characterization gear ratio comes from the feedback encoder unless the code overrides it. */
class CharacterizationGearTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

    @Test
    void encoderOnTheMotorShaftGivesOneMotorTurnPerTurn() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees);
        ControlBinding<?> control = Controls.position(MOTOR, Units.Degrees).feedback(encoder);
        assertEquals(1.0, Characterization.gearFromFeedback(control), 1.0e-9);
    }

    @Test
    void tenToOneReductionReadsAsTenMotorTurnsPerMechanismTurn() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 3).reduction(10.0)
                .measures(Units.Degrees);
        ControlBinding<?> control = Controls.position(MOTOR, Units.Degrees).feedback(encoder);
        assertEquals(10.0, Characterization.gearFromFeedback(control), 1.0e-9);
    }

    @Test
    void explicitOverrideWinsOverTheEncoder() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 4).reduction(10.0)
                .measures(Units.Degrees);
        Characterization characterization = Characterization.of(Controls.position(MOTOR, Units.Degrees).feedback(encoder))
                .gearRatio(3.0);
        assertEquals(3.0, characterization.gearRatio(), 1.0e-9);
    }
}
