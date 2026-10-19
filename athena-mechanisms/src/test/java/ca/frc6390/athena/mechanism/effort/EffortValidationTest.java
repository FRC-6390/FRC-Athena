package ca.frc6390.athena.mechanism.effort;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class EffortValidationTest {
    private static final MotorDevice KRAKEN = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final MotorDevice KRAKEN_2 = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
    private static final MotorDevice MINION = MotorDevice.of(MotorKinds.MINION, 3);

    private static EffortUnits.Kind kind(edu.wpi.first.units.Measure<?> measure, ca.frc6390.athena.hardware.units.MotionKind motion) {
        return EffortUnits.kind(measure, motion);
    }

    @Test
    void rotaryAcceptsTorqueAndCurrentOnly() {
        assertEquals(EffortUnits.Kind.TORQUE,
                kind(Units.NewtonMeters.of(4), ca.frc6390.athena.hardware.units.MotionKind.ROTARY));
        assertEquals(EffortUnits.Kind.CURRENT,
                kind(Units.Amps.of(20), ca.frc6390.athena.hardware.units.MotionKind.ROTARY));
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> kind(Units.Newtons.of(4), ca.frc6390.athena.hardware.units.MotionKind.ROTARY));
        assertTrue(error.getMessage().contains("Rotary control accepts Torque"));
    }

    @Test
    void linearAcceptsForceAndCurrentOnly() {
        assertEquals(EffortUnits.Kind.FORCE,
                kind(Units.Newtons.of(40), ca.frc6390.athena.hardware.units.MotionKind.LINEAR));
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> kind(Units.NewtonMeters.of(4), ca.frc6390.athena.hardware.units.MotionKind.LINEAR));
        assertTrue(error.getMessage().contains("Linear control accepts Force"));
    }

    @Test
    void untypedAcceptsCurrentOnly() {
        assertEquals(EffortUnits.Kind.CURRENT,
                kind(Units.Amps.of(10), ca.frc6390.athena.hardware.units.MotionKind.UNSPECIFIED));
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> kind(Units.NewtonMeters.of(1), ca.frc6390.athena.hardware.units.MotionKind.UNSPECIFIED));
        assertTrue(error.getMessage().contains("Current (A) effort only"));
    }

    @Test
    void torqueWithoutGearingRequiresAmps() {
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> Controls.position(KRAKEN, Units.Rotations)
                        .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 4).measures(Units.Rotations))
                        .effort(Effort.max(Units.NewtonMeters.of(5))));
        assertTrue(error.getMessage().contains("requires known gearing"));
        assertTrue(error.getMessage().contains("use Amps"));
    }

    @Test
    void forceWithoutRadiusIsRejected() {
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> Controls.position(KRAKEN, Units.Meters)
                        .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 5).measures(Units.Meters))
                        .effort(Effort.max(Units.Newtons.of(100)).gearing(10.0)));
        assertTrue(error.getMessage().contains("drum radius"));
    }

    @Test
    void torqueNeedsMotorConstantsForTheKind() {
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> Controls.position(MINION, Units.Rotations)
                        .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 6).measures(Units.Rotations))
                        .effort(Effort.max(Units.NewtonMeters.of(2)).gearing(5.0)));
        assertTrue(error.getMessage().contains("torque constants"));
        // Amps still declare: the hardware limit is the only guard for such a motor.
        ControlBinding<?> amps = Controls.position(MINION, Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 6).measures(Units.Rotations))
                .effort(Effort.max(Units.Amps.of(20)));
        assertEquals(1, amps.loops().stream().filter(EffortLimit.class::isInstance).count());
    }

    @Test
    void perMotorSplitUsesEachMotorsTorqueConstantRatioAndCount() {
        MotorConstants kraken = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
        double torque = 6.0;
        double gearing = 25.0;
        double[] amps = EffortUnits.perMotorAmps(Units.NewtonMeters.of(torque), EffortUnits.Kind.TORQUE,
                List.of(KRAKEN, KRAKEN_2), gearing, null);
        double expected = torque / (2 * gearing * kraken.raw().torqueConstantNmPerAmp());
        assertEquals(expected, amps[0], 1e-9);
        assertEquals(expected, amps[1], 1e-9);
        // Round trip through the model: N motors at this current produce the declared torque.
        assertEquals(torque, EffortUnits.mechanismTorque(amps, List.of(KRAKEN, KRAKEN_2), gearing), 1e-9);
    }

    @Test
    void forceConvertsThroughTheDrumRadius() {
        double[] amps = EffortUnits.perMotorAmps(Units.Newtons.of(200), EffortUnits.Kind.FORCE,
                List.of(KRAKEN), 10.0, Units.Inches.of(2.0));
        double torque = 200 * Units.Inches.of(2.0).in(Units.Meters);
        MotorConstants kraken = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
        assertEquals(torque / (10.0 * kraken.raw().torqueConstantNmPerAmp()), amps[0], 1e-9);
    }

    @Test
    void actionEffortOnAControlPositionCarriesAPerMotorLimit() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 7).measures(Units.Rotations))
                .effort(Effort.max(Units.NewtonMeters.of(8)).gearing(25.0));
        Action action = arm.position(0.5).effort(Units.NewtonMeters.of(4));
        var position = assertInstanceOf(Actions.ControlPosition.class, action);
        MotorConstants kraken = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
        assertEquals(4.0 / (25.0 * kraken.raw().torqueConstantNmPerAmp()), position.effort().currentLimitAmps(), 1e-9);
        // Amps on the action need no gearing.
        var amps = assertInstanceOf(Actions.ControlPosition.class,
                Controls.position(KRAKEN, Units.Rotations).feedback(EncoderDevice.of(EncoderKinds.CANCODER, 8)
                        .measures(Units.Rotations)).position(0.5).effort(Units.Amps.of(12)));
        assertEquals(12.0, amps.effort().currentLimitAmps(), 1e-12);
    }

    @Test
    void actionTorqueWithoutBindingGearingIsRejectedAtDeclaration() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 9).measures(Units.Rotations));
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> arm.position(0.5).effort(Units.NewtonMeters.of(4)));
        assertTrue(error.getMessage().contains("requires known gearing"));
    }

    @Test
    void compositesCarryTheLimitToTheirControlActions() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 10).measures(Units.Rotations));
        Action composed = arm.position(0.5).until(() -> false)
                .then(arm.position(0.0)).effort(Units.Amps.of(3));
        Actions.Conditional conditional = assertInstanceOf(Actions.Conditional.class, composed);
        var first = assertInstanceOf(Actions.ControlPosition.class, conditional.action());
        assertEquals(3.0, first.effort().currentLimitAmps(), 1e-12);
        var next = assertInstanceOf(Actions.ControlPosition.class, conditional.next());
        assertEquals(3.0, next.effort().currentLimitAmps(), 1e-12);
    }

    @Test
    void loadResponsesValidateTheirArguments() {
        assertThrows(IllegalArgumentException.class, () -> LoadResponse.slow(0.0));
        assertThrows(IllegalArgumentException.class, () -> LoadResponse.slow(1.5));
        assertThrows(NullPointerException.class, () -> Effort.max(Units.Amps.of(1)).onLoad((LoadResponse) null));
        IllegalArgumentException duplicate = assertThrows(IllegalArgumentException.class,
                () -> Controls.position(KRAKEN, Units.Rotations)
                        .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 11).measures(Units.Rotations))
                        .effort(Effort.max(Units.Amps.of(5)).onLoad(LoadResponse.HOLD, LoadResponse.HOLD)));
        assertTrue(duplicate.getMessage().contains("at most one"));
        IllegalArgumentException burstUnit = assertThrows(IllegalArgumentException.class,
                () -> Controls.position(KRAKEN, Units.Rotations)
                        .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 12).measures(Units.Rotations))
                        .effort(Effort.max(Units.Amps.of(5)).gearing(1.0)
                                .onLoad(LoadResponse.pushThrough(Units.Newtons.of(3), Units.Seconds.of(0.2)))));
        assertTrue(burstUnit.getMessage().contains("Current"));
    }

    @Test
    void ceilingMustBeNonNegativeAndGearingPositive() {
        assertThrows(IllegalArgumentException.class, () -> Effort.max(Units.Amps.of(1)).gearing(0.0));
        assertThrows(IllegalArgumentException.class, () -> Controls.position(KRAKEN, Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 13).measures(Units.Rotations))
                .effort(Effort.max(Units.Amps.of(-1))));
    }

    @Test
    void publishesTheCeilingInMechanismUnitsAndAmps() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 14).measures(Units.Rotations))
                .effort(Effort.max(Units.NewtonMeters.of(4)).gearing(25.0));
        EffortLimit limit = (EffortLimit) arm.loops().stream().filter(EffortLimit.class::isInstance).findFirst().orElseThrow();
        var telemetry = limit.telemetry();
        assertTrue(telemetry.containsKey("Ceiling"));
        assertTrue(telemetry.containsKey("CeilingAmps"));
        assertTrue(telemetry.containsKey("ActiveAmps"));
        assertTrue(telemetry.containsKey("ActiveEffort"));
        assertEquals(4.0, telemetry.get("Ceiling").number(), 1e-12);
        MotorConstants kraken = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
        assertEquals(4.0 / (25.0 * kraken.raw().torqueConstantNmPerAmp()), telemetry.get("CeilingAmps").number(), 1e-9);
    }
}
