package ca.frc6390.athena.hardware.ref;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.input.InputType;

class HardwareRefTest {
    @Test
    void digitalInputsReadDirectlyAndLowerToInputSpecs() {
        DigitalInputRef homeSwitch = DigitalInputs.rio("Home Switch", 8)
                .inverted()
                .bind(() -> false);

        assertFalse(homeSwitch.raw());
        assertTrue(homeSwitch.active());

        var spec = homeSwitch.toSpec("intake.arm");
        assertEquals("intake.arm.home_switch", spec.path());
        assertEquals(InputType.BOOLEAN, spec.type());
        assertEquals(InputSourceKind.DIGITAL_CHANNEL, spec.sourceKind());
        assertEquals(8, spec.channel());
    }

    @Test
    void softwareNumberAndBooleanRefsCanUseDefaultsOrBoundReaders() {
        NumberRef distance = Numbers.of("targetDistanceMeters", 3.2);
        BooleanRef hasTarget = Booleans.of("vision/hasTarget").bind(() -> true);

        assertEquals(3.2, distance.value(), 1.0e-9);
        assertTrue(hasTarget.active());
        assertThrows(IllegalStateException.class, () -> Numbers.of("unbound").value());
    }

    @Test
    void controllerOwnsAxisAndButtonRefs() {
        ControllerRef driver = Controllers.xbox(0);
        AxisRef throttle = driver.leftY().bind(() -> -0.8);
        ButtonRef fire = driver.rightTrigger().bind(() -> 0.7).at(0.5);

        assertEquals("xbox_0", driver.defaultName());
        assertEquals("xbox_0_lefty", throttle.defaultName());
        assertEquals(-0.8, throttle.value(), 1.0e-9);
        assertTrue(fire.pressed());
    }

    @Test
    void rangeAndGearRatioRefsCarryReusableMechanismValues() {
        RangeRef range = RangeRef.degrees(-10.0, 40.0);
        GearRatioRef ratio = GearRatioRef.sensorToMechanism(0.25);

        assertEquals(-10.0, range.clamp(-20.0), 1.0e-9);
        assertEquals(20.0, range.clamp(20.0), 1.0e-9);
        assertEquals(40.0, range.clamp(50.0), 1.0e-9);
        assertEquals(0.25, ratio.ratio(), 1.0e-9);
    }

    @Test
    void simRefsBindSharedSimulationModelsToHardware() {
        MotorRef left = MotorRef.of(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 1);
        MotorRef right = MotorRef.of(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 2)
                .inverted()
                .follow(left);
        SimRef simulation = Sim.flywheel(left, right).moi(0.02);

        assertEquals(SimProfile.Kind.FLYWHEEL, simulation.kind());
        assertEquals(0.02, simulation.moi().orElseThrow(), 1.0e-9);
        assertEquals(left, simulation.motors().get(0));
        assertEquals(right, simulation.motors().get(1));
        assertFalse(simulation.simulatesGravity());
    }

    @Test
    void motorRefsCanDeclareFollowers() {
        MotorRef leader = MotorRef.of(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 1);
        MotorRef follower = MotorRef.of(ca.frc6390.athena.api.hardware.AthenaMotor.SIM, 2)
                .inverted()
                .follow(leader)
                .currentLimit(30);

        assertEquals(leader, follower.follower().leader());
        assertTrue(follower.isInverted());
        assertEquals(30, follower.currentLimitAmps());
    }
}
