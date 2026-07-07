package ca.frc6390.athena.hardware.ref;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.input.InputSourceKind;
import ca.frc6390.athena.hardware.input.InputType;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.hardware.spec.NeutralMode;

class HardwareRefTest {
    @Test
    void digitalInputsReadDirectlyAndLowerToInputSpecs() {
        DigitalInputRef homeSwitch = DigitalInputs.rio(8)
                .inverted()
                .bind(() -> false);

        assertFalse(homeSwitch.raw());
        assertTrue(homeSwitch.active());

        var spec = homeSwitch.toSpec("intake.arm");
        assertEquals("intake.arm.dio8", spec.path());
        assertEquals(InputType.BOOLEAN, spec.type());
        assertEquals(InputSourceKind.DIGITAL_CHANNEL, spec.sourceKind());
        assertEquals(8, spec.channel());
    }

    @Test
    void softwareNumberAndBooleanRefsCanUseDefaultsOrBoundReaders() {
        NumberRef distance = Numbers.of("targetDistanceMeters", 3.2);
        BooleanRef hasTarget = Booleans.of("vision/hasTarget").bind(() -> true);
        BooleanRef blocked = Booleans.value(false);

        assertEquals(3.2, distance.value(), 1.0e-9);
        assertTrue(hasTarget.active());
        assertFalse(blocked.active());
        assertThrows(IllegalStateException.class, () -> Numbers.of("unbound").value());
    }

    @Test
    void controllerOwnsAxisAndButtonRefs() {
        ControllerRef driver = Controllers.xbox(0);
        ControllerAxisRef throttle = driver.leftY().bind(() -> -0.8);
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
        MotorRef left = MotorRef.of(AthenaMotor.SIM, 1);
        MotorRef right = MotorRef.of(AthenaMotor.SIM, 2)
                .inverted()
                .follow(left);
        SimRef simulation = Sim.flywheel(left, right).momentOfInertia(0.02);

        assertEquals(SimProfile.Kind.FLYWHEEL, simulation.kind());
        assertEquals(0.02, simulation.momentOfInertia().orElseThrow(), 1.0e-9);
        assertEquals(left, simulation.motors().get(0));
        assertEquals(right, simulation.motors().get(1));
        assertFalse(simulation.simulatesGravity());
    }

    @Test
    void motorRefsCanDeclareFollowers() {
        MotorRef leader = MotorRef.of(AthenaMotor.SIM, 1);
        MotorRef follower = MotorRef.of(AthenaMotor.SIM, 2)
                .inverted()
                .follow(leader)
                .currentLimit(30);

        assertEquals(leader, follower.follower().leader());
        assertTrue(follower.isInverted());
        assertEquals(30, follower.currentLimitAmps());
    }

    @Test
    void motorRefsProvideIntegratedEncoderRefs() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 7)
                .canbus("canivore");
        EncoderRef encoder = motor.encoder();

        assertEquals(ca.frc6390.athena.api.hardware.AthenaEncoder.INTEGRATED_MOTOR, encoder.kind());
        assertEquals(7, encoder.id());
        assertEquals("canivore", encoder.canbus());
    }

    @Test
    void imuRefsCarryHardwareBusAndMountPose() {
        ImuRef imu = HardwareBus.can("canivore")
                .imu(AthenaImu.PIGEON_2, 30)
                .mountPose(1.0, 2.0, 3.0);

        assertEquals(AthenaImu.PIGEON_2, imu.kind());
        assertEquals(30, imu.id());
        assertEquals("canivore", imu.canbus());
        assertEquals(1.0, imu.mountPose().yawDegrees(), 1.0e-9);
    }

    @Test
    void encoderRefsCarryConversionOffsetAndUnitsOnOneRef() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 7);
        EncoderRef encoder = motor.encoder()
                .gearRatio(5.36)
                .wheelDiameterInches(4.0)
                .offset(0.25)
                .units(EncoderUnit.METERS);

        assertEquals(5.36, encoder.gearRatio(), 1.0e-9);
        assertEquals(Math.PI * 4.0 * 0.0254, encoder.conversion(), 1.0e-9);
        assertEquals(0.25, encoder.offset(), 1.0e-9);
        assertEquals(EncoderUnit.METERS, encoder.units());
    }

    @Test
    void runtimeMotorWrapsBackendDeviceClosedLoopTargets() {
        RecordingMotorDevice device = new RecordingMotorDevice();
        RuntimeMotor motor = RuntimeMotor.from(device);

        motor.percent(0.5);
        motor.voltage(6.0);
        motor.position(12.0);
        motor.velocity(34.0);
        motor.stop();

        assertEquals(0.0, device.percent, 1.0e-9);
        assertEquals(6.0, device.voltage, 1.0e-9);
        assertEquals(12.0, device.position, 1.0e-9);
        assertEquals(34.0, device.velocity, 1.0e-9);
    }

    private static final class RecordingMotorDevice implements MotorDevice {
        private final MotorSpec spec = new MotorSpec(
                "test",
                "motor",
                ca.frc6390.athena.api.hardware.AthenaMotor.SIM,
                1,
                "rio",
                NeutralMode.COAST,
                40,
                false);
        private double percent;
        private double voltage;
        private double position;
        private double velocity;

        @Override
        public MotorSpec spec() {
            return spec;
        }

        @Override
        public void setPercentOutput(double percent) {
            this.percent = percent;
        }

        @Override
        public void setVoltage(double volts) {
            voltage = volts;
        }

        @Override
        public void setPositionTargetRotations(double rotations) {
            position = rotations;
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocity = rotationsPerSecond;
        }
    }
}
