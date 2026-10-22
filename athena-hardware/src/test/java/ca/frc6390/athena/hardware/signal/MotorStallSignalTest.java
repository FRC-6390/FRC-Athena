package ca.frc6390.athena.hardware.signal;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorCurrentLimits;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import org.junit.jupiter.api.Test;

class MotorStallSignalTest {
    @Test
    void revPortableCurrentMapsToPhaseWhileExplicitSupplyRemainsInBatteryAmps() {
        MotorCurrentLimits portable = MotorDevice.of(MotorKinds.NEO, 1)
                .currentLimit(Units.Amps.of(40))
                .currentLimits();
        MotorCurrentLimits both = MotorDevice.of(MotorKinds.NEO, 2)
                .supplyCurrentLimit(Units.Amps.of(50))
                .statorCurrentLimit(Units.Amps.of(60))
                .currentLimits();
        MotorCurrentLimits statorOnly = MotorDevice.of(MotorKinds.NEO, 3)
                .currentLimit(Units.Amps.of(0))
                .statorCurrentLimit(Units.Amps.of(55))
                .currentLimits();

        assertEquals(new MotorCurrentLimits(Units.Amps.of(0), Units.Amps.of(40)), portable);
        assertEquals(new MotorCurrentLimits(Units.Amps.of(50), Units.Amps.of(60)), both);
        assertEquals(new MotorCurrentLimits(Units.Amps.of(0), Units.Amps.of(55)), statorOnly);

        MotorKind customRev = () -> "rev:spark-max/custom";
        MotorKind customOther = () -> "custom:motor";
        assertEquals(new MotorCurrentLimits(Units.Amps.of(0), Units.Amps.of(25)),
                MotorDevice.of(customRev, 4).currentLimit(Units.Amps.of(25)).currentLimits());
        assertEquals(new MotorCurrentLimits(Units.Amps.of(25), Units.Amps.of(0)),
                MotorDevice.of(customOther, 5).currentLimit(Units.Amps.of(25)).currentLimits());
    }

    @Test
    void currentLimitedAppliedVoltageDoesNotHideACommandedStall() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).statorCurrentLimit(Units.Amps.of(40));
        FakeMotorHandle handle = new FakeMotorHandle(motor);
        AutoCloseable binding = motor.bindRuntime(new RuntimeScope("stall-test"), handle);
        try {
            MotorStallSignal stall = motor.stall().atCurrentLimit(0.9).velocityBelow(Units.RotationsPerSecond.of(1.0));

            motor.recordCommand(new MotorCommandSnapshot(MotorCommandSnapshot.Mode.PERCENT, 0.8));
            handle.statorCurrent = 38.0;
            handle.velocity = 0.2;
            handle.appliedVoltage = 0.4;

            assertTrue(stall.commanded());
            assertTrue(stall.instantaneousActive());
            motor.recordCommand(MotorCommandSnapshot.neutral());
            assertFalse(stall.instantaneousActive());
        } finally {
            binding.close();
        }
    }

    @Test
    void revStallDetectionUsesTheStatorLimitWhenBothCurrentDomainsAreDeclared() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 6)
                .supplyCurrentLimit(Units.Amps.of(10)).statorCurrentLimit(Units.Amps.of(60));
        FakeMotorHandle handle = new FakeMotorHandle(motor);
        AutoCloseable binding = motor.bindRuntime(new RuntimeScope("rev-stall-domains"), handle);
        try {
            MotorStallSignal stall = motor.stall().atCurrentLimit(0.9);
            motor.recordCommand(new MotorCommandSnapshot(MotorCommandSnapshot.Mode.VOLTAGE, 8.0));
            handle.velocity = 0.0;
            handle.supplyCurrent = 10.0;
            handle.statorCurrent = 20.0;
            assertFalse(stall.instantaneousActive(), "supply saturation alone is not stator saturation");
            handle.supplyCurrent = 2.0;
            handle.statorCurrent = 55.0;
            assertTrue(stall.instantaneousActive(), "partial-duty winding saturation uses the 60 A phase limit");
        } finally {
            binding.close();
        }
    }

    @Test
    void eachPhysicalPredicateCanPreventAFalseStall() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 2).supplyCurrentLimit(Units.Amps.of(30));
        FakeMotorHandle handle = new FakeMotorHandle(motor);
        AutoCloseable binding = motor.bindRuntime(new RuntimeScope("stall-predicates"), handle);
        try {
            // Fixed-threshold predicates: the default detector is model-based for known motors.
            MotorStallSignal stall = motor.stall().atCurrentLimit(0.9);
            motor.recordCommand(new MotorCommandSnapshot(MotorCommandSnapshot.Mode.VOLTAGE, 8.0));

            handle.supplyCurrent = 20.0;
            handle.velocity = 0.0;
            assertFalse(stall.instantaneousActive());
            handle.supplyCurrent = 29.0;
            handle.velocity = 3.0;
            assertFalse(stall.instantaneousActive());
            handle.velocity = 0.0;
            assertTrue(stall.instantaneousActive());
        } finally {
            binding.close();
        }
    }

    private static final class FakeMotorHandle implements MotorHandle {
        private final MotorDevice device;
        private double appliedVoltage;
        private double supplyCurrent;
        private double statorCurrent;
        private double velocity;

        private FakeMotorHandle(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public double appliedVoltage() { return appliedVoltage; }
        @Override public double supplyCurrentAmps() { return supplyCurrent; }
        @Override public double statorCurrentAmps() { return statorCurrent; }
        @Override public double integratedVelocityRotationsPerSecond() { return velocity; }
    }
}
