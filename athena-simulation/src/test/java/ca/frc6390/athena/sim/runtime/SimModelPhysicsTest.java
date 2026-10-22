package ca.frc6390.athena.sim.runtime;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import org.junit.jupiter.api.Test;

/** Physical checks of the default model runner against the motor characteristics it is built from. */
class SimModelPhysicsTest {
    private static final double KRAKEN_X44_FREE_SPEED_RAD_PER_SECOND = 812.416;
    private static final double KRAKEN_X44_STALL_TORQUE = 4.11;
    private static final double KRAKEN_X44_STALL_CURRENT = 279.0;
    private static final double KRAKEN_X44_FREE_CURRENT = 2.0;

    @Test
    void motorConstantsReproduceTheCatalogueFreeSpeedStallTorqueAndFreeCurrent() {
        DcMotorModel x44 = DcMotorModel.of(MotorKinds.KRAKEN_X44);

        // Stall: zero speed, 12 V, the torque is the catalogue stall torque.
        assertEquals(KRAKEN_X44_STALL_TORQUE,
                x44.torque(x44.current(12.0, 0.0), 0.0), 1.0e-3);
        // Free speed: at the no-load speed the motor draws its free current and produces no net torque.
        assertEquals(KRAKEN_X44_FREE_CURRENT,
                x44.current(12.0, KRAKEN_X44_FREE_SPEED_RAD_PER_SECOND), 1.0e-6);
        assertEquals(0.0, x44.torque(x44.current(12.0, KRAKEN_X44_FREE_SPEED_RAD_PER_SECOND),
                KRAKEN_X44_FREE_SPEED_RAD_PER_SECOND), 1.0e-6);
        assertEquals(12.0 / KRAKEN_X44_STALL_CURRENT, x44.resistanceOhms(), 1.0e-9);
        assertEquals(KRAKEN_X44_STALL_CURRENT, 12.0 / x44.resistanceOhms(), 1.0e-6);
    }

    @Test
    void unloadedMotorSpinsUpToTheFreeSpeedOfItsKind() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 60);
        EncoderDevice encoder = motor.encoder();
        SimulationSession session = SimulationSession.create().model("free-axis", SimModel.motor(motor));
        session.motor(motor).setVoltage(12.0);

        for (int cycle = 0; cycle < 1000; cycle++) {
            session.step(edu.wpi.first.units.Units.Seconds.of(0.02));
        }

        double freeSpeedRotations = KRAKEN_X44_FREE_SPEED_RAD_PER_SECOND / (2.0 * Math.PI);
        assertEquals(freeSpeedRotations, session.encoder(encoder).velocityRotationsPerSecond(),
                0.03 * freeSpeedRotations, "no-load speed should be the X44 free speed");
    }

    @Test
    void gearingDividesTheMotorSpeedAtTheMechanism() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 61);
        EncoderDevice encoder = motor.encoder().gearing(GearRatio.reduction(100.0, 1.0));
        SimulationSession session = SimulationSession.create().model("geared-free-axis", SimModel.motor(motor)
                .gearRatio(GearRatio.reduction(100.0, 1.0)));
        session.motor(motor).setVoltage(12.0);

        for (int cycle = 0; cycle < 1000; cycle++) {
            session.step(edu.wpi.first.units.Units.Seconds.of(0.02));
        }

        double motorRotationsPerSecond = session.motor(motor).integratedVelocityRotationsPerSecond();
        double rawEncoderRotationsPerSecond = session.encoder(encoder).velocityRotationsPerSecond();
        // The integrated encoder counts motor rotations; its configured gearing gives mechanism degrees.
        assertEquals(motorRotationsPerSecond, rawEncoderRotationsPerSecond, 1.0e-9);
        assertEquals(motorRotationsPerSecond / 100.0,
                encoder.raw().velocityFromRotationsPerSecond(rawEncoderRotationsPerSecond), 1.0e-9);
        assertEquals(KRAKEN_X44_FREE_SPEED_RAD_PER_SECOND / (2.0 * Math.PI), motorRotationsPerSecond,
                0.03 * KRAKEN_X44_FREE_SPEED_RAD_PER_SECOND / (2.0 * Math.PI));
    }

    @Test
    void brakedAxisDecaysFasterThanCoastingAxis() {
        double brakedRate = decayRate(true);
        double coastRate = decayRate(false);

        assertTrue(brakedRate > 20.0 * coastRate,
                () -> "shorted windings should decelerate the axis much faster: braked=" + brakedRate
                        + " coast=" + coastRate);
    }

    @Test
    void supplyAmpsAreNotAnEqualWindingTorqueLimitAtPartialDuty() {
        MotorDevice unlimited = MotorDevice.of(MotorKinds.NEO, 70).currentLimit(Units.Amps.of(0));
        MotorDevice supply = MotorDevice.of(MotorKinds.NEO, 71).currentLimit(Units.Amps.of(0))
                .supplyCurrentLimit(Units.Amps.of(5));
        MotorDevice phase = MotorDevice.of(MotorKinds.NEO, 72).currentLimit(Units.Amps.of(0))
                .supplyCurrentLimit(Units.Amps.of(5)).statorCurrentLimit(Units.Amps.of(5));

        // At 1 V/12 V duty, the stationary NEO draws about 9 A winding but less than 1 A
        // from the battery. The host power pass owns supply enforcement; this physics runner
        // must not silently replace its 5 A supply allowance with only 5 A of winding torque.
        double unlimitedSpeed = speedAfterLowDutyStep(unlimited);
        assertEquals(unlimitedSpeed, speedAfterLowDutyStep(supply), 1e-12);
        assertTrue(speedAfterLowDutyStep(phase) < 0.75 * unlimitedSpeed,
                "a separately declared 5 A phase limit still limits simulated torque");
    }

    private static double speedAfterLowDutyStep(MotorDevice motor) {
        motor.encoder();
        SimulationSession session = SimulationSession.create().model("current-domain-axis", SimModel.motor(motor)
                .momentOfInertia(Units.KilogramSquareMeters.of(1.0)));
        session.motor(motor).setVoltage(1.0);
        session.step(Units.Seconds.of(0.02));
        return session.motor(motor).integratedVelocityRotationsPerSecond();
    }

    /** Decay rate (1/s) of a released 100:1, 0.5 kg m^2 axis, from the speed ratio after 50 ms. */
    private static double decayRate(boolean brake) {
        MotorDevice motor = brake
                ? MotorDevice.of(MotorKinds.KRAKEN_X44, 62).brake()
                : MotorDevice.of(MotorKinds.KRAKEN_X44, 63).coast();
        EncoderDevice encoder = motor.encoder();
        SimulationSession session = SimulationSession.create().model("axis", SimModel.motor(motor)
                .gearRatio(GearRatio.reduction(100.0, 1.0))
                .momentOfInertia(edu.wpi.first.units.Units.KilogramSquareMeters.of(0.5)));
        session.encoder(encoder).positionRotations(0.0).velocityRotationsPerSecond(50.0);
        session.motor(motor).stop();
        session.step(edu.wpi.first.units.Units.Seconds.of(0.05));
        double after = session.encoder(encoder).velocityRotationsPerSecond();
        return -Math.log(after / 50.0) / 0.05;
    }
}
