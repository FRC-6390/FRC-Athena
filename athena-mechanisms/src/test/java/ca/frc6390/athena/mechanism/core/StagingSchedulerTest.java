package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.staging.Staging;
import edu.wpi.first.units.Units;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Stage Actions are requested and cancelled through the scheduler, and PowerPolicy is discovered. */
class StagingSchedulerTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 81);

    enum Health { NORMAL, HOT }

    private static final class Robot implements Mechanism {
        boolean hot;
        @SuppressWarnings("unused")
        private final MotorDevice motor = MOTOR;
        @SuppressWarnings("unused")
        private final PowerPolicy power = PowerPolicy.defaults().floor(Units.Volts.of(7.5));
        @SuppressWarnings("unused")
        private final Staging<Health> health = Staging.of(Health.NORMAL)
                .stage(Health.HOT, () -> hot, (Action) MOTOR.percent(0.5))
                .exitAfter(Units.Seconds.of(0.0));
    }

    private static final class Hardware implements ActionContext {
        final Map<MotorDevice, Motor> motors = new HashMap<>();
        @Override public Motor motor(MotorDevice motor) { return motors.computeIfAbsent(motor, Motor::new); }
    }

    private static final class Motor implements MotorHandle {
        final MotorDevice device;
        double percent = Double.NaN;

        Motor(MotorDevice device) { this.device = device; }

        @Override public MotorDevice device() { return device; }
        @Override public void setPercentOutput(double value) { percent = value; }
        @Override public void stop() { percent = 0.0; }
    }

    @Test
    void stageActionIsRequestedOnEntryAndCancelledOnExit() {
        Robot robot = new Robot();
        Hardware hardware = new Hardware();
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(robot);
        assertEquals(7.5, scheduler.powerPolicy().raw().floorVolts(), 1e-9);

        scheduler.teleopPeriodic(0.00, 0.02);
        assertTrue(Double.isNaN(hardware.motor(MOTOR).percent), "nothing driven while normal");

        robot.hot = true;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(0.5, hardware.motor(MOTOR).percent, 1e-9, "HOT stage action requested");

        robot.hot = false;
        scheduler.teleopPeriodic(0.04, 0.02);
        assertEquals(0.0, hardware.motor(MOTOR).percent, 1e-9, "stage action cancelled on exit");
        assertEquals(2, scheduler.drainStageChanges().size());
        assertEquals(1, scheduler.powerSnapshot().motors().size());
    }
}
