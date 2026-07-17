package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import org.junit.jupiter.api.Test;

class VelocityContributionActionTest {
    @Test
    void contributionPublishesWhileScheduledAndClearsWhenCancelled() {
        double[] omega = {0.5};
        TestMechanism mechanism = new TestMechanism(() -> RobotVelocity.angular(omega[0]));
        MechanismScheduler scheduler = MechanismScheduler.create(new MotorContext()).register(mechanism);

        scheduler.request(mechanism.aim);
        scheduler.teleopPeriodic(0.0, 0.02);
        assertTrue(mechanism.aimVelocity.isActive());
        assertEquals(0.5, mechanism.velocity.robotRelative(0.0).angularRadiansPerSecond(), 1.0e-9);

        omega[0] = -0.75;
        scheduler.teleopPeriodic(0.02, 0.02);
        assertEquals(-0.75, mechanism.velocity.robotRelative(0.0).angularRadiansPerSecond(), 1.0e-9);

        scheduler.cancel(mechanism.aim);
        assertFalse(mechanism.aimVelocity.isActive());
        assertEquals(RobotVelocity.zero(), mechanism.velocity.robotRelative(0.0));
    }

    private static final class TestMechanism implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        private final RobotVelocityPool velocity = new RobotVelocityPool();
        private final RobotVelocityPool.Channel aimVelocity = velocity.channel();
        private final Action drive = motor.voltage(0.0);
        private final Action aim;

        private TestMechanism(java.util.function.Supplier<RobotVelocity> target) {
            aim = Actions.contributeVelocity(aimVelocity, target, drive);
        }
    }

    private static final class MotorContext implements ActionContext {
        @Override
        public MotorHandle motor(MotorDevice motor) {
            return new MotorHandle() {
                @Override public MotorDevice device() { return motor; }
                @Override public void setVoltage(double volts) { }
                @Override public void stop() { }
            };
        }
    }
}
