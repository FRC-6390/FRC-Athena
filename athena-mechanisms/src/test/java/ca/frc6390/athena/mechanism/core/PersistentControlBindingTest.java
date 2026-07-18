package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import org.junit.jupiter.api.Test;

class PersistentControlBindingTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 81);

    @Test
    void targetChangesReuseOneBoundLoopWithoutResettingControllerState() {
        CountingLoop loop = new CountingLoop();
        ControlledMechanism mechanism = new ControlledMechanism(loop);
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);

        scheduler.request(mechanism.firstTarget);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.request(mechanism.secondTarget);
        scheduler.teleopPeriodic(0.02, 0.02);
        scheduler.cancel(mechanism.secondTarget);
        scheduler.teleopPeriodic(0.04, 0.02);

        assertEquals(1, loop.binds);
        assertEquals(1, loop.resets);
        assertEquals(3, loop.calculations);
    }

    @Test
    void cancellationResetsStateButReactivationDoesNotRebindLoop() {
        CountingLoop loop = new CountingLoop();
        ControlledMechanism mechanism = new ControlledMechanism(loop);
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);

        scheduler.request(mechanism.firstTarget);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.cancel(mechanism.firstTarget);
        scheduler.teleopPeriodic(0.02, 0.02);
        scheduler.request(mechanism.firstTarget);
        scheduler.teleopPeriodic(0.04, 0.02);

        assertEquals(1, loop.binds);
        assertEquals(3, loop.resets);
        assertEquals(2, loop.calculations);
    }

    @Test
    void disableResetsStateOnceAndEnableReusesBoundLoop() {
        CountingLoop loop = new CountingLoop();
        ControlledMechanism mechanism = new ControlledMechanism(loop);
        MechanismScheduler scheduler = MechanismScheduler.create(new RecordingContext()).register(mechanism);

        scheduler.request(mechanism.firstTarget);
        scheduler.teleopPeriodic(0.0, 0.02);
        scheduler.disabledPeriodic(0.02, 0.02);
        scheduler.disabledPeriodic(0.04, 0.02);
        scheduler.teleopPeriodic(0.06, 0.02);

        assertEquals(1, loop.binds);
        assertEquals(3, loop.resets);
        assertEquals(2, loop.calculations);
        assertTrue(scheduler.isRunning(mechanism.firstTarget));
    }

    private static final class ControlledMechanism implements Mechanism {
        private final MotorDevice motor = MOTOR;
        private final ControlBinding position;
        private final Action firstTarget;
        private final Action secondTarget;

        private ControlledMechanism(CountingLoop loop) {
            position = Controls.position(motor)
                    .feedback(() -> 0.0)
                    .loop(loop);
            firstTarget = position.position(1.0);
            secondTarget = position.position(2.0);
        }
    }

    private static final class CountingLoop implements ControlLoop {
        private int binds;
        private int resets;
        private int calculations;

        @Override
        public ControlLoopRuntime bind(ControlLoopBinding binding) {
            binds++;
            return new ControlLoopRuntime() {
                @Override
                public void reset(ControlLoopContext context) {
                    resets++;
                }

                @Override
                public ControlOutput calculate(ControlLoopContext context) {
                    calculations++;
                    return ControlOutput.voltage(context.target());
                }
            };
        }
    }

    private static final class RecordingContext implements ActionContext {
        private final RecordingMotor motor = new RecordingMotor();

        @Override
        public MotorHandle motor(MotorDevice requested) {
            return motor;
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        @Override
        public MotorDevice device() {
            return MOTOR;
        }
    }
}
