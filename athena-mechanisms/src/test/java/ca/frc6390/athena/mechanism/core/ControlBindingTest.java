package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import java.util.List;
import org.junit.jupiter.api.Test;

class ControlBindingTest {
    @Test
    void motorsArePrecomputedAndImmutable() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        ControlBinding control = new ControlBinding(null, null, null, null, null, null)
                .output(leader)
                .follower(follower);

        List<MotorDevice> motors = control.motors();

        assertSame(motors, control.motors());
        assertEquals(List.of(leader, follower), motors);
        assertThrows(UnsupportedOperationException.class, () -> motors.add(MotorDevice.of(MotorKinds.KRAKEN_X60, 3)));
    }

    @Test
    void slotIsClampedAndPreservedAcrossBindingUpdates() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);

        ControlBinding control = Controls.position(leader)
                .slot(-1)
                .follower(follower)
                .slot(3)
                .pid(0.1, 0.0, 0.0);

        assertEquals(3, control.slot());
        assertEquals(List.of(leader, follower), control.motors());
    }

    @Test
    void ordinaryEncoderFeedbackBindsBothChannelsWithoutExtraApiLayer() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2);

        ControlBinding control = Controls.position(motor).feedback(encoder);

        assertSame(encoder, control.feedback().position());
        assertSame(encoder, control.feedback().velocity());
        assertEquals(java.util.Set.of(encoder), control.feedback().encoders());
    }

    @Test
    void explicitFeedbackBindingKeepsMixedChannelsDistinct() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        PositionSignal position = context -> 12.0;
        VelocitySignal velocity = context -> -3.0;
        FeedbackBinding feedback = new FeedbackBinding(position, velocity);

        ControlBinding control = Controls.position(motor).feedback(feedback);

        assertSame(feedback, control.feedback());
        assertSame(position, control.feedback().position());
        assertSame(velocity, control.feedback().velocity());
    }
}
