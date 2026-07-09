package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
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
}
