package ca.frc6390.athena.hardware.signal;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import org.junit.jupiter.api.Test;

class ImusTest {
    @Test
    void relativeSourceMaintainsIndependentHeadingOffset() {
        double[] yaw = {35.0};
        ImuSource physical = Imus.yaw(() -> yaw[0]);
        ImuSource relative = physical.relative();

        assertEquals(0.0, relative.yawDegrees(), 1.0e-9);
        yaw[0] = 50.0;
        assertEquals(15.0, relative.yawDegrees(), 1.0e-9);

        relative.applyYaw(ActionContext.empty(), -10.0);
        assertEquals(-10.0, relative.yawDegrees(), 1.0e-9);
        assertEquals(50.0, physical.yawDegrees(), 1.0e-9);
    }

    @Test
    void yawOnlySourceRejectsMeasurementsItDoesNotProvide() {
        ImuSource source = Imus.yaw(() -> 12.0);

        assertThrows(UnsupportedOperationException.class, source::pitchDegrees);
        assertThrows(UnsupportedOperationException.class, source::linearAccelerationXG);
    }
}
