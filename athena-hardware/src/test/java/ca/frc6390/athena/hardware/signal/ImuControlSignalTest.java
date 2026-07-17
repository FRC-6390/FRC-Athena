package ca.frc6390.athena.hardware.signal;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.List;
import org.junit.jupiter.api.Test;

class ImuControlSignalTest {
    @Test
    void axesExposeRadiansAndRetainSourceDependencies() {
        Object dependency = new Object();
        ImuSource source = new ImuSource() {
            @Override public double yawDegrees() { return 90.0; }
            @Override public double pitchDegrees() { return -45.0; }
            @Override public double rollDegrees() { return 30.0; }
            @Override public double angleDegrees() { return 90.0; }
            @Override public double yawRateDegreesPerSecond() { return 180.0; }
            @Override public double linearAccelerationXG() { return 0.0; }
            @Override public double linearAccelerationYG() { return 0.0; }
            @Override public double linearAccelerationZG() { return 0.0; }
            @Override public List<?> dependencies() { return List.of(dependency); }
            @Override public void applyYaw(ActionContext context, double yawDegrees) { }
        };

        assertEquals(Math.PI / 2.0, source.yaw().position(), 1.0e-9);
        assertEquals(-Math.PI / 4.0, source.pitch().position(), 1.0e-9);
        assertEquals(Math.PI / 6.0, source.roll().position(), 1.0e-9);
        assertEquals(Math.PI, source.yawRate().velocity(), 1.0e-9);
        assertEquals(List.of(dependency), source.yaw().dependencies());
        assertEquals(List.of(dependency), source.yawRate().dependencies());
    }
}
