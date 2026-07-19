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
            @Override public double pitchRateDegreesPerSecond() { return -90.0; }
            @Override public double rollRateDegreesPerSecond() { return 45.0; }
            @Override public double linearAccelerationXG() { return 0.1; }
            @Override public double linearAccelerationYG() { return -0.2; }
            @Override public double linearAccelerationZG() { return 0.3; }
            @Override public List<?> dependencies() { return List.of(dependency); }
            @Override public void applyYaw(ActionContext context, double yawDegrees) { }
        };

        assertEquals(Math.PI / 2.0, source.yaw().position(), 1.0e-9);
        assertEquals(-Math.PI / 4.0, source.pitch().position(), 1.0e-9);
        assertEquals(Math.PI / 6.0, source.roll().position(), 1.0e-9);
        assertEquals(Math.PI, source.yawRate().velocity(), 1.0e-9);
        assertEquals(-Math.PI / 2.0, source.pitchRate().velocity(), 1.0e-9);
        assertEquals(Math.PI / 4.0, source.rollRate().velocity(), 1.0e-9);
        assertEquals(0.1, source.accelerationX().value(), 1.0e-9);
        assertEquals(-0.2, source.accelerationY().value(), 1.0e-9);
        assertEquals(0.3, source.accelerationZ().value(), 1.0e-9);
        assertEquals(List.of(dependency), source.yaw().dependencies());
        assertEquals(List.of(dependency), source.yawRate().dependencies());

        assertEquals(-Math.PI / 2.0, source.yaw().inverted().position(), 1.0e-9);
        assertEquals(Math.PI / 4.0, source.pitch().inverted().position(), 1.0e-9);
        assertEquals(-Math.PI, source.yawRate().inverted().velocity(), 1.0e-9);
        assertEquals(List.of(dependency), source.yaw().inverted().dependencies());
        assertEquals(List.of(dependency), source.yawRate().inverted().dependencies());
        assertEquals(List.of(dependency), source.accelerationX().dependencies());
        assertEquals(-0.1, source.accelerationX().inverted().value(), 1.0e-9);
        assertEquals(source.yaw().position(), source.yaw().inverted(false).position(), 1.0e-9);
    }

    @Test
    void headingIsWrappedWhileContinuousHeadingUsesAccumulatedAngle() {
        ImuSource source = new TestSource(450.0, 810.0, 0.0, 0.0);

        assertEquals(Math.PI / 2.0, source.heading().position(), 1.0e-9);
        assertEquals(Math.PI / 2.0, source.yaw().position(), 1.0e-9);
        assertEquals(4.5 * Math.PI, source.continuousHeading().position(), 1.0e-9);
    }

    private record TestSource(double yawValue, double angleValue, double pitchValue, double rollValue)
            implements ImuSource {
        @Override public double yawDegrees() { return yawValue; }
        @Override public double angleDegrees() { return angleValue; }
        @Override public double pitchDegrees() { return pitchValue; }
        @Override public double rollDegrees() { return rollValue; }
        @Override public double yawRateDegreesPerSecond() { return 0.0; }
        @Override public double linearAccelerationXG() { return 0.0; }
        @Override public double linearAccelerationYG() { return 0.0; }
        @Override public double linearAccelerationZG() { return 0.0; }
        @Override public void applyYaw(ActionContext context, double yawDegrees) { }
    }
}
