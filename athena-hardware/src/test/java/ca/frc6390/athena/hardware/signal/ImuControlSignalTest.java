package ca.frc6390.athena.hardware.signal;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class ImuControlSignalTest {
    @Test
    void axesExposeRadiansAndRetainSourceDependencies() {
        Object dependency = new Object();
        ImuSource source = new ImuSource() {
            @Override public double readYawDegrees() { return 90.0; }
            @Override public double readPitchDegrees() { return -45.0; }
            @Override public double readRollDegrees() { return 30.0; }
            @Override public double readAngleDegrees() { return 90.0; }
            @Override public double readYawRateDegreesPerSecond() { return 180.0; }
            @Override public double readPitchRateDegreesPerSecond() { return -90.0; }
            @Override public double readRollRateDegreesPerSecond() { return 45.0; }
            @Override public double readLinearAccelerationXG() { return 0.1; }
            @Override public double readLinearAccelerationYG() { return -0.2; }
            @Override public double readLinearAccelerationZG() { return 0.3; }
            @Override public List<?> dependencies() { return List.of(dependency); }
            @Override public void writeYawDegrees(ActionContext context, double yawDegrees) { }
        };

        assertEquals(Math.PI / 2.0, source.headingSignal().position(), 1.0e-9);
        assertEquals(-Math.PI / 4.0, source.pitchSignal().position(), 1.0e-9);
        assertEquals(Math.PI / 6.0, source.rollSignal().position(), 1.0e-9);
        assertEquals(Math.PI, source.yawRateSignal().velocity(), 1.0e-9);
        assertEquals(-Math.PI / 2.0, source.pitchRateSignal().velocity(), 1.0e-9);
        assertEquals(Math.PI / 4.0, source.rollRateSignal().velocity(), 1.0e-9);
        assertEquals(0.1, source.accelerationXSignal().value(), 1.0e-9);
        assertEquals(-0.2, source.accelerationYSignal().value(), 1.0e-9);
        assertEquals(0.3, source.accelerationZSignal().value(), 1.0e-9);
        assertEquals(List.of(dependency), source.headingSignal().dependencies());
        assertEquals(List.of(dependency), source.yawRateSignal().dependencies());

        assertEquals(-Math.PI / 2.0, source.inverted().headingSignal().position(), 1.0e-9);
        assertEquals(Math.PI / 4.0, source.inverted().pitchSignal().position(), 1.0e-9);
        assertEquals(-Math.PI, source.inverted().yawRateSignal().velocity(), 1.0e-9);
        assertEquals(List.of(dependency), source.inverted().headingSignal().dependencies());
        assertEquals(List.of(dependency), source.inverted().yawRateSignal().dependencies());
        assertEquals(List.of(dependency), source.accelerationXSignal().dependencies());
        assertEquals(-0.1, source.inverted().accelerationXSignal().value(), 1.0e-9);
        assertEquals(source.headingSignal().position(), source.inverted(false).headingSignal().position(), 1.0e-9);
    }

    @Test
    void typedAccessorsMatchPrimitiveReadingsInTheirUnits() {
        ImuSource source = new TestSource(90.0, 90.0, -45.0, 30.0);

        assertEquals(90.0, source.yaw().in(Units.Degrees), 1.0e-12);
        assertEquals(-45.0, source.pitch().in(Units.Degrees), 1.0e-12);
        assertEquals(30.0, source.roll().in(Units.Degrees), 1.0e-12);
        assertEquals(90.0, source.angle().in(Units.Degrees), 1.0e-12);
    }

    @Test
    void headingIsWrappedWhileContinuousHeadingUsesAccumulatedAngle() {
        ImuSource source = new TestSource(450.0, 810.0, 0.0, 0.0);

        assertEquals(Math.PI / 2.0, source.headingSignal().position(), 1.0e-9);
        // yaw() is the raw reading (450 degrees); only the heading signal wraps.
        assertEquals(2.5 * Math.PI, source.yaw().in(Units.Radians), 1.0e-9);
        assertEquals(4.5 * Math.PI, source.continuousHeadingSignal().position(), 1.0e-9);
    }

    private record TestSource(double yawValue, double angleValue, double pitchValue, double rollValue)
            implements ImuSource {
        @Override public double readYawDegrees() { return yawValue; }
        @Override public double readAngleDegrees() { return angleValue; }
        @Override public double readPitchDegrees() { return pitchValue; }
        @Override public double readRollDegrees() { return rollValue; }
        @Override public double readYawRateDegreesPerSecond() { return 0.0; }
        @Override public double readLinearAccelerationXG() { return 0.0; }
        @Override public double readLinearAccelerationYG() { return 0.0; }
        @Override public double readLinearAccelerationZG() { return 0.0; }
        @Override public void writeYawDegrees(ActionContext context, double yawDegrees) { }
    }
}
