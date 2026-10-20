package ca.frc6390.athena.hardware.units;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Pins the typed encoder reads ({@code positionAngle()}, {@code positionDistance()}, their absolute and velocity
 * forms) against the primitive reads they wrap ({@code raw()}), so the typed path cannot change a number. Values are
 * read through a bound runtime, the same path robot code takes.
 */
class EncoderTypedReadsTest {
    private static final double TOLERANCE = 1.0e-9;

    /** Runs one encoder through a fake backend whose handle the test sets directly. */
    private static final class Bound {
        final HardwareGraph graph;
        final FakeHandle handle;

        Bound(EncoderDevice encoder) {
            FakeBackend backend = new FakeBackend();
            graph = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(backend), List.of()));
            graph.encoder(encoder);
            handle = backend.last;
        }
    }

    private static final class FakeBackend implements EncoderBackend {
        FakeHandle last;

        @Override
        public boolean supports(EncoderKind kind) {
            return kind == EncoderKinds.CANCODER;
        }

        @Override
        public EncoderHandle create(EncoderDevice device) {
            last = new FakeHandle(device);
            return last;
        }
    }

    private static final class FakeHandle implements EncoderHandle {
        private final EncoderDevice device;
        double position;
        double absolutePosition;
        double velocity;

        FakeHandle(EncoderDevice device) {
            this.device = device;
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double absolutePositionRotations() {
            return absolutePosition;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocity;
        }
    }

    private static final double[] ROTATIONS = {0.0, 0.25, -1.5, 3.7, 1234.5};

    @Test
    void degreesReadsMatchTheRawDoublesAndTheFormula() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2)
                .offset(0.25).reduction(2.0).measures(Units.Degrees);
        Bound bound = new Bound(encoder);

        for (double rotations : ROTATIONS) {
            bound.handle.position = rotations;
            bound.handle.absolutePosition = -rotations;
            bound.handle.velocity = rotations * 0.5;

            double expectedPosition = (rotations - 0.25) * 0.5 * 360.0;
            double expectedAbsolute = (-rotations - 0.25) * 0.5 * 360.0;
            double expectedVelocity = rotations * 0.5 * 0.5 * 360.0;

            assertEquals(expectedPosition, encoder.raw().position(), TOLERANCE);
            assertEquals(expectedPosition, encoder.positionAngle().in(Units.Degrees), TOLERANCE);
            assertEquals(encoder.raw().position(), encoder.positionAngle().magnitude(), 0.0);
            assertEquals(expectedAbsolute, encoder.absolutePositionAngle().in(Units.Degrees), TOLERANCE);
            assertEquals(encoder.raw().absolutePosition(), encoder.absolutePositionAngle().magnitude(), 0.0);
            assertEquals(expectedVelocity, encoder.velocityAngular().in(Units.DegreesPerSecond), TOLERANCE);
            assertEquals(encoder.raw().velocity(), encoder.velocityAngular().magnitude(), 0.0);
        }
        bound.graph.close();
    }

    @Test
    void rotationsReadsMatchTheRawDoubles() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 3).reduction(4.0).offset(0.5);
        Bound bound = new Bound(encoder);

        for (double rotations : ROTATIONS) {
            bound.handle.position = rotations;
            bound.handle.absolutePosition = rotations + 1.0;
            bound.handle.velocity = -rotations;

            assertEquals((rotations - 0.5) * 0.25, encoder.positionAngle().in(Units.Rotations), TOLERANCE);
            assertEquals(encoder.raw().position(), encoder.positionAngle().magnitude(), 0.0);
            assertEquals(encoder.raw().absolutePosition(), encoder.absolutePositionAngle().magnitude(), 0.0);
            assertEquals(encoder.raw().velocity(), encoder.velocityAngular().magnitude(), 0.0);
        }
        bound.graph.close();
    }

    @Test
    void metersReadsMatchTheRawDoubles() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 4)
                .reduction(6.75).measures(Units.Meters).travelPerRevolution(Units.Inches.of(12.0));
        Bound bound = new Bound(encoder);

        for (double rotations : ROTATIONS) {
            bound.handle.position = rotations;
            bound.handle.absolutePosition = rotations * 2.0;
            bound.handle.velocity = rotations / 3.0;

            double expectedMeters = rotations * (1.0 / 6.75) * 0.3048;
            assertEquals(expectedMeters, encoder.positionDistance().in(Units.Meters), TOLERANCE);
            assertEquals(encoder.raw().position(), encoder.positionDistance().magnitude(), 0.0);
            assertEquals(encoder.raw().absolutePosition(), encoder.absolutePositionDistance().magnitude(), 0.0);
            assertEquals(encoder.raw().velocity(), encoder.velocityLinear().magnitude(), 0.0);
            assertEquals(rotations / 3.0 * (1.0 / 6.75) * 0.3048,
                    encoder.velocityLinear().in(Units.MetersPerSecond), TOLERANCE);
        }
        bound.graph.close();
    }

    @Test
    void typedReadsRejectTheOtherDimension() {
        EncoderDevice angle = EncoderDevice.of(EncoderKinds.CANCODER, 5).measures(Units.Degrees);
        EncoderDevice distance = EncoderDevice.of(EncoderKinds.CANCODER, 6).measures(Units.Meters);
        Bound angleBound = new Bound(angle);
        Bound distanceBound = new Bound(distance);

        IllegalStateException angleError = assertThrows(IllegalStateException.class, angle::positionDistance);
        assertTrue(angleError.getMessage().contains("positionAngle()"));
        assertThrows(IllegalStateException.class, angle::absolutePositionDistance);
        assertThrows(IllegalStateException.class, angle::velocityLinear);
        assertThrows(IllegalStateException.class, distance::positionAngle);
        assertThrows(IllegalStateException.class, distance::absolutePositionAngle);
        assertThrows(IllegalStateException.class, distance::velocityAngular);
        angleBound.graph.close();
        distanceBound.graph.close();
    }

    @Test
    void feedbackSignalsReadTheSameConfiguredValues() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 7).reduction(10.0).measures(Units.Degrees);
        Bound bound = new Bound(encoder);
        bound.handle.position = 0.3;
        bound.handle.absolutePosition = 0.1;
        bound.handle.velocity = 2.0;

        PositionSignal position = encoder.positionSignal();
        PositionSignal absolute = encoder.absoluteSignal();
        VelocitySignal velocity = encoder.velocitySignal();

        assertEquals(encoder.raw().position(), position.position(), 0.0);
        assertEquals(encoder.raw().absolutePosition(), absolute.position(), 0.0);
        assertEquals(encoder.raw().velocity(), velocity.velocity(), 0.0);
        assertEquals(List.of(encoder), position.dependencies());
        assertEquals(-encoder.raw().position(), position.inverted().position(), 0.0);
        bound.graph.close();
    }
}
