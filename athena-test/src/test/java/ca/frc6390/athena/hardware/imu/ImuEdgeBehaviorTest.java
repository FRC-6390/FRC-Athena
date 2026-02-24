package ca.frc6390.athena.hardware.imu;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.examples.ImuEncoderEdgeExamples;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

final class ImuEdgeBehaviorTest {

    @Test
    void virtualImuAppliesInversionToSimulatedReadings() {
        FakeImu raw = new FakeImu();
        VirtualImu imu = ImuEncoderEdgeExamples.createVirtualImu(raw);

        imu.setInverted(true);
        imu.setSimulatedReadings(
                Rotation2d.fromDegrees(30.0),
                Rotation2d.fromDegrees(5.0),
                Rotation2d.fromDegrees(-2.0),
                Rotation2d.fromRadians(0.0),
                Rotation2d.fromRadians(0.0),
                Rotation2d.fromRadians(1.5));

        assertEquals(30.0, imu.getYaw().getDegrees(), 1e-9);
        assertEquals(5.0, imu.getPitch().getDegrees(), 1e-9);
        assertEquals(-2.0, imu.getRoll().getDegrees(), 1e-9);
        assertEquals(1.5, imu.getVelocityZ().getRadians(), 1e-9);
    }

    @Test
    void maxSpeedWindowRejectsNaNAndClampsToMinimum() {
        FakeImu raw = new FakeImu();
        VirtualImu imu = ImuEncoderEdgeExamples.createVirtualImu(raw);

        double original = imu.getMaxSpeedWindowSeconds();
        ImuEncoderEdgeExamples.configureMaxSpeedWindow(imu, Double.NaN);
        assertEquals(original, imu.getMaxSpeedWindowSeconds(), 1e-9);

        ImuEncoderEdgeExamples.configureMaxSpeedWindow(imu, 0.0);
        assertEquals(0.02, imu.getMaxSpeedWindowSeconds(), 1e-9);
    }

    @Test
    void updateTracksFiniteAngularAccelerationAndMaxRadialSpeed() {
        FakeImu raw = new FakeImu();
        VirtualImu imu = ImuEncoderEdgeExamples.createVirtualImu(raw);

        imu.setSimulatedHeading(Rotation2d.fromDegrees(0.0), Rotation2d.fromRadians(0.5));
        imu.update();

        imu.setSimulatedHeading(Rotation2d.fromDegrees(0.0), Rotation2d.fromRadians(1.5));
        imu.update();

        assertTrue(Double.isFinite(imu.getAngularAccelerationZRadiansPerSecondSquared()));
        assertTrue(imu.getMaxRadialSpeed() >= 1.5 - 1e-9);
    }

    private static final class FakeImu implements Imu {
        private static final ImuType TYPE = () -> "test";

        @Override
        public Rotation2d getRoll() {
            return new Rotation2d();
        }

        @Override
        public Rotation2d getPitch() {
            return new Rotation2d();
        }

        @Override
        public Rotation2d getYaw() {
            return new Rotation2d();
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public ImuConfig getConfig() {
            return ImuConfig.create(TYPE, 0);
        }
    }
}
