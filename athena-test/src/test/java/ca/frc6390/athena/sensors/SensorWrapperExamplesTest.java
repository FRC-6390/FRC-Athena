package ca.frc6390.athena.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicReference;

import ca.frc6390.athena.core.RobotTime;
import ca.frc6390.athena.sensors.beambreak.IRBeamBreak;
import ca.frc6390.athena.sensors.button.GenericButton;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.examples.SensorWrapperExamples;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.BlockDirection;
import org.junit.jupiter.api.Test;

final class SensorWrapperExamplesTest {

    @Test
    void hardstopLimitSwitchAppliesConfigValues() {
        GenericLimitSwitch limitSwitch = SensorWrapperExamples.createHardstopSwitch(
                0,
                BlockDirection.NegativeInput,
                1.25,
                0.02);

        try {
            assertTrue(limitSwitch.isHardstop());
            assertEquals(1.25, limitSwitch.getPosition(), 1e-9);
            assertEquals(-1, limitSwitch.getBlockDirectionMultiplier());
        } finally {
            limitSwitch.close();
        }
    }

    @Test
    void buttonAndBeamBreakDefaultToInverted() {
        GenericButton button = SensorWrapperExamples.createButton(1);
        IRBeamBreak beamBreak = SensorWrapperExamples.createBeamBreak(2);

        try {
            assertTrue(button.isInverted());
            assertTrue(beamBreak.isInverted());

            if (button.supportsSimulation()) {
                button.setSimulatedTriggered(true);
                assertTrue(button.getAsBoolean());
                button.setSimulatedTriggered(false);
                assertFalse(button.getAsBoolean());
            }
        } finally {
            button.close();
            beamBreak.close();
        }
    }

    @Test
    void cameraWrapperReturnsYawOnlyWhenFiniteTargetExists() {
        AtomicReference<Double> yaw = new AtomicReference<>(12.0);
        VisionCamera camera = SensorWrapperExamples.createTargetingCamera("cam", yaw::get);

        resetRobotTimeCache();
        OptionalDouble first = SensorWrapperExamples.readYaw(camera);
        assertTrue(first.isPresent());
        assertEquals(12.0, first.getAsDouble(), 1e-9);

        yaw.set(Double.NaN);
        resetRobotTimeCache();
        OptionalDouble second = SensorWrapperExamples.readYaw(camera);
        assertTrue(second.isEmpty());
    }

    private static void resetRobotTimeCache() {
        RobotTime.resetNowSecondsForTest();
    }
}
