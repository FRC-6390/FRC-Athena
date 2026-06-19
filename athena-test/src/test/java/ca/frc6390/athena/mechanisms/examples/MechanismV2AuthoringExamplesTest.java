package ca.frc6390.athena.mechanisms.examples;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.mechanisms.examples.v2.ArmMechanismExamples;
import ca.frc6390.athena.mechanisms.examples.v2.ElevatorMechanismExamples;
import ca.frc6390.athena.mechanisms.examples.v2.FlywheelMechanismExamples;
import ca.frc6390.athena.mechanisms.examples.v2.SimpleMotorMechanismExamples;
import ca.frc6390.athena.mechanisms.examples.v2.TurretMechanismExamples;

final class MechanismV2AuthoringExamplesTest {
    @Test
    void annotationPrimaryExamplesProduceDefinitions() {
        assertEquals("SimpleRoller", SimpleMotorMechanismExamples.definition().name());
        assertEquals("ShooterFlywheel", FlywheelMechanismExamples.definition().name());
        assertEquals("ArmPivot", ArmMechanismExamples.definition().name());
        assertEquals("Elevator", ElevatorMechanismExamples.definition().name());
        assertEquals("Turret", TurretMechanismExamples.definition().name());
    }

    @Test
    void annotationPrimaryExamplesStayDefinitionComplete() {
        assertNotNull(SimpleMotorMechanismExamples.definition().initialStateName().orElse(null));
        assertNotNull(FlywheelMechanismExamples.definition().initialStateName().orElse(null));
        assertNotNull(ArmMechanismExamples.definition().identity().travelRange().orElse(null));
        assertNotNull(ElevatorMechanismExamples.definition().identity().travelRange().orElse(null));
        assertNotNull(TurretMechanismExamples.definition().encoders().stream()
            .filter(encoder -> encoder.defaultPositionSource())
            .findFirst()
            .orElse(null));
    }
}
