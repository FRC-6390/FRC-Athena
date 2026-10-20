package ca.frc6390.athena.mechanism.effort;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Retarget;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

/** Effort limits (B5) and motion policies (B4) are independent fields of one control Action. */
class EffortPolicyCompositionTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

    @Test
    void retargetAfterEffortKeepsTheEffortLimit() {
        ControlBinding<?> control = Controls.position(MOTOR, Units.Rotations);
        Action action = control.position(Units.Rotations.of(1.0)).effort(Units.Amps.of(10)).retarget(Retarget.IMMEDIATE);

        Actions.ControlPosition position = assertInstanceOf(Actions.ControlPosition.class, action);
        assertEquals(Retarget.IMMEDIATE, position.policy().retarget());
        assertTrue(position.effort().limited());
    }

    @Test
    void effortAfterRetargetKeepsTheMotionPolicy() {
        ControlBinding<?> control = Controls.velocity(MOTOR, Units.Rotations);
        Action action = control.velocity(Units.RotationsPerSecond.of(5.0)).retarget(Retarget.IMMEDIATE).effort(Units.Amps.of(10));

        Actions.ControlVelocity velocity = assertInstanceOf(Actions.ControlVelocity.class, action);
        assertEquals(Retarget.IMMEDIATE, velocity.policy().retarget());
        assertTrue(velocity.effort().limited());
    }
}
