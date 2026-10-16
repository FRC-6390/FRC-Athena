package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.runtime.DeviceActionFactories;
import org.junit.jupiter.api.Test;

/**
 * Hardware declarations create their device actions through {@link DeviceActionFactories}, and
 * composition on them returns the mechanism Action types without any reflective lookup.
 */
class DeviceActionCompositionTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

    @Test
    void deviceActionCompositionReturnsMechanismTypesWithoutReflection() {
        assertInstanceOf(MechanismDeviceActions.class, DeviceActionFactories.get());

        Actions.Timeout timeout = assertInstanceOf(Actions.Timeout.class, MOTOR.percent(0.2).timeout(0.5));
        assertEquals(0.5, timeout.seconds());

        Actions.Then then = assertInstanceOf(Actions.Then.class, MOTOR.percent(0.2).then(MOTOR.neutral()));
        assertInstanceOf(Actions.MotorNeutral.class, then.next());

        DeviceAction device = MOTOR.percent(0.2);
        Actions.Then composed = assertInstanceOf(Actions.Then.class, device.then(MOTOR.neutral()));
        assertInstanceOf(Actions.MotorPercent.class, composed.action());
    }

    @Test
    void hardwareDeclarationsReturnTheirMechanismActionTypes() {
        EncoderDevice encoder = MOTOR.encoder();
        assertInstanceOf(Actions.EncoderSetPosition.class, encoder.setPosition(1.0));
        assertInstanceOf(Actions.MotorPercent.class, MOTOR.percent(0.2));
        assertInstanceOf(Actions.DynamicMotorPercent.class, MOTOR.percent(() -> 0.2));
        assertInstanceOf(Actions.MotorVoltage.class, MOTOR.voltage(2.0));
        assertInstanceOf(Actions.MotorNeutral.class, MOTOR.neutral());
    }
}
