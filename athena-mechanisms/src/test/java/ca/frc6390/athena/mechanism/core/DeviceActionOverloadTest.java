package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import org.junit.jupiter.api.Test;

/**
 * Hardware declaration actions resolve through every {@code Runnable} / {@code ActionBinding} overload
 * pair, and compose with {@code then} into Action values without casts.
 */
class DeviceActionOverloadTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).statorCurrentLimit(40);
    private static final EncoderDevice ENCODER = MOTOR.encoder();

    @Test
    void doOnceAndRunAcceptDeclarationsAsMechanismActions() {
        DeviceAction set = ENCODER.setPosition(0.0);
        Action once = Actions.doOnce(ENCODER.setPosition(0.0));
        Action run = Actions.run(ENCODER.setPosition(0.0));
        assertInstanceOf(Actions.EncoderSetPosition.class, once);
        assertInstanceOf(Actions.EncoderSetPosition.class, run);
        assertSame(set.getClass(), once.getClass());
    }

    @Test
    void sequenceDoOnceAndRunAcceptDeclarations() {
        Actions.Sequence once = Actions.sequence().doOnce(ENCODER.setPosition(0.0));
        Actions.Sequence run = Actions.sequence().run(ENCODER.setPosition(0.0));
        assertEquals(1, once.steps().size());
        assertEquals(1, run.steps().size());
    }

    @Test
    void actionThenDeclarationComposesIntoAction() {
        Action percent = MOTOR.percent(0.3);
        Action composed = percent.then(ENCODER.setPosition(0.0));
        Actions.Then then = assertInstanceOf(Actions.Then.class, composed);
        assertInstanceOf(Actions.EncoderSetPosition.class, then.next());
        assertInstanceOf(Actions.MotorPercent.class, then.action());
    }

    @Test
    void declarationThenDeclarationStillComposes() {
        Action composed = MOTOR.percent(0.3).then(MOTOR.neutral());
        Actions.Then then = assertInstanceOf(Actions.Then.class, composed);
        assertInstanceOf(Actions.MotorNeutral.class, then.next());
    }

    @Test
    void sequenceThenDeclarationComposesIntoAction() {
        Action composed = Actions.sequence().run(MOTOR.percent(0.2)).then(ENCODER.setPosition(0));
        assertInstanceOf(Actions.Then.class, composed);
    }

    @Test
    void chainedActionMethodsAfterDeclarationCompose() {
        Action base = MOTOR.percent(0.1);
        // Action-only methods must remain callable after a declaration is composed in.
        Action chained = base.then(ENCODER.setPosition(0.0)).then(MOTOR.neutral()).timeout(2.0).until(() -> true);
        assertInstanceOf(Action.class, chained);
    }

    @Test
    void untilStalledKeepsActionType() {
        Action base = MOTOR.percent(-0.2);
        Action stalled = base.untilStalled(MOTOR.stall().atCurrentLimit(0.9)).then(ENCODER.setPosition(0.0));
        assertInstanceOf(Actions.Then.class, stalled);
    }
}
