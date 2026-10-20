package ca.frc6390.athena.mechanism.core;

import static edu.wpi.first.units.Units.Seconds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class ActionsTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

    @Test
    void neutralActionIsShared() {
        assertSame(Actions.neutral(), Actions.neutral());
    }

    @Test
    void controlNeutralRetainsItsTargetControl() {
        ControlBinding<?> speed = Controls.velocity(MOTOR, Units.Rotations);

        Actions.ControlNeutral neutral = (Actions.ControlNeutral) speed.neutral();

        assertSame(speed, neutral.control());
    }

    @Test
    void motorNeutralRetainsItsTargetMotor() {
        Actions.MotorNeutral neutral = (Actions.MotorNeutral) MOTOR.neutral();

        assertSame(MOTOR, neutral.motor());
    }

    @Test
    void builderActionAccessorsReturnStableImmutableViews() {
        Action first = MOTOR.percent(0.2);
        Action second = MOTOR.percent(0.4);
        Actions.Parallel parallel = Actions.parallel(first);
        Actions.Sequence sequence = Actions.sequence().run(first).run(second);
        Actions.Cycle cycle = Actions.cycle().forTime(Seconds.of(0.1), first).forTime(Seconds.of(0.2), second);

        assertSame(parallel.Actions(), parallel.Actions());
        assertSame(sequence.steps(), sequence.steps());
        assertSame(cycle.steps(), cycle.steps());
        assertThrows(UnsupportedOperationException.class, () -> parallel.Actions().clear());
        assertThrows(UnsupportedOperationException.class, () -> sequence.steps().clear());
        assertThrows(UnsupportedOperationException.class, () -> cycle.steps().clear());
    }

    @Test
    void deadlineActionsAccessorDoesNotCopyEachCall() {
        Action primary = MOTOR.percent(0.2);
        Action other = MOTOR.voltage(Units.Volts.of(3.0));
        Actions.Deadline deadline = Actions.deadline(primary, other);

        List<Action> actions = deadline.Actions();

        assertSame(actions, deadline.Actions());
        assertEquals(List.of(primary, other), actions);
        assertThrows(UnsupportedOperationException.class, () -> actions.clear());
    }

    @Test
    void hooksAcceptDeviceActionsDirectly() {
        HookBinding hook = Events.when(() -> true).active().onStart(MOTOR.encoder().setPosition(1.0));

        assertEquals(1, hook.actions().size());
    }

    @Test
    void timeoutIsAvailableOnDeviceActionsAndComposedActions() {
        Action timedDeviceAction = MOTOR.percent(0.2).timeout(Seconds.of(0.5));
        Action timedComposition = MOTOR.percent(0.2).then(MOTOR.voltage(Units.Volts.of(3.0))).timeout(Seconds.of(1.0));

        assertEquals(0.5, ((Actions.Timeout) timedDeviceAction).duration().in(Seconds));
        Actions.Timeout timeout = (Actions.Timeout) timedComposition;
        assertEquals(1.0, timeout.duration().in(Seconds));
        assertEquals(Actions.Then.class, timeout.action().getClass());
    }

    @Test
    void timeoutBoundsAFluentAction() {
        // Action.forTime(seconds) was removed: it read like Sequence.forTime(seconds, action) but meant
        // "time out after". Timed sequence steps use forTime(seconds, action); single actions use timeout.
        Actions.Timeout timed = (Actions.Timeout) MOTOR.percent(0.2).timeout(Seconds.of(0.5));

        assertEquals(0.5, timed.duration().in(Seconds));
        assertEquals(MOTOR.percent(0.2).getClass(), timed.action().getClass());
    }

    @Test
    void untilWithinOnlyAcceptsClosedLoopTargetsAndValidTolerance() {
        ControlBinding<?> position = Controls.position(MOTOR, Units.Rotations);
        Action openLoop = MOTOR.percent(0.2);

        assertThrows(IllegalArgumentException.class, () -> openLoop.untilWithin(Units.Rotations.of(0.1)));
        assertThrows(IllegalArgumentException.class, () -> position.position(Units.Rotations.of(1.0)).untilWithin(Units.Rotations.of(-0.1)));
        assertThrows(IllegalArgumentException.class, () -> position.position(Units.Rotations.of(1.0)).untilWithin(Units.Rotations.of(Double.NaN)));
    }

    @Test
    void settleAndVelocityOptionsRequireUntilWithinAndValidValues() {
        ControlBinding<?> position = Controls.position(MOTOR, Units.Rotations);
        Action target = position.position(Units.Rotations.of(1.0));

        assertThrows(IllegalArgumentException.class, () -> target.settled(Seconds.of(0.1)));
        assertThrows(IllegalArgumentException.class, () -> target.maxVelocity(Units.RotationsPerSecond.of(5.0)));
        assertThrows(IllegalArgumentException.class, () -> target.untilWithin(Units.Rotations.of(0.1)).settled(Seconds.of(-0.1)));
        assertThrows(IllegalArgumentException.class, () -> target.untilWithin(Units.Rotations.of(0.1)).settled(Seconds.of(Double.NaN)));
        assertThrows(IllegalArgumentException.class, () -> target.untilWithin(Units.Rotations.of(0.1)).maxVelocity(Units.RotationsPerSecond.of(-1.0)));
        assertThrows(IllegalArgumentException.class, () -> target.untilWithin(Units.Rotations.of(0.1)).maxVelocity(Units.RotationsPerSecond.of(Double.NaN)));
    }

    @Test
    void velocityControlsRejectMaxVelocityButAcceptSettle() {
        ControlBinding<?> velocity = Controls.velocity(MOTOR, Units.Rotations);
        Action target = velocity.velocity(Units.RotationsPerSecond.of(3.0));

        assertThrows(IllegalArgumentException.class, () -> target.untilWithin(Units.RotationsPerSecond.of(0.5)).maxVelocity(Units.RotationsPerSecond.of(1.0)));
        assertEquals(0.1, assertInstanceOf(Actions.WithinTolerance.class,
                target.untilWithin(Units.RotationsPerSecond.of(0.5)).settled(Seconds.of(0.1))).settleSeconds(), 1.0e-9);
    }

    @Test
    void settledOptionsPreserveToleranceAndDefaultsStayLegacy() {
        ControlBinding<?> position = Controls.position(MOTOR, Units.Rotations);
        Action target = position.position(Units.Rotations.of(1.0));

        Actions.WithinTolerance legacy = assertInstanceOf(Actions.WithinTolerance.class, target.untilWithin(Units.Rotations.of(0.5)));
        assertEquals(0.0, legacy.settleSeconds(), 1.0e-9);
        assertTrue(Double.isInfinite(legacy.maxVelocity()));

        Actions.WithinTolerance settled = assertInstanceOf(Actions.WithinTolerance.class,
                target.untilWithin(Units.Rotations.of(0.5)).maxVelocity(Units.RotationsPerSecond.of(2.0)).settled(java.time.Duration.ofMillis(250)));
        assertEquals(0.5, settled.tolerance(), 1.0e-9);
        assertEquals(0.25, settled.settleSeconds(), 1.0e-9);
        assertEquals(2.0, settled.maxVelocity(), 1.0e-9);
    }

    @Test
    void typedMaxVelocityConvertsRateToControlUnitsPerSecond() {
        ControlBinding<?> position = Controls.position(MOTOR, Units.Rotations);
        Actions.WithinTolerance typed = assertInstanceOf(Actions.WithinTolerance.class,
                position.position(Units.Rotations.of(0.5))
                        .untilWithin(Units.Rotations.of(0.01))
                        .maxVelocity(Units.RPM.of(60.0)));

        assertEquals(1.0, typed.maxVelocity(), 1.0e-9);
    }

    private record TestMechanism(Action initial) implements Mechanism {
    }
}
