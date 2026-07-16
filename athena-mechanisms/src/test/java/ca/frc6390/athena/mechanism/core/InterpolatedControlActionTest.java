package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import ca.frc6390.athena.mechanism.interpolation.InterpolationModel;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class InterpolatedControlActionTest {
    @Test
    void linearInterpolationSortsPointsInterpolatesAndClamps() {
        double[] input = {2.0};
        ControlBinding control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 1));
        InterpolatedControlAction action = control.interpolate(InterpolationKinds.LINEAR, () -> input[0])
                .at(3.0, 30.0)
                .at(1.0, 10.0);

        assertEquals(20.0, action.target(), 1.0e-9);
        input[0] = -5.0;
        assertEquals(10.0, action.target(), 1.0e-9);
        input[0] = 8.0;
        assertEquals(30.0, action.target(), 1.0e-9);
    }

    @Test
    void customModelReceivesTheCompleteTableAndDynamicValuesAreSampledOnce() {
        AtomicInteger inputSamples = new AtomicInteger();
        AtomicInteger firstSamples = new AtomicInteger();
        AtomicInteger secondSamples = new AtomicInteger();
        InterpolationModel custom = (input, data) ->
                data.value(0) + data.value(0) + data.value(data.size() - 1) + input;
        InterpolatedControlAction action = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 2))
                .interpolate(custom, () -> {
                    inputSamples.incrementAndGet();
                    return 4.0;
                })
                .at(1.0, () -> {
                    firstSamples.incrementAndGet();
                    return 2.0;
                })
                .at(3.0, () -> {
                    secondSamples.incrementAndGet();
                    return 5.0;
                });

        assertEquals(13.0, action.target(), 1.0e-9);
        assertEquals(1, inputSamples.get());
        assertEquals(1, firstSamples.get());
        assertEquals(1, secondSamples.get());
    }

    @Test
    void dynamicPointsAreSampledOnceAndResortedForEveryEvaluation() {
        double[] firstPoint = {1.0};
        double[] secondPoint = {3.0};
        AtomicInteger firstSamples = new AtomicInteger();
        AtomicInteger secondSamples = new AtomicInteger();
        InterpolatedControlAction action = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 7))
                .interpolate(InterpolationKinds.LINEAR, () -> 2.0)
                .at(() -> {
                    firstSamples.incrementAndGet();
                    return firstPoint[0];
                }, 10.0)
                .at(() -> {
                    secondSamples.incrementAndGet();
                    return secondPoint[0];
                }, 30.0);

        assertEquals(20.0, action.target(), 1.0e-9);
        assertEquals(1, firstSamples.get());
        assertEquals(1, secondSamples.get());

        firstPoint[0] = 4.0;
        secondPoint[0] = 0.0;
        assertEquals(20.0, action.target(), 1.0e-9);
        assertEquals(2, firstSamples.get());
        assertEquals(2, secondSamples.get());

        secondPoint[0] = 4.0;
        assertThrows(IllegalArgumentException.class, action::target);
    }

    @Test
    void outputResolutionUsesTheControlBindingsTargetMode() {
        MotorDevice positionMotor = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);
        MotorDevice velocityMotor = MotorDevice.of(MotorKinds.KRAKEN_X60, 4);
        InterpolatedControlAction position = Controls.position(positionMotor)
                .interpolate(InterpolationKinds.LINEAR, () -> 0.5)
                .at(0.0, 2.0)
                .at(1.0, 4.0);
        InterpolatedControlAction velocity = Controls.velocity(velocityMotor)
                .interpolate(InterpolationKinds.LINEAR, () -> 0.5)
                .at(0.0, 20.0)
                .at(1.0, 40.0);
        OutputResolver resolver = OutputResolver.empty();
        Mechanism mechanism = new Mechanism() { };

        List<ResolvedOutput> positionOutputs = resolver.resolve(mechanism, position);
        List<ResolvedOutput> velocityOutputs = resolver.resolve(mechanism, velocity);

        assertEquals(3.0, assertInstanceOf(Output.Position.class, positionOutputs.get(0).output()).position(), 1.0e-9);
        assertEquals(30.0, assertInstanceOf(Output.Velocity.class, velocityOutputs.get(0).output()).velocity(), 1.0e-9);
    }

    @Test
    void schedulerOwnsAndAppliesInterpolatedControlActions() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 5);
        RecordingMotor handle = new RecordingMotor(motor);
        ControlBinding control = Controls.position(motor)
                .constraint(Constraints.range(Range.of(0.0, 5.0)));
        InterpolatedControlAction action = control.interpolate(InterpolationKinds.LINEAR, () -> 2.0)
                .at(1.0, 4.0)
                .at(3.0, 8.0);
        Mechanism mechanism = new Mechanism() {
            @SuppressWarnings("unused")
            private final ControlBinding position = control;
            @SuppressWarnings("unused")
            private final Action automatic = action;
        };
        ActionContext hardware = new ActionContext() {
            @Override public MotorHandle motor(MotorDevice requested) { return handle; }
        };
        MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(mechanism);

        scheduler.request(action);
        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(5.0, handle.positionTarget, 1.0e-9);
    }

    @Test
    void invalidTablesAndRuntimeValuesFailClearly() {
        ControlBinding control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 6));
        InterpolatedControlAction empty = control.interpolate(InterpolationKinds.LINEAR, () -> 0.0);

        assertThrows(IllegalStateException.class, empty::target);
        assertThrows(IllegalArgumentException.class, () -> empty.at(Double.NaN, 1.0));
        assertThrows(IllegalArgumentException.class, () -> empty.at(1.0, Double.NaN));
        InterpolatedControlAction one = empty.at(1.0, 2.0);
        assertThrows(IllegalArgumentException.class, () -> one.at(1.0, 3.0));
        assertThrows(IllegalArgumentException.class,
                () -> control.interpolate(InterpolationKinds.LINEAR, () -> Double.NaN)
                        .at(0.0, 1.0)
                        .at(1.0, 2.0)
                        .target());
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double positionTarget;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public void setPositionTargetRotations(double rotations) { positionTarget = rotations; }
        @Override public double integratedPositionRotations() { return 0.0; }
        @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
    }
}
