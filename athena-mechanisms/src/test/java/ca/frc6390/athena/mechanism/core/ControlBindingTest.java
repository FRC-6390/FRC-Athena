package ca.frc6390.athena.mechanism.core;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import edu.wpi.first.units.AngleUnit;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.motion.MotionProfiles;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import java.util.List;
import org.junit.jupiter.api.Test;

class ControlBindingTest {
    @Test
    void limitDeclarationsExistOnlyOnConstraintsApi() {
        assertThrows(NoSuchMethodException.class,
                () -> ControlBinding.class.getMethod("positionRange", double.class, double.class));
        assertThrows(NoSuchMethodException.class,
                () -> ControlBinding.class.getMethod("motionLimits", double.class, double.class));
        assertThrows(NoSuchMethodException.class,
                () -> ControlBinding.class.getMethod("maxOutput", double.class));
    }

    @Test
    void controlFactoriesAcceptAnyNumberOfMotors() {
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice second = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        MotorDevice third = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);

        ControlBinding<?> position = Controls.position(first, Units.Rotations).followers(second, third);
        ControlBinding<?> velocity = Controls.velocity(first, Units.Rotations).followers(second, third);

        assertEquals(List.of(first, second, third), position.motors());
        assertEquals(List.of(first, second, third), velocity.motors());
        assertSame(first, position.output());
        assertEquals(List.of(second, third), position.followers());
    }

    @Test
    void positionFactoryAcceptsScalarSinkAndContinuousInput() {
        RobotVelocityPool.AngularChannel angular = new RobotVelocityPool().angularChannel();

        ControlBinding<?> control = Controls.position(angular, Units.Rotations)
                .feedback(() -> 0.0, () -> 0.0)
                .continuous(Units.Rotations.of(-Math.PI), Units.Rotations.of(Math.PI))
                .constraint(Constraints.maxVoltage(Units.Volts.of(2.0)));

        assertSame(angular, control.sink());
        assertEquals(-Math.PI, control.continuousRange().raw().minimumInUnit(), 1.0e-9);
        assertEquals(Math.PI, control.continuousRange().raw().maximumInUnit(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> control.continuous(Units.Rotations.of(1.0), Units.Rotations.of(1.0)));
        assertThrows(IllegalStateException.class,
                () -> Controls.velocity(angular, Units.Rotations).continuous(Units.RotationsPerSecond.of(-Math.PI), Units.RotationsPerSecond.of(Math.PI)));
    }

    @Test
    void motorsArePrecomputedAndImmutable() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        ControlBinding<?> control = new ControlBinding<>(ControlMode.POSITION, null, null, null, null, null)
                .output(leader)
                .follower(follower);

        List<MotorDevice> motors = control.motors();

        assertSame(motors, control.motors());
        assertEquals(List.of(leader, follower), motors);
        assertThrows(UnsupportedOperationException.class, () -> motors.add(MotorDevice.of(MotorKinds.KRAKEN_X60, 3)));
    }

    @Test
    void slotIsClampedAndPreservedAcrossBindingUpdates() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);

        ControlBinding<?> control = Controls.position(leader, Units.Rotations)
                .slot(-1)
                .follower(follower)
                .slot(3)
                .pid(0.1, 0.0, 0.0);

        assertEquals(3, control.slot());
        assertEquals(List.of(leader, follower), control.motors());
    }

    @Test
    void ordinaryEncoderFeedbackBindsBothChannelsWithoutExtraApiLayer() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2);

        ControlBinding<?> control = Controls.position(motor, Units.Rotations).feedback(encoder);

        assertSame(encoder, ((EncoderDevice.PositionFeedback) control.feedback().position()).encoder());
        assertSame(encoder, ((EncoderDevice.VelocityFeedback) control.feedback().velocity()).encoder());
        assertEquals(java.util.Set.of(encoder), control.feedback().encoders());
    }

    @Test
    void explicitFeedbackBindingKeepsMixedChannelsDistinct() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        PositionSignal position = () -> 12.0;
        VelocitySignal velocity = () -> -3.0;
        FeedbackBinding feedback = new FeedbackBinding(position, velocity);

        ControlBinding<?> control = Controls.position(motor, Units.Rotations).feedback(feedback);

        assertSame(feedback, control.feedback());
        assertSame(position, control.feedback().position());
        assertSame(velocity, control.feedback().velocity());
    }

    @Test
    void positionSignalFeedbackDoesNotRequireSyntheticVelocityAtCallSite() {
        Object dependency = new Object();
        PositionSignal position = new PositionSignal() {
            @Override public double position() { return 1.25; }
            @Override public List<?> dependencies() { return List.of(dependency); }
        };

        ControlBinding<?> control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 1), Units.Rotations)
                .feedback(position);

        assertSame(position, control.feedback().position());
        assertEquals(0.0, control.feedback().velocity().velocity(), 1.0e-9);
        assertEquals(java.util.Set.of(dependency), control.feedback().dependencies());
    }

    @Test
    void measurementsAndTargetConditionsUseConfiguredFeedback() {
        double[] position = {0.08};
        double[] velocity = {1.5};
        ControlBinding<?> control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 1), Units.Rotations)
                .feedback(() -> position[0], () -> velocity[0]);
        var atExtended = control.at(Units.Rotations.of(0.10), Units.Rotations.of(0.025));

        assertEquals(0.08, control.position(), 1.0e-9);
        assertEquals(1.5, control.velocity(), 1.0e-9);
        assertEquals(0.08, control.measurement(), 1.0e-9);
        assertEquals(0.02, control.error(Units.Rotations.of(0.10)).magnitude(), 1.0e-9);
        assertEquals(true, control.isAt(Units.Rotations.of(0.10), Units.Rotations.of(0.025)));
        assertEquals(true, atExtended.getAsBoolean());

        position[0] = 0.05;
        assertEquals(false, atExtended.getAsBoolean());
    }

    @Test
    void velocityConditionsUseVelocityFeedbackAndValidateConfiguration() {
        ControlBinding<?> velocity = Controls.velocity(MotorDevice.of(MotorKinds.KRAKEN_X60, 1), Units.Rotations)
                .feedback(() -> 4.0, () -> 19.5);
        ControlBinding<?> unconfigured = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 2), Units.Rotations);

        assertEquals(19.5, velocity.measurement(), 1.0e-9);
        assertEquals(true, velocity.isAt(Units.RotationsPerSecond.of(20.0), Units.RotationsPerSecond.of(0.5)));
        assertThrows(IllegalStateException.class, unconfigured::position);
        assertThrows(IllegalArgumentException.class, () -> velocity.at(Units.RotationsPerSecond.of(20.0), Units.RotationsPerSecond.of(-0.1)));
        assertThrows(IllegalArgumentException.class, () -> velocity.isAt(Units.RotationsPerSecond.of(Double.NaN), Units.RotationsPerSecond.of(1.0)));
        var dynamicTarget = velocity.at(() -> Double.NaN, Units.Rotations, Units.RotationsPerSecond.of(1.0));
        assertThrows(IllegalArgumentException.class, dynamicTarget::getAsBoolean);
        assertThrows(NullPointerException.class,
                () -> velocity.at((java.util.function.DoubleSupplier) null, Units.Rotations, Units.RotationsPerSecond.of(1.0)));
    }

    @Test
    void pidRejectsInvalidIntegralConfiguration() {
        assertThrows(IllegalArgumentException.class, () -> PidGains.of(0.0, Double.NaN, 0.0));
        assertThrows(IllegalArgumentException.class, () -> PidGains.of(0.0, 1.0, 0.0).iZone(-1.0));
    }

    @Test
    void gainConfigurationReplacesSameGainTypeAndKeepsCustomLoops() {
        ControlLoop custom = binding -> context -> ControlOutput.voltage(0.25);
        PidGains finalPid = PidGains.of(2.0, 0.5, 0.1).iZone(3.0);
        FeedforwardGains finalFeedforward = FeedforwardGains.of(0.2, 1.5, 0.4, 0.6);

        ControlBinding<?> control = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 1), Units.Rotations)
                .loop(custom)
                .pid(1.0, 0.0, 0.0)
                .ff(0.1, 1.0, 0.2)
                .pid(finalPid)
                .feedforward(finalFeedforward);

        assertEquals(3, control.loops().size());
        assertSame(custom, control.loops().get(0));
        assertSame(finalPid, control.loops().get(1));
        assertSame(finalFeedforward, control.loops().get(2));
    }

    @Test
    void feedforwardGainsExposeAllTermsAndRejectInvalidValues() {
        FeedforwardGains gains = FeedforwardGains.of(0.2, 1.5, 0.4, 0.6);

        assertEquals(0.2, gains.staticGain(), 1.0e-9);
        assertEquals(1.5, gains.velocityGain(), 1.0e-9);
        assertEquals(0.4, gains.accelerationGain(), 1.0e-9);
        assertEquals(0.6, gains.gravityGain(), 1.0e-9);
        assertEquals(0.8, gains.acceleration(0.8).accelerationGain(), 1.0e-9);
        assertThrows(IllegalArgumentException.class,
                () -> FeedforwardGains.of(Double.NaN, 0.0, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> FeedforwardGains.of(0.0, Double.POSITIVE_INFINITY, 0.0, 0.0));
    }

    @Test
    void targetFactoriesEnforceControlModeAndFiniteValues() {
        ControlBinding<?> position = Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 1), Units.Rotations);
        ControlBinding<?> velocity = Controls.velocity(MotorDevice.of(MotorKinds.KRAKEN_X60, 2), Units.Rotations);

        assertInstanceOf(Actions.ControlPosition.class, position.set(Units.Rotations.of(2.0)));
        assertInstanceOf(Actions.ControlVelocity.class, velocity.set(Units.RotationsPerSecond.of(3.0)));
        assertThrows(IllegalStateException.class, () -> position.velocity(Units.Rotations.of(1.0)));
        assertThrows(IllegalStateException.class, () -> velocity.position(Units.RotationsPerSecond.of(1.0)));
        assertThrows(IllegalArgumentException.class, () -> position.position(Units.Rotations.of(Double.NaN)));
        assertThrows(IllegalArgumentException.class, () -> velocity.velocity(Units.RotationsPerSecond.of(Double.POSITIVE_INFINITY)));
        assertThrows(IllegalArgumentException.class, () -> position.percent(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> position.voltage(Units.Volts.of(Double.NEGATIVE_INFINITY)));

        Output.Position dynamic = assertInstanceOf(
                Output.Position.class,
                position.position(() -> Double.NaN, Units.Rotations));
        assertThrows(IllegalArgumentException.class, dynamic::position);
        assertThrows(IllegalStateException.class,
                () -> new Actions.ControlVelocity(position, 1.0));
        assertThrows(IllegalArgumentException.class,
                () -> new Actions.ControlPosition(position, Double.NaN));
        assertThrows(IllegalArgumentException.class,
                () -> new Actions.DynamicControlVoltage(position, () -> Double.NaN).volts());
    }

    @Test
    void bindingCopiesPreserveConfigurationWithoutSharingMutableCollections() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 3);
        ControlBinding<?> original = Controls.position(motor, Units.Rotations)
                .feedback(encoder)
                .pid(0.2, 0.01, 0.03)
                .ff(0.1, 0.2, 0.3)
                .constraint(Constraints.range(ca.frc6390.athena.hardware.device.Range.of(Units.Value.of(-1.0), Units.Value.of(1.0))))
                .dependency(encoder);

        ControlBinding<?> copied = original.follower(follower).slot(2);

        assertEquals(List.of(), original.followers());
        assertEquals(List.of(follower), copied.followers());
        assertSame(original.feedback(), copied.feedback());
        assertEquals(original.loops(), copied.loops());
        assertEquals(original.dependencies(), copied.dependencies());
        assertEquals(original.constraints(), copied.constraints());
        assertThrows(UnsupportedOperationException.class, () -> copied.loops().clear());
        assertThrows(UnsupportedOperationException.class, () -> copied.dependencies().clear());
        assertThrows(UnsupportedOperationException.class, () -> copied.constraints().clear());
    }

    @Test
    void canonicalConstructorAlwaysDerivesMotorTargets() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        MotorDevice unrelated = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);

        ControlBinding<?> control = new ControlBinding<>(
                ControlMode.POSITION,
                leader,
                0,
                List.of(follower),
                null,
                List.of(),
                List.of(),
                List.of(),
                null,
                null,
                List.of(unrelated));

        assertEquals(List.of(leader, follower), control.motors());
    }

    @Test
    void motorTargetsRejectSelfFollowingAndDuplicates() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        ControlBinding<?> control = Controls.position(leader, Units.Rotations).follower(follower);

        assertThrows(IllegalArgumentException.class, () -> Controls.position(leader, Units.Rotations).follower(leader));
        assertThrows(IllegalArgumentException.class, () -> control.follower(follower));
        assertThrows(IllegalArgumentException.class, () -> control.output(follower));
    }

    @Test
    void incompleteAndNullConfigurationsFailClearly() {
        ControlBinding<?> incomplete = new ControlBinding<>(ControlMode.POSITION, null, 0, null, null, null, null,
                null, null, null, null, false, null, null, Units.Rotations, null, null, null);

        assertThrows(NullPointerException.class, () -> Controls.position(MotorDevice.of(MotorKinds.KRAKEN_X60, 3), (AngleUnit) null));
        assertThrows(IllegalStateException.class, () -> incomplete.position(Units.Rotations.of(1.0)));
        assertThrows(IllegalStateException.class, incomplete::neutral);
        assertThrows(NullPointerException.class, () -> incomplete.followers((MotorDevice[]) null));
        assertThrows(NullPointerException.class, () -> incomplete.dependencies((Object[]) null));
        assertThrows(NullPointerException.class, () -> incomplete.loops((ControlLoop[]) null));
        assertThrows(NullPointerException.class,
                () -> incomplete.constraints((ca.frc6390.athena.mechanism.constraint.Constraint<Double>[]) null));
    }

    @Test
    void positionOnlyFeaturesRejectVelocityBindings() {
        ControlBinding<?> velocity = Controls.velocity(MotorDevice.of(MotorKinds.KRAKEN_X60, 1), Units.Rotations);

        assertThrows(IllegalStateException.class,
                () -> velocity.profile(MotionProfiles.trapezoid(1.0, 1.0)));
        assertThrows(IllegalStateException.class,
                () -> velocity.planner((context, constraints) -> Constraints.evaluate(constraints, context)));
    }

    @Test
    void absoluteEncoderSignalReadsAbsoluteChannelAndRetainsDependency() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2).offset(0.25);
        PositionSignal absolute = encoder.absoluteSignal();
        EncoderHandle handle = new EncoderHandle() {
            @Override
            public EncoderDevice device() {
                return encoder;
            }

            @Override
            public double positionRotations() {
                return 7.0;
            }

            @Override
            public double absolutePositionRotations() {
                return 0.75;
            }
        };
        encoder.bindRuntime(new RuntimeScope("absolute-test"), handle);

        assertEquals(0.5, absolute.position(), 1.0e-9);
        assertEquals(List.of(encoder), absolute.dependencies());
    }
}
