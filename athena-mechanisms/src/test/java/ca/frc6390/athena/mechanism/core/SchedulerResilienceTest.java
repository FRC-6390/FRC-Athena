package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import java.util.Map;
import org.junit.jupiter.api.Test;

class SchedulerResilienceTest {
    @Test
    void timeoutInterruptsRunningChildOnce() {
        RecordingPath path = new RecordingPath("timeout");
        MechanismRuntime runtime = runtime(path.action.timeout(0.02), path);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, path.interrupts);
    }

    @Test
    void raceInterruptsRunningLoserOnce() {
        RecordingPath path = new RecordingPath("race");
        MechanismRuntime runtime = runtime(Actions.race(Actions.waitSeconds(0.02), path.action), path);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, path.interrupts);
    }

    @Test
    void deadlineInterruptsRunningCompanionOnce() {
        RecordingPath path = new RecordingPath("deadline");
        MechanismRuntime runtime = runtime(Actions.deadline(Actions.waitSeconds(0.02), path.action), path);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        runtime.periodic(contextAt(0.02), EventContext.empty());
        runtime.periodic(contextAt(0.04), EventContext.empty());

        assertEquals(1, path.interrupts);
    }

    @Test
    void replacingComputedChildInterruptsOldRuntime() {
        boolean[] selectSecond = new boolean[1];
        RecordingPath first = new RecordingPath("first");
        RecordingPath second = new RecordingPath("second");
        Action computed = Actions.compute(context -> selectSecond[0] ? second.action : first.action);
        TestMechanism mechanism = new TestMechanism(computed);
        MechanismRuntime runtime = MechanismRuntime.of(mechanism, ActionContext.empty())
                .path(first.action, first)
                .path(second.action, second);
        runtime.set(computed);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        selectSecond[0] = true;
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertEquals(1, first.interrupts);
        assertEquals(0, second.interrupts);
    }

    @Test
    void disappearingPathMarkerInterruptsItsRuntime() {
        boolean[] markerActive = {true};
        PathAction outer = Paths.of("test", "outer");
        RecordingPath marker = new RecordingPath("marker");
        PathRuntime outerRuntime = new PathRuntime() {
            @Override public boolean isFinished(PathAction path, MechanismContext context) { return false; }
            @Override public Map<String, Action> activeMarkers(PathAction path, MechanismContext context) {
                return markerActive[0] ? Map.of("marker", marker.action) : Map.of();
            }
        };
        MechanismRuntime runtime = MechanismRuntime.of(new TestMechanism(outer), ActionContext.empty())
                .path(outer, outerRuntime)
                .path(marker.action, marker);
        runtime.set(outer);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        markerActive[0] = false;
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertEquals(1, marker.interrupts);
    }

    @Test
    void completedVelocityContributionClearsItsChannel() {
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.Channel channel = pool.channel();
        Action contribution = Actions.contributeVelocity(
                channel, () -> RobotVelocity.angular(1.0), Actions.waitSeconds(0.02));
        MechanismRuntime runtime = MechanismRuntime.of(new TestMechanism(contribution), ActionContext.empty());
        runtime.set(contribution);

        runtime.periodic(contextAt(0.0), EventContext.empty());
        assertTrue(channel.isActive());
        runtime.periodic(contextAt(0.02), EventContext.empty());

        assertFalse(channel.isActive());
    }

    @Test
    void finiteComputedRequestCompletesAndLeavesActiveLeaseSet() {
        Action computed = Actions.compute(context -> Actions.doOnce(() -> { }));
        TestMechanism mechanism = new TestMechanism(computed);
        MechanismScheduler scheduler = MechanismScheduler.create().register(mechanism);

        scheduler.request(computed);
        scheduler.robotPeriodic(0.0, 0.02);

        assertFalse(scheduler.isRunning(computed));
        assertTrue(scheduler.isComplete(computed));
        scheduler.robotPeriodic(0.02, 0.02);
        assertEquals(0, scheduler.traceSnapshots().get(0).activeLeaseCount());
    }

    @Test
    void failedEncoderMutationIsReportedOnceWithoutCrashing() {
        FailingMutationContext context = new FailingMutationContext();
        Action mutation = (Action) context.encoder.setPosition(2.0);
        MutationMechanism mechanism = new MutationMechanism(context.encoder, context.imu, mutation);
        MechanismScheduler scheduler = MechanismScheduler.create(context).register(mechanism);

        scheduler.request(mutation);
        scheduler.robotPeriodic(0.0, 0.02);
        scheduler.robotPeriodic(0.02, 0.02);

        assertEquals(1, context.encoderAttempts);
        assertEquals(1, context.failures);
        assertEquals(context.encoder, context.lastFailureDeclaration);
    }

    @Test
    void failedHookImuMutationIsReportedAndDoesNotPoisonEdgeState() {
        FailingMutationContext context = new FailingMutationContext();
        HookBinding hook = Events.when(() -> true).active().onStart(context.imu.setYaw(0.0));
        HookRuntime runtime = new HookRuntime();

        runtime.run(EventContext.empty(), context, hook);
        runtime.run(EventContext.empty(), context, hook);

        assertEquals(1, context.imuAttempts);
        assertEquals(1, context.failures);
        assertEquals(context.imu, context.lastFailureDeclaration);
    }

    private static MechanismRuntime runtime(Action action, RecordingPath path) {
        MechanismRuntime runtime = MechanismRuntime.of(new TestMechanism(action), ActionContext.empty())
                .path(path.action, path);
        runtime.set(action);
        return runtime;
    }

    private static MechanismContext contextAt(double seconds) {
        return new MechanismContext(seconds, 0.0, 0.02, true, false, false);
    }

    private static final class TestMechanism implements Mechanism {
        private final Action initial;

        private TestMechanism(Action initial) {
            this.initial = initial;
        }
    }

    private static final class MutationMechanism implements Mechanism {
        private final EncoderDevice encoder;
        private final ImuDevice imu;
        private final Action mutation;

        private MutationMechanism(EncoderDevice encoder, ImuDevice imu, Action mutation) {
            this.encoder = encoder;
            this.imu = imu;
            this.mutation = mutation;
        }
    }

    private static final class RecordingPath implements PathRuntime {
        private final PathAction action;
        private int interrupts;

        private RecordingPath(String name) {
            action = Paths.of("test", name);
        }

        @Override
        public boolean isFinished(PathAction path, MechanismContext context) {
            return false;
        }

        @Override
        public void end(PathAction path, MechanismContext context, boolean interrupted) {
            if (interrupted) interrupts++;
        }
    }

    private static final class FailingMutationContext implements ActionContext {
        private final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 41);
        private final ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 42);
        private int encoderAttempts;
        private int imuAttempts;
        private int failures;
        private Object lastFailureDeclaration;

        @Override
        public EncoderHandle encoder(EncoderDevice requested) {
            return new EncoderHandle() {
                @Override public EncoderDevice device() { return requested; }
                @Override public void setPositionRotations(double rotations) {
                    encoderAttempts++;
                    throw new IllegalStateException("encoder disconnected");
                }
            };
        }

        @Override
        public ImuHandle imu(ImuDevice requested) {
            return new ImuHandle() {
                @Override public ImuDevice device() { return requested; }
                @Override public double yawDegrees() { return 0.0; }
                @Override public void setYawDegrees(double yawDegrees) {
                    imuAttempts++;
                    throw new IllegalStateException("imu disconnected");
                }
            };
        }

        @Override
        public void hardwareFailure(Object declaration, RuntimeException exception) {
            failures++;
            lastFailureDeclaration = declaration;
        }
    }
}
