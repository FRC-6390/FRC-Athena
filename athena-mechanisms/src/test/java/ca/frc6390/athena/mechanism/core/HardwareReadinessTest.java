package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.ConfigurationStatus;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.runtime.MappedActionContext;
import ca.frc6390.athena.hardware.signal.ImuSource;
import edu.wpi.first.units.Units;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class HardwareReadinessTest {
    private static final ConfigurationStatus PENDING = new ConfigurationStatus(1, 0, true, "");

    @Test
    void sharedDeclarationsControlsAndAliasedHandlesAreResolvedAndCheckedOnce() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 91);
        MotorDevice alias = MotorDevice.of(MotorKinds.KRAKEN_X60, 91);
        Motor handle = new Motor(motor);
        int[] lookups = {0};
        ActionContext context = new ActionContext() {
            @Override public long bindingRevision() { return 0; }
            @Override public MotorHandle motor(MotorDevice ignored) { lookups[0]++; return handle; }
        };
        HardwareReadiness readiness = new HardwareReadiness(context, List.of(motor, motor, alias,
                Controls.position(motor, Units.Rotations), Controls.velocity(motor, Units.Rotations)));

        for (int index = 0; index < 50; index++) assertTrue(readiness.ready());

        assertEquals(2, lookups[0], "one resolution per distinct declaration identity");
        assertEquals(50, handle.checks, "aliases sharing a handle must not multiply acknowledgement checks");
    }

    @Test
    void configurationBecomingPendingInvalidatesReadinessWithoutReplacingTheHandle() {
        Motor handle = new Motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 92));
        int[] lookups = {0};
        HardwareReadiness readiness = new HardwareReadiness(new ActionContext() {
            @Override public long bindingRevision() { return 0; }
            @Override public MotorHandle motor(MotorDevice ignored) { lookups[0]++; return handle; }
        }, List.of(handle.device));
        assertTrue(readiness.ready());
        handle.status = PENDING;
        assertFalse(readiness.ready(), "a hook or earlier parallel branch can enqueue config within this cycle");
        handle.status = new ConfigurationStatus(1, 1, false, "");
        assertTrue(readiness.ready());
        assertEquals(1, lookups[0]);
    }

    @Test
    void replacementDuringResolutionIsRetriedBeforeReportingReady() {
        Motor first = new Motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 93));
        Motor replacement = new Motor(first.device);
        replacement.status = PENDING;
        int[] lookups = {0};
        long[] revision = {0};
        HardwareReadiness readiness = new HardwareReadiness(new ActionContext() {
            @Override public long bindingRevision() { return revision[0]; }
            @Override public MotorHandle motor(MotorDevice ignored) {
                if (lookups[0]++ == 0) { revision[0]++; return first; }
                return replacement;
            }
        }, List.of(first.device));
        assertFalse(readiness.ready(), "the old ready handle must not survive a concurrent binding replacement");
        replacement.status = ConfigurationStatus.READY;
        assertTrue(readiness.ready());
        assertEquals(2, lookups[0]);
    }

    @Test
    void unversionedPartialContextsRetryUnresolvedAndReplacedBindings() {
        Motor handle = new Motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 94));
        Motor[] installed = {null};
        int[] lookups = {0};
        HardwareReadiness readiness = new HardwareReadiness(new ActionContext() {
            @Override public MotorHandle motor(MotorDevice ignored) {
                lookups[0]++;
                if (installed[0] == null) throw new UnsupportedOperationException("partial context");
                return installed[0];
            }
        }, List.of(handle.device));
        assertTrue(readiness.ready());
        installed[0] = handle;
        handle.status = PENDING;
        assertFalse(readiness.ready());
        installed[0] = new Motor(handle.device);
        assertTrue(readiness.ready());
        assertEquals(3, lookups[0], "contexts without a revision promise must always resolve current handles");
    }

    @Test
    void compiledImuDependenciesDoNotAllocateTheirListAtEveryFence() {
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 95);
        CountingSource source = new CountingSource(imu);
        int[] lookups = {0};
        int[] checks = {0};
        HardwareReadiness readiness = new HardwareReadiness(new ActionContext() {
            @Override public long bindingRevision() { return 0; }
            @Override public ImuHandle imu(ImuDevice device) {
                lookups[0]++;
                return new ImuHandle() {
                    @Override public ImuDevice device() { return device; }
                    @Override public double yawDegrees() { return 0; }
                    @Override public ConfigurationStatus configurationStatus() { checks[0]++; return ConfigurationStatus.READY; }
                };
            }
        }, List.of(source, imu));
        for (int index = 0; index < 50; index++) assertTrue(readiness.ready());
        assertEquals(1, source.dependencyReads);
        assertEquals(1, lookups[0]);
        assertEquals(50, checks[0]);
    }

    @Test
    void schedulerOverlayInvalidatesEveryRegisteredDeviceKind() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 96);
        EncoderDevice encoder = motor.encoder();
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 97);
        MechanismScheduler scheduler = MechanismScheduler.create();
        Motor motorHandle = new Motor(motor);
        EncoderHandle encoderHandle = encoder(encoder, ConfigurationStatus.READY);
        ImuHandle imuHandle = imu(imu, ConfigurationStatus.READY);
        scheduler.motor(motor, motorHandle).encoder(encoder, encoderHandle).imu(imu, imuHandle);
        HardwareReadiness readiness = new HardwareReadiness(scheduler.actionContext(), List.of(motor, encoder, imu));
        assertTrue(readiness.ready());
        Motor replacement = new Motor(motor);
        replacement.status = PENDING;
        scheduler.motor(motor, replacement);
        assertFalse(readiness.ready());
        replacement.status = ConfigurationStatus.READY;
        assertTrue(readiness.ready());
        scheduler.encoder(encoder, encoder(encoder, PENDING));
        assertFalse(readiness.ready());
        scheduler.encoder(encoder, encoderHandle).imu(imu, imu(imu, PENDING));
        assertFalse(readiness.ready());
        scheduler.imu(imu, imuHandle);
        assertTrue(readiness.ready());
    }

    @Test
    void graphPlaceholderRecoveryAndCloseInvalidateCachedBindings() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 98);
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        MotorBackend backend = new MotorBackend() {
            @Override public boolean supports(MotorKind kind) { return true; }
            @Override public MotorHandle create(MotorDevice device) {
                entered.countDown();
                try { release.await(); }
                catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                return new Motor(device);
            }
        };
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend)).asynchronousBindings();
        try {
            HardwareReadiness readiness = new HardwareReadiness(graph, List.of(motor));
            assertFalse(readiness.ready());
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            assertFalse(readiness.ready());
            release.countDown();
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
            while (!readiness.ready() && System.nanoTime() < deadline) Thread.sleep(1);
            assertTrue(readiness.ready(), "the public readiness view must observe background installation");
            graph.close();
            assertThrows(IllegalStateException.class, readiness::ready);
        } finally {
            release.countDown();
            graph.close();
        }
    }

    @Test
    void hookConfigurationPausesActionClockAndAcknowledgementResumesIt() {
        Motor handle = new Motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 99));
        ClockAxis axis = new ClockAxis(handle.device, () -> handle.status = PENDING);
        MappedActionContext context = new MappedActionContext().motor(handle.device, handle);
        MechanismRuntime runtime = MechanismRuntime.of(axis, context);
        runtime.set(Actions.waitSeconds(Units.Seconds.of(0.1)).then(Actions.doOnce(() -> axis.completed++)));
        runtime.periodic(at(0.0), EventContext.empty());
        assertEquals(0, axis.completed, "the hook's pending config must fence action evaluation in this cycle");
        handle.status = ConfigurationStatus.READY;
        runtime.periodic(at(5.0), EventContext.empty());
        assertEquals(0, axis.completed, "pending time must not consume the wait");
        runtime.periodic(at(5.1), EventContext.empty());
        assertEquals(1, axis.completed);
    }

    @Test
    void dynamicallyRequestedDeviceActionKeepsItsOwnAcknowledgementFence() {
        EncoderDevice encoder = MotorDevice.of(MotorKinds.KRAKEN_X60, 100).encoder();
        ConfigurationStatus[] status = {ConfigurationStatus.READY};
        int[] mutations = {0};
        int[] completed = {0};
        MappedActionContext context = new MappedActionContext().encoder(encoder, new EncoderHandle() {
            @Override public EncoderDevice device() { return encoder; }
            @Override public ConfigurationStatus configurationStatus() { return status[0]; }
            @Override public double positionRotations() { return 0; }
            @Override public void setPositionRotations(double value) { mutations[0]++; status[0] = PENDING; }
        });
        // This device is deliberately absent from mechanism declarations and introduced by a runtime request.
        MechanismRuntime runtime = MechanismRuntime.of(new Mechanism() { }, context);
        runtime.set(Actions.setPosition(encoder, 0).then(Actions.doOnce(() -> completed[0]++)));
        runtime.periodic(at(0), EventContext.empty());
        runtime.periodic(at(5), EventContext.empty());
        assertEquals(1, mutations[0]);
        assertEquals(0, completed[0]);
        status[0] = new ConfigurationStatus(1, 1, false, "");
        runtime.periodic(at(5.02), EventContext.empty());
        assertEquals(1, completed[0]);
    }

    private static MechanismContext at(double seconds) {
        return new MechanismContext(seconds, 0, 0.02, true, false, false);
    }

    private static EncoderHandle encoder(EncoderDevice device, ConfigurationStatus status) {
        return new EncoderHandle() {
            @Override public EncoderDevice device() { return device; }
            @Override public ConfigurationStatus configurationStatus() { return status; }
            @Override public double positionRotations() { return 0; }
        };
    }

    private static ImuHandle imu(ImuDevice device, ConfigurationStatus status) {
        return new ImuHandle() {
            @Override public ImuDevice device() { return device; }
            @Override public double yawDegrees() { return 0; }
            @Override public ConfigurationStatus configurationStatus() { return status; }
        };
    }

    private static final class Motor implements MotorHandle {
        final MotorDevice device;
        ConfigurationStatus status = ConfigurationStatus.READY;
        int checks;
        Motor(MotorDevice device) { this.device = device; }
        @Override public MotorDevice device() { return device; }
        @Override public ConfigurationStatus configurationStatus() { checks++; return status; }
    }

    private static final class ClockAxis implements Mechanism {
        final MotorDevice motor;
        final HookBinding configure;
        int completed;
        ClockAxis(MotorDevice motor, Runnable configure) {
            this.motor = motor;
            this.configure = Events.when(() -> true).active().onStart(configure);
        }
    }

    private static final class CountingSource implements ImuSource {
        final ImuDevice imu;
        int dependencyReads;
        CountingSource(ImuDevice imu) { this.imu = imu; }
        @Override public List<?> dependencies() { dependencyReads++; return List.of(imu); }
        @Override public double readYawDegrees() { return 0; }
        @Override public double readPitchDegrees() { return 0; }
        @Override public double readRollDegrees() { return 0; }
        @Override public double readAngleDegrees() { return 0; }
        @Override public double readYawRateDegreesPerSecond() { return 0; }
        @Override public double readLinearAccelerationXG() { return 0; }
        @Override public double readLinearAccelerationYG() { return 0; }
        @Override public double readLinearAccelerationZG() { return 0; }
        @Override public void writeYawDegrees(ActionContext context, double degrees) { }
    }
}
