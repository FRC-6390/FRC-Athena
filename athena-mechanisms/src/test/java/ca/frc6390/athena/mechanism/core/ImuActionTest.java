package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class ImuActionTest {
    @Test
    void setYawActionUsesRegisteredPhysicalImuHandle() {
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 1);
        RecordingImuHandle handle = new RecordingImuHandle(imu);
        ActionContext context = new ActionContext() {
            @Override
            public ImuHandle imu(ImuDevice requested) {
                return handle;
            }
        };
        ImuMechanism mechanism = new ImuMechanism(imu);
        MechanismScheduler scheduler = MechanismScheduler.create(context).register(mechanism);

        scheduler.request(mechanism.setHeading);
        scheduler.robotPeriodic(0.0, 0.02);

        assertEquals(90.0, handle.yawDegrees, 1.0e-9);
    }

    @Test
    void invertedSourceRetainsPhysicalOwnershipAndTranslatesSetYaw() {
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 1);
        RecordingImuHandle handle = new RecordingImuHandle(imu);
        ActionContext context = new ActionContext() {
            @Override
            public ImuHandle imu(ImuDevice requested) {
                return handle;
            }
        };
        InvertedImuMechanism mechanism = new InvertedImuMechanism(imu);
        MechanismScheduler scheduler = MechanismScheduler.create(context).register(mechanism);

        scheduler.request(mechanism.setHeading);
        scheduler.robotPeriodic(0.0, 0.02);

        assertEquals(-90.0, handle.yawDegrees, 1.0e-9);
        assertEquals(90.0,
                RuntimeHardwareAccess.call(context, mechanism.heading::yawDegrees),
                1.0e-9);
    }

    @Test
    void setYawActionAppliesImmediatelyWhenTriggeredByAHook() {
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 1);
        RecordingImuHandle handle = new RecordingImuHandle(imu);
        ActionContext context = new ActionContext() {
            @Override
            public ImuHandle imu(ImuDevice requested) {
                return handle;
            }
        };
        Action resetHeading = imu.setYaw(0.0);
        HookBinding hook = Events.when(() -> true).active().onStart(resetHeading);
        handle.yawDegrees = 42.0;

        new HookRuntime().run(EventContext.empty(), context, hook);

        assertEquals(0.0, handle.yawDegrees, 1.0e-9);
    }

    @Test
    void dynamicYawIsReadWhenLifecycleHookRuns() {
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 1);
        RecordingImuHandle handle = new RecordingImuHandle(imu);
        AtomicReference<Double> allianceHeading = new AtomicReference<>(0.0);
        HookBinding hook = Events.autonomousInit().onStart(imu.setYaw(allianceHeading::get));
        ActionContext context = new ActionContext() {
            @Override
            public ImuHandle imu(ImuDevice requested) {
                return handle;
            }
        };
        allianceHeading.set(-180.0);

        new HookRuntime().run(
                new EventContext(0.0, 0.02, LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, false),
                context,
                hook);

        assertEquals(-180.0, handle.yawDegrees, 1.0e-9);
    }

    @Test
    void schedulerRunsDynamicYawOnDerivedSourceDuringAutonomousInit() {
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 1);
        RecordingImuHandle handle = new RecordingImuHandle(imu);
        ActionContext context = new ActionContext() {
            @Override
            public ImuHandle imu(ImuDevice requested) {
                return handle;
            }
        };
        AutoHeadingMechanism mechanism = new AutoHeadingMechanism(imu);
        MechanismScheduler scheduler = MechanismScheduler.create(context).register(mechanism);

        RuntimeHardwareAccess.run(context, () -> scheduler.periodic(
                new MechanismContext(0.0, 0.0, 0.02, true, true, false),
                new EventContext(0.0, 0.02, LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, false)));

        assertEquals(-180.0,
                RuntimeHardwareAccess.call(context, mechanism.driverHeading::yawDegrees),
                1.0e-9);
    }

    @Test
    void dynamicAutoHeadingReevaluatesAcrossConsecutiveAutoRuns() {
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 1);
        RecordingImuHandle handle = new RecordingImuHandle(imu);
        ActionContext context = new ActionContext() {
            @Override
            public ImuHandle imu(ImuDevice requested) {
                return handle;
            }
        };
        AtomicReference<Double> allianceHeading = new AtomicReference<>(0.0);
        ReusableAutoHeadingMechanism mechanism = new ReusableAutoHeadingMechanism(imu, allianceHeading);
        MechanismScheduler scheduler = MechanismScheduler.create(context).register(mechanism);

        runLifecycle(scheduler, context, LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, 0.0);
        runLifecycle(scheduler, context, LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT, true, 0.02);
        assertEquals(0.0, RuntimeHardwareAccess.call(context, mechanism.driverHeading::yawDegrees), 1.0e-9);

        handle.yawDegrees = 35.0;
        runLifecycle(scheduler, context, LifecycleMode.DISABLED, LifecyclePhase.INIT, false, 0.04);
        allianceHeading.set(-180.0);
        runLifecycle(scheduler, context, LifecycleMode.AUTONOMOUS, LifecyclePhase.INIT, true, 0.06);
        runLifecycle(scheduler, context, LifecycleMode.AUTONOMOUS, LifecyclePhase.EXIT, true, 0.08);

        assertEquals(-180.0, RuntimeHardwareAccess.call(context, mechanism.driverHeading::yawDegrees), 1.0e-9);
    }

    private static void runLifecycle(
            MechanismScheduler scheduler,
            ActionContext context,
            LifecycleMode mode,
            LifecyclePhase phase,
            boolean enabled,
            double nowSeconds) {
        RuntimeHardwareAccess.run(context, () -> scheduler.periodic(
                new MechanismContext(nowSeconds, 0.0, 0.02, enabled, mode == LifecycleMode.AUTONOMOUS, false),
                new EventContext(nowSeconds, 0.02, mode, phase, enabled, false)));
    }

    private static final class ImuMechanism implements Mechanism {
        private final ImuDevice imu;
        private final Action setHeading;

        private ImuMechanism(ImuDevice imu) {
            this.imu = imu;
            setHeading = imu.setYaw(90.0);
        }
    }

    private static final class AutoHeadingMechanism implements Mechanism {
        private final ImuDevice imu;
        private final ca.frc6390.athena.hardware.signal.ImuSource driverHeading;
        private final HookBinding alignHeading;

        private AutoHeadingMechanism(ImuDevice imu) {
            this.imu = imu;
            driverHeading = imu.relative();
            alignHeading = Events.autonomousInit().onStart(driverHeading.setYaw(() -> -180.0));
        }
    }

    private static final class InvertedImuMechanism implements Mechanism {
        private final ImuDevice imu;
        private final ca.frc6390.athena.hardware.signal.ImuSource heading;
        private final Action setHeading;

        private InvertedImuMechanism(ImuDevice imu) {
            this.imu = imu;
            heading = imu.inverted();
            setHeading = heading.setYaw(90.0);
        }
    }

    private static final class ReusableAutoHeadingMechanism implements Mechanism {
        private final ImuDevice imu;
        private final ca.frc6390.athena.hardware.signal.ImuSource driverHeading;
        private final HookBinding alignHeading;

        private ReusableAutoHeadingMechanism(ImuDevice imu, AtomicReference<Double> allianceHeading) {
            this.imu = imu;
            driverHeading = imu.relative();
            alignHeading = Events.autonomousExit().onStart(driverHeading.setYaw(allianceHeading::get));
        }
    }

    private static final class RecordingImuHandle implements ImuHandle {
        private final ImuDevice device;
        private double yawDegrees;

        private RecordingImuHandle(ImuDevice device) {
            this.device = device;
        }

        @Override
        public ImuDevice device() {
            return device;
        }

        @Override
        public double yawDegrees() {
            return yawDegrees;
        }

        @Override
        public void setYawDegrees(double yawDegrees) {
            this.yawDegrees = yawDegrees;
        }
    }
}
