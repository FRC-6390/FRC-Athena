package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
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

    private static final class ImuMechanism implements Mechanism {
        private final ImuDevice imu;
        private final Action setHeading;

        private ImuMechanism(ImuDevice imu) {
            this.imu = imu;
            setHeading = imu.setYaw(90.0);
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
