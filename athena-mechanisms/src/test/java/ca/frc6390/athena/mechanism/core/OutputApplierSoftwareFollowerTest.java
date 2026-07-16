package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.List;
import org.junit.jupiter.api.Test;

class OutputApplierSoftwareFollowerTest {
    @Test
    void oneControlBindingCommandsEverySuppliedMotor() {
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice second = MotorDevice.of(MotorKinds.KRAKEN_X60, 2);
        MotorDevice third = MotorDevice.of(MotorKinds.KRAKEN_X60, 3);
        RecordingMotor firstHandle = new RecordingMotor(first);
        RecordingMotor secondHandle = new RecordingMotor(second);
        RecordingMotor thirdHandle = new RecordingMotor(third);
        ControlBinding control = Controls.velocity(first, second, third);
        ActionContext context = new ActionContext() {
            @Override
            public MotorHandle motor(MotorDevice device) {
                if (device.equals(first)) return firstHandle;
                if (device.equals(second)) return secondHandle;
                return thirdHandle;
            }
        };
        Output velocity = Outputs.velocity(42.0);

        OutputApplier.using(context).apply(new ResolvedOutput(OutputRequest.of(control, velocity), velocity));

        assertEquals(42.0, firstHandle.velocity, 1.0e-9);
        assertEquals(42.0, secondHandle.velocity, 1.0e-9);
        assertEquals(42.0, thirdHandle.velocity, 1.0e-9);
    }

    @Test
    void resolvedLeaderOutputReachesSoftwareFollowerInTheSameApply() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).canbus("rio");
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2).canbus("canivore").follow(leader);
        RecordingMotor leaderHandle = new RecordingMotor(leader);
        RecordingMotor followerHandle = new RecordingMotor(follower);
        ActionContext context = new ActionContext() {
            @Override
            public MotorHandle motor(MotorDevice device) {
                return device.equals(leader) ? leaderHandle : followerHandle;
            }

            @Override
            public List<SoftwareMotorFollower> softwareFollowers(MotorDevice device) {
                return device.equals(leader)
                        ? List.of(new SoftwareMotorFollower(follower, followerHandle))
                        : List.of();
            }
        };
        Output voltage = Outputs.voltage(6.0);

        OutputApplier.using(context).apply(new ResolvedOutput(OutputRequest.of(leader, voltage), voltage));

        assertEquals(6.0, leaderHandle.voltage, 1.0e-9);
        assertEquals(6.0, followerHandle.voltage, 1.0e-9);
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double voltage;
        private double velocity;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public void setVoltage(double volts) { voltage = volts; }
        @Override public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocity = rotationsPerSecond;
        }
    }
}
