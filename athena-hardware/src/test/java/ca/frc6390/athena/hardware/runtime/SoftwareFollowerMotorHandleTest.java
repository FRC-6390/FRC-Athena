package ca.frc6390.athena.hardware.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import org.junit.jupiter.api.Test;

class SoftwareFollowerMotorHandleTest {
    @Test
    void reportsCommandDirectionAndDerateWithoutChangingPhysicalTelemetry() {
        MotorDevice device = MotorDevice.of(MotorKinds.FALCON_500, 33).inverted();
        PhysicalMotor backend = new PhysicalMotor(device.independent().inverted(false));
        SoftwareFollowerMotorHandle follower = new SoftwareFollowerMotorHandle(device, backend);
        backend.speed = -50.0;
        follower.scale(0.5);

        follower.setVoltage(12.0);

        assertEquals(-0.5, follower.voltageCommandScale());
        assertEquals(-6.0, follower.appliedVoltage());
        assertEquals(-50.0, follower.integratedVelocityRotationsPerSecond());

        follower.applyRuntimeConfiguration(new MotorRuntimeConfig(MotorNeutralMode.BRAKE, false, 40, 40));
        follower.setVoltage(12.0);
        assertEquals(0.5, follower.voltageCommandScale(), "runtime inversion changes the command mapping");
        assertEquals(6.0, follower.appliedVoltage());
        assertEquals(-50.0, follower.integratedVelocityRotationsPerSecond(), "sensors retain shaft signs");
    }

    @Test
    void combinesWrapperDirectionWithAnInvertedBackendAndFailoverPromotion() {
        MotorDevice device = MotorDevice.of(MotorKinds.FALCON_500, 33).inverted();
        PhysicalMotor backend = new PhysicalMotor(device);
        backend.scale = -1.0;
        backend.speed = 50.0;
        SoftwareFollowerMotorHandle follower = new SoftwareFollowerMotorHandle(device, backend, -1.0);
        follower.scale(0.5);
        follower.setVoltage(12.0);
        assertEquals(0.5, follower.voltageCommandScale());
        assertEquals(6.0, follower.appliedVoltage());

        FailoverMotorHandle leader = new FailoverMotorHandle(new PhysicalMotor(device.independent().inverted(false)));
        leader.lead(backend, -1.0, true, 0.0);
        leader.scale(0.5);
        leader.setVoltage(12.0);
        assertEquals(-0.5, leader.voltageCommandScale(), "route direction also transforms the velocity reads");
        assertEquals(-50.0, leader.integratedVelocityRotationsPerSecond());
        assertEquals(-6.0, leader.appliedVoltage());
    }

    @Test
    void hybridFeedforwardFollowsPositionAndVelocityTargetDirection() {
        MotorDevice device = MotorDevice.of(MotorKinds.FALCON_500, 33).inverted();
        PhysicalMotor backend = new PhysicalMotor(device.independent().inverted(false));
        SoftwareFollowerMotorHandle follower = new SoftwareFollowerMotorHandle(device, backend);
        follower.scale(0.5);
        MotorClosedLoopConfig config = MotorClosedLoopConfig.empty();
        MotorClosedLoopRequest request = MotorClosedLoopRequest.hybrid(config, 2.0);

        follower.setPositionTargetRotations(3.0, request);
        assertEquals(-3.0, backend.target);
        assertEquals(-2.0, backend.request.arbitraryFeedforwardVolts());
        assertSame(config, backend.request.config());
        follower.setVelocityTargetRotationsPerSecond(-50.0, MotorClosedLoopRequest.hybrid(config, -2.0));
        assertEquals(50.0, backend.target);
        assertEquals(2.0, backend.request.arbitraryFeedforwardVolts());
        follower.applyRuntimeConfiguration(new MotorRuntimeConfig(MotorNeutralMode.BRAKE, false, 40, 40));
        follower.setVelocityTargetRotationsPerSecond(50.0, request);
        assertEquals(50.0, backend.target);
        assertSame(request, backend.request, "aligned requests are passed through without allocation");
    }

    private static final class PhysicalMotor implements MotorHandle {
        private final MotorDevice device;
        double speed;
        double voltage;
        double scale = 1.0;
        double target;
        MotorClosedLoopRequest request;

        PhysicalMotor(MotorDevice device) { this.device = device; }
        @Override public MotorDevice device() { return device; }
        @Override public void setVoltage(double volts) { voltage = scale * volts; }
        @Override public double appliedVoltage() { return voltage; }
        @Override public double integratedVelocityRotationsPerSecond() { return speed; }
        @Override public double voltageCommandScale() { return scale; }
        @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
            scale = configuration.inverted() ? -1.0 : 1.0;
        }
        @Override public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest value) {
            target = rotations;
            request = value;
        }
        @Override public void setVelocityTargetRotationsPerSecond(double velocity, MotorClosedLoopRequest value) {
            target = velocity;
            request = value;
        }
    }
}
