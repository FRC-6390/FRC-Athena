package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Objects;

/** Independent motor handle used when Athena must mirror a follower in software. */
final class SoftwareFollowerMotorHandle implements MotorHandle, AutoCloseable {
    private final MotorDevice device;
    private final MotorHandle delegate;
    private final double direction;

    SoftwareFollowerMotorHandle(MotorDevice device, MotorHandle delegate) {
        this.device = Objects.requireNonNull(device, "device");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        direction = device.isInverted() ? -1.0 : 1.0;
    }

    MotorHandle backendHandle() { return delegate; }

    @Override public MotorDevice device() { return device; }
    @Override public void activate() { delegate.activate(); }
    @Override public void refreshInputs() { delegate.refreshInputs(); }
    @Override public double appliedVoltage() { return delegate.appliedVoltage(); }
    @Override public double supplyCurrentAmps() { return delegate.supplyCurrentAmps(); }
    @Override public double statorCurrentAmps() { return delegate.statorCurrentAmps(); }
    @Override public void setPercentOutput(double percent) { delegate.setPercentOutput(direction * percent); }
    @Override public void setVoltage(double volts) { delegate.setVoltage(direction * volts); }
    @Override public void setPositionTargetRotations(double rotations) {
        delegate.setPositionTargetRotations(direction * rotations);
    }
    @Override public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
        delegate.setPositionTargetRotations(direction * rotations, request);
    }
    @Override public void setVelocityTargetRotationsPerSecond(double velocity) {
        delegate.setVelocityTargetRotationsPerSecond(direction * velocity);
    }
    @Override public void setVelocityTargetRotationsPerSecond(double velocity, MotorClosedLoopRequest request) {
        delegate.setVelocityTargetRotationsPerSecond(direction * velocity, request);
    }
    @Override public MotorControlCapabilities controlCapabilities() { return delegate.controlCapabilities(); }
    @Override public void stop() { delegate.stop(); }
    @Override public double integratedPositionRotations() { return delegate.integratedPositionRotations(); }
    @Override public double integratedVelocityRotationsPerSecond() {
        return delegate.integratedVelocityRotationsPerSecond();
    }
    @Override public void setIntegratedPositionRotations(double rotations) {
        delegate.setIntegratedPositionRotations(rotations);
    }
    @Override public boolean supportsIntegratedPositionSetting() {
        return delegate.supportsIntegratedPositionSetting();
    }
    @Override public void enableIntegratedEncoder() { delegate.enableIntegratedEncoder(); }
    @Override public void enableAbsoluteEncoder() { delegate.enableAbsoluteEncoder(); }
    @Override public double absolutePositionRotations() { return delegate.absolutePositionRotations(); }
    @Override public double absoluteVelocityRotationsPerSecond() {
        return delegate.absoluteVelocityRotationsPerSecond();
    }

    @Override
    public void close() {
        if (delegate instanceof AutoCloseable closeable) {
            try {
                closeable.close();
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("Interrupted while closing " + device.defaultName(), exception);
            } catch (Exception exception) {
                throw new IllegalStateException("Failed to close " + device.defaultName(), exception);
            }
        }
    }
}
