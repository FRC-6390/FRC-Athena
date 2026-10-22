package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Objects;

/** Independent motor handle used when Athena must mirror a follower in software. */
final class SoftwareFollowerMotorHandle implements MotorHandle, AutoCloseable {
    private final MotorDevice device;
    private final MotorHandle delegate;
    private double direction;
    /** Percent and voltage derate set by motor failover while its group runs degraded. */
    private double scale = 1.0;

    SoftwareFollowerMotorHandle(MotorDevice device, MotorHandle delegate) {
        this(device, delegate, device.isInverted() ? -1.0 : 1.0);
    }

    /** Creates a follower driven with an explicit direction relative to its group's command. */
    SoftwareFollowerMotorHandle(MotorDevice device, MotorHandle delegate, double direction) {
        this.device = Objects.requireNonNull(device, "device");
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.direction = direction;
    }

    MotorHandle backendHandle() { return delegate; }

    @Override public MotorHandle commandTarget() { return delegate.commandTarget(); }

    double direction() { return direction; }

    void scale(double scale) { this.scale = scale; }

    @Override public MotorDevice device() { return device; }
    @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
        return delegate.configurationStatus();
    }

    @Override public boolean supportsRuntimeConfiguration() { return delegate.supportsRuntimeConfiguration(); }
    @Override public boolean enforcesSupplyCurrentLimit() { return delegate.enforcesSupplyCurrentLimit(); }

    @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        Objects.requireNonNull(configuration, "configuration");
        direction = configuration.inverted() ? -1.0 : 1.0;
        delegate.applyRuntimeConfiguration(new MotorRuntimeConfig(
                configuration.neutralMode(), false,
                configuration.supplyCurrentLimitAmps(), configuration.statorCurrentLimitAmps()));
    }
    @Override public void activate() { delegate.activate(); }
    @Override public void refreshInputs() { delegate.refreshInputs(); }
    @Override public boolean faulted() { return delegate.faulted(); }
    @Override public void stopFollowing() { delegate.stopFollowing(); }
    @Override public double appliedVoltage() { return delegate.appliedVoltage(); }
    @Override public double supplyCurrentAmps() { return delegate.supplyCurrentAmps(); }
    @Override public double statorCurrentAmps() { return delegate.statorCurrentAmps(); }
    @Override public double voltageCommandScale() { return direction * scale * delegate.voltageCommandScale(); }
    @Override public void setPercentOutput(double percent) { delegate.setPercentOutput(direction * scale * percent); }
    @Override public void setVoltage(double volts) { delegate.setVoltage(direction * scale * volts); }
    @Override public void setPositionTargetRotations(double rotations) {
        delegate.setPositionTargetRotations(direction * rotations);
    }
    @Override public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
        delegate.setPositionTargetRotations(direction * rotations, directed(request));
    }
    @Override public void setVelocityTargetRotationsPerSecond(double velocity) {
        delegate.setVelocityTargetRotationsPerSecond(direction * velocity);
    }
    @Override public void setVelocityTargetRotationsPerSecond(double velocity, MotorClosedLoopRequest request) {
        delegate.setVelocityTargetRotationsPerSecond(direction * velocity, directed(request));
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

    private MotorClosedLoopRequest directed(MotorClosedLoopRequest request) {
        if (request == null || direction > 0.0 || request.arbitraryFeedforwardVolts() == 0.0) return request;
        // Like failover, the output derate applies only to direct voltage/percent commands. Offloaded loops
        // keep their gains/targets; their additive voltage must still follow the target's direction.
        return new MotorClosedLoopRequest(request.route(), request.config(), -request.arbitraryFeedforwardVolts());
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
