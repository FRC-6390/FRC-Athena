package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;

/** Cached root-to-parent transform for a software follower below another follower. */
final class ChainedFollowerMotorHandle implements MotorHandle {
    private final MotorHandle local;
    private final MotorHandle parentFrame;

    ChainedFollowerMotorHandle(MotorHandle local, MotorHandle parentFrame) {
        this.local = local;
        this.parentFrame = parentFrame;
    }

    private double scale() { return parentFrame.voltageCommandScale(); }
    private double direction() { return scale() < 0.0 ? -1.0 : 1.0; }
    private MotorClosedLoopRequest directed(MotorClosedLoopRequest request) {
        return request == null || direction() > 0.0 || request.arbitraryFeedforwardVolts() == 0.0 ? request
                : new MotorClosedLoopRequest(request.route(), request.config(), -request.arbitraryFeedforwardVolts());
    }
    @Override public MotorDevice device() { return local.device(); }
    @Override public MotorHandle commandTarget() { return local.commandTarget(); }
    @Override public boolean supportsRuntimeConfiguration() { return local.supportsRuntimeConfiguration(); }
    @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) { local.applyRuntimeConfiguration(configuration); }
    @Override public void setPercentOutput(double percent) { local.setPercentOutput(scale() * percent); }
    @Override public void setVoltage(double volts) { local.setVoltage(scale() * volts); }
    @Override public void setPositionTargetRotations(double position) { local.setPositionTargetRotations(direction() * position); }
    @Override public void setPositionTargetRotations(double position, MotorClosedLoopRequest request) {
        local.setPositionTargetRotations(direction() * position, directed(request));
    }
    @Override public void setVelocityTargetRotationsPerSecond(double velocity) {
        local.setVelocityTargetRotationsPerSecond(direction() * velocity);
    }
    @Override public void setVelocityTargetRotationsPerSecond(double velocity, MotorClosedLoopRequest request) {
        local.setVelocityTargetRotationsPerSecond(direction() * velocity, directed(request));
    }
    @Override public MotorControlCapabilities controlCapabilities() { return local.controlCapabilities(); }
    @Override public boolean enforcesSupplyCurrentLimit() { return local.enforcesSupplyCurrentLimit(); }
    @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
        return local.configurationStatus();
    }
    @Override public void stop() { local.stop(); }
    @Override public double integratedVelocityRotationsPerSecond() { return local.integratedVelocityRotationsPerSecond(); }
    @Override public double appliedVoltage() { return local.appliedVoltage(); }
    @Override public double supplyCurrentAmps() { return local.supplyCurrentAmps(); }
    @Override public double statorCurrentAmps() { return local.statorCurrentAmps(); }
    @Override public double voltageCommandScale() { return scale() * local.voltageCommandScale(); }
}
