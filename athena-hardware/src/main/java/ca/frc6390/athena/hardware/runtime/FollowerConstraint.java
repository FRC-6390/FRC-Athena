package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.function.DoubleSupplier;

/** A read-only native/active-leader constraint; it must never receive mirrored output commands. */
record FollowerConstraint(MotorDevice device, MotorHandle handle) implements ActionContext.MotorFollower {
    static FollowerConstraint of(MotorDevice device, MotorHandle backend, DoubleSupplier commandScale) {
        return new FollowerConstraint(device, new MotorHandle() {
            @Override public MotorDevice device() { return device; }
            @Override public MotorHandle commandTarget() { return backend.commandTarget(); }
            @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
                return backend.configurationStatus();
            }
            @Override public double integratedVelocityRotationsPerSecond() {
                return backend.integratedVelocityRotationsPerSecond();
            }
            @Override public double appliedVoltage() { return backend.appliedVoltage(); }
            @Override public double supplyCurrentAmps() { return backend.supplyCurrentAmps(); }
            @Override public double statorCurrentAmps() { return backend.statorCurrentAmps(); }
            @Override public boolean enforcesSupplyCurrentLimit() { return backend.enforcesSupplyCurrentLimit(); }
            @Override public double voltageCommandScale() { return commandScale.getAsDouble(); }
        });
    }
}
