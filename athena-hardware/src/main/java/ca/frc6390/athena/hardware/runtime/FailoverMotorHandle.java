package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.Objects;
import java.util.OptionalDouble;

/**
 * Stands in for a motor group's declared leader. Commands and integrated-encoder reads go to whichever motor leads
 * the group, in the declared leader's frame, so controls, feedback and robot code that name the leader keep working
 * after a promotion. Refresh, health and the attached absolute encoder stay on the declared leader's own controller.
 */
final class FailoverMotorHandle implements MotorHandle, AutoCloseable {
    private final MotorHandle original;
    private volatile Route route;

    FailoverMotorHandle(MotorHandle original) {
        this.original = Objects.requireNonNull(original, "original");
        route = new Route(original, 1.0, 0.0, 1.0, original.device().isInverted(), true);
    }

    MotorHandle original() { return original; }

    MotorHandle target() { return route.handle; }

    /** Leads with another motor. Direction maps this group's command frame onto that motor's controller. */
    void lead(MotorHandle handle, double direction, boolean controllerInverted, double offsetRotations) {
        Route current = route;
        route = new Route(handle, direction, offsetRotations, current.scale, controllerInverted,
                handle == original && direction == 1.0 && offsetRotations == 0.0);
    }

    void scale(double scale) {
        Route current = route;
        if (current.scale != scale) {
            route = new Route(current.handle, current.direction, current.offset, scale,
                    current.controllerInverted, current.nativePosition);
        }
    }

    @Override public MotorDevice device() { return original.device(); }
    @Override public void activate() { original.activate(); }
    @Override public void refreshInputs() { original.refreshInputs(); }
    @Override public boolean faulted() { return original.faulted(); }
    @Override public boolean supportsRuntimeConfiguration() { return route.handle.supportsRuntimeConfiguration(); }

    @Override
    public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        Objects.requireNonNull(configuration, "configuration");
        Route current = route;
        current.handle.applyRuntimeConfiguration(current.handle == original
                ? configuration
                : new MotorRuntimeConfig(configuration.neutralMode(), current.controllerInverted,
                        configuration.supplyCurrentLimitAmps(), configuration.statorCurrentLimitAmps()));
    }

    @Override public OptionalDouble temperatureCelsius() { return route.handle.temperatureCelsius(); }
    @Override public double appliedVoltage() { Route r = route; return r.direction * r.handle.appliedVoltage(); }
    @Override public double supplyCurrentAmps() { return route.handle.supplyCurrentAmps(); }
    @Override public double statorCurrentAmps() { return route.handle.statorCurrentAmps(); }

    @Override
    public void follow(MotorHandle leader, boolean inverted) {
        throw new UnsupportedOperationException("A motor group leader cannot follow another motor.");
    }

    @Override public void setPercentOutput(double percent) {
        Route r = route;
        r.handle.setPercentOutput(r.direction * r.scale * percent);
    }

    @Override public void setVoltage(double volts) {
        Route r = route;
        r.handle.setVoltage(r.direction * r.scale * volts);
    }

    @Override public void setPositionTargetRotations(double rotations) {
        Route r = route;
        r.handle.setPositionTargetRotations(r.direction * (rotations - r.offset));
    }

    @Override public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
        Route r = route;
        r.handle.setPositionTargetRotations(r.direction * (rotations - r.offset), directed(request, r.direction));
    }

    @Override public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        Route r = route;
        r.handle.setVelocityTargetRotationsPerSecond(r.direction * rotationsPerSecond);
    }

    @Override public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond, MotorClosedLoopRequest request) {
        Route r = route;
        r.handle.setVelocityTargetRotationsPerSecond(r.direction * rotationsPerSecond, directed(request, r.direction));
    }

    @Override public MotorControlCapabilities controlCapabilities() { return route.handle.controlCapabilities(); }
    @Override public void stop() { route.handle.stop(); }

    @Override public double integratedPositionRotations() {
        Route r = route;
        return r.direction * r.handle.integratedPositionRotations() + r.offset;
    }

    @Override public double integratedVelocityRotationsPerSecond() {
        Route r = route;
        return r.direction * r.handle.integratedVelocityRotationsPerSecond();
    }

    @Override
    public void setIntegratedPositionRotations(double rotations) {
        Route r = route;
        if (r.nativePosition) {
            original.setIntegratedPositionRotations(rotations);
            return;
        }
        if (!Double.isFinite(rotations)) throw new IllegalArgumentException("Encoder position must be finite.");
        route = new Route(r.handle, r.direction,
                rotations - r.direction * r.handle.integratedPositionRotations(),
                r.scale, r.controllerInverted, false);
    }

    @Override
    public boolean supportsIntegratedPositionSetting() {
        Route r = route;
        return !r.nativePosition || original.supportsIntegratedPositionSetting();
    }

    @Override public void enableIntegratedEncoder() { original.enableIntegratedEncoder(); }
    @Override public void enableAbsoluteEncoder() { original.enableAbsoluteEncoder(); }
    @Override public double absolutePositionRotations() { return original.absolutePositionRotations(); }
    @Override public double absoluteVelocityRotationsPerSecond() { return original.absoluteVelocityRotationsPerSecond(); }

    @Override
    public void close() {
        if (original instanceof AutoCloseable closeable) {
            try {
                closeable.close();
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("Interrupted while closing " + device().defaultName(), exception);
            } catch (Exception exception) {
                throw new IllegalStateException("Failed to close " + device().defaultName(), exception);
            }
        }
    }

    private static MotorClosedLoopRequest directed(MotorClosedLoopRequest request, double direction) {
        if (request == null || direction > 0.0 || request.arbitraryFeedforwardVolts() == 0.0) return request;
        return new MotorClosedLoopRequest(request.route(), request.config(), -request.arbitraryFeedforwardVolts());
    }

    /**
     * Immutable routing swapped as one write.
     *
     * @param handle motor that leads the group
     * @param direction sign mapping the group command frame onto that motor's controller
     * @param offset position offset keeping integrated position continuous across a promotion
     * @param scale percent and voltage derate
     * @param controllerInverted inversion to keep on that controller when runtime configuration is reapplied
     * @param nativePosition true while the declared leader leads unchanged, so position setting stays native
     */
    private record Route(
            MotorHandle handle,
            double direction,
            double offset,
            double scale,
            boolean controllerInverted,
            boolean nativePosition) {
    }
}
