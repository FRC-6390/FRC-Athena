package ca.frc6390.athena.hardware.imu;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Decorates a concrete {@link Imu} with virtual-axis support to preserve legacy heading helpers
 * while routing all sensor reads through the new vendordep-backed IMU implementations.
 */
public class VirtualImu implements Imu {
    private final Imu delegate;
    private final Map<String, VirtualAxis> virtualAxes = new HashMap<>();
    private boolean useSimulatedReadings = false;
    private Rotation2d simRoll = new Rotation2d();
    private Rotation2d simPitch = new Rotation2d();
    private Rotation2d simYaw = new Rotation2d();
    private Rotation2d simVelX = new Rotation2d();
    private Rotation2d simVelY = new Rotation2d();
    private Rotation2d simVelZ = new Rotation2d();

    private static class VirtualAxis {
        private final Supplier<Rotation2d> supplier;
        private Rotation2d offset = new Rotation2d();

        VirtualAxis(Supplier<Rotation2d> supplier) {
            this.supplier = supplier;
        }

        Rotation2d get() {
            return supplier.get().minus(offset);
        }

        void set(Rotation2d value) {
            offset = supplier.get().minus(value);
        }

        void setOffset(Rotation2d value) {
            offset = value;
        }
    }

    public VirtualImu(Imu delegate) {
        this.delegate = delegate;
        addVirtualAxis("driver", delegate::getYaw);
    }

    @Override
    public Rotation2d getRoll() {
        return useSimulatedReadings ? simRoll : delegate.getRoll();
    }

    @Override
    public Rotation2d getPitch() {
        return useSimulatedReadings ? simPitch : delegate.getPitch();
    }

    @Override
    public Rotation2d getYaw() {
        return useSimulatedReadings ? simYaw : delegate.getYaw();
    }

    @Override
    public Rotation2d getVelocityX() {
        return useSimulatedReadings ? simVelX : delegate.getVelocityX();
    }

    @Override
    public Rotation2d getVelocityY() {
        return useSimulatedReadings ? simVelY : delegate.getVelocityY();
    }

    @Override
    public Rotation2d getVelocityZ() {
        return useSimulatedReadings ? simVelZ : delegate.getVelocityZ();
    }

    @Override
    public void setInverted(boolean inverted) {
        delegate.setInverted(inverted);
    }

    @Override
    public boolean isInverted() {
        return delegate.isInverted();
    }

    @Override
    public boolean isConnected() {
        return delegate.isConnected();
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        setVirtualAxis("driver", yaw);
    }

    @Override
    public void setYaw(double yawDegrees) {
        setYaw(Rotation2d.fromDegrees(yawDegrees));
    }

    @Override
    public void addVirtualAxis(String name, Supplier<Rotation2d> supplier) {
        virtualAxes.put(name, new VirtualAxis(supplier));
    }

    @Override
    public Rotation2d getVirtualAxis(String name) {
        VirtualAxis axis = virtualAxes.get(name);
        return axis != null ? axis.get() : new Rotation2d();
    }

    @Override
    public void setVirtualAxis(String name, Rotation2d value) {
        VirtualAxis axis = virtualAxes.get(name);
        if (axis != null) {
            axis.set(value);
        }
    }

    public void setVirtualOffset(String name, Rotation2d value) {
        VirtualAxis axis = virtualAxes.get(name);
        if (axis != null) {
            axis.setOffset(value);
        }
    }

    @Override
    public void update() {
        if (!useSimulatedReadings) {
            delegate.update();
        }
    }

    @Override
    public void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {
        setSimulatedReadings(yaw, new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d(), angularVelocityZ);
    }

    @Override
    public void setSimulatedReadings(Rotation2d yaw, Rotation2d pitch, Rotation2d roll,
                                     Rotation2d velX, Rotation2d velY, Rotation2d velZ) {
        simYaw = yaw;
        simPitch = pitch;
        simRoll = roll;
        simVelX = velX;
        simVelY = velY;
        simVelZ = velZ;
        useSimulatedReadings = true;
    }

    @Override
    public void disableSimulatedReadings() {
        useSimulatedReadings = false;
    }

    @Override
    public ImuConfig getConfig() {
        return delegate.getConfig();
    }
}
