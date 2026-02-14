package ca.frc6390.athena.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotNetworkTables;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Vendor-agnostic IMU interface for the new vendordep system.
 */
public interface Imu extends RobotSendableSystem.RobotSendableDevice {
    Rotation2d getRoll();

    Rotation2d getPitch();

    Rotation2d getYaw();

    default Rotation2d getRoll(boolean poll) {
        if (poll) {
            update();
        }
        return getRoll();
    }

    default Rotation2d getPitch(boolean poll) {
        if (poll) {
            update();
        }
        return getPitch();
    }

    default Rotation2d getYaw(boolean poll) {
        if (poll) {
            update();
        }
        return getYaw();
    }

    /**
     */
    void setInverted(boolean inverted);

    default boolean isInverted() { return false; }

    default boolean isConnected() { return true; }
    default boolean isConnected(boolean poll) {
        if (poll) {
            update();
        }
        return isConnected();
    }

    // Optional velocity accessors (default to zero)
    default Rotation2d getVelocityX() { return new Rotation2d(); }
    default Rotation2d getVelocityY() { return new Rotation2d(); }
    default Rotation2d getVelocityZ() { return new Rotation2d(); }
    default Rotation2d getVelocityX(boolean poll) {
        if (poll) {
            update();
        }
        return getVelocityX();
    }
    default Rotation2d getVelocityY(boolean poll) {
        if (poll) {
            update();
        }
        return getVelocityY();
    }
    default Rotation2d getVelocityZ(boolean poll) {
        if (poll) {
            update();
        }
        return getVelocityZ();
    }

    // Optional canonical speed helpers derived from velocity axes (default in rad/s).
    default double getXSpeedRadiansPerSecond() {
        Rotation2d velX = getVelocityX();
        return velX != null ? velX.getRadians() : 0.0;
    }

    default double getYSpeedRadiansPerSecond() {
        Rotation2d velY = getVelocityY();
        return velY != null ? velY.getRadians() : 0.0;
    }

    default double getThetaSpeedRadiansPerSecond() {
        Rotation2d velZ = getVelocityZ();
        return velZ != null ? velZ.getRadians() : 0.0;
    }

    default double getMovementSpeedRadiansPerSecond() {
        return Math.hypot(getXSpeedRadiansPerSecond(), getYSpeedRadiansPerSecond());
    }

    // Optional linear speed aliases in m/s for call-sites that work in linear units.
    // Implementations should override these when native/estimated linear velocity is available.
    default double getXSpeedMetersPerSecond() {
        return 0.0;
    }

    default double getYSpeedMetersPerSecond() {
        return 0.0;
    }

    default double getMovementSpeedMetersPerSecond() {
        return Math.hypot(getXSpeedMetersPerSecond(), getYSpeedMetersPerSecond());
    }

    default double getNormalizedMovementSpeed() {
        double maxLinear = getMaxLinearSpeed();
        double movement = getMovementSpeedMetersPerSecond();
        if (!Double.isFinite(movement) || movement <= 0.0) {
            return 0.0;
        }
        if (!Double.isFinite(maxLinear) || maxLinear <= 1e-9) {
            return 1.0;
        }
        double normalized = movement / maxLinear;
        if (!Double.isFinite(normalized)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, normalized));
    }

    // Convenience alias for callers that just want "normalized speed".
    default double getNormalizedSpeed() {
        return getNormalizedMovementSpeed();
    }

    // Optional angular acceleration accessors (default to zero)
    default double getAngularAccelerationZRadiansPerSecondSquared() { return 0.0; }
    default double getAngularAccelerationZDegreesPerSecondSquared() {
        return Math.toDegrees(getAngularAccelerationZRadiansPerSecondSquared());
    }

    // Optional linear acceleration accessors (default to zero)
    default double getAccelerationX() { return 0.0; }
    default double getAccelerationY() { return 0.0; }
    default double getAccelerationZ() { return 0.0; }
    default double getAccelerationX(boolean poll) {
        if (poll) {
            update();
        }
        return getAccelerationX();
    }
    default double getAccelerationY(boolean poll) {
        if (poll) {
            update();
        }
        return getAccelerationY();
    }
    default double getAccelerationZ(boolean poll) {
        if (poll) {
            update();
        }
        return getAccelerationZ();
    }

    // Optional max speed tracking helpers (default to no-op)
    default double getMaxLinearSpeed() { return 0.0; }
    default double getMaxRadialSpeed() { return 0.0; }
    default double getMaxSpeedWindowSeconds() { return 5.0; }
    /**
     */
    default void setMaxSpeedWindowSeconds(double windowSeconds) {}
    /**
     */
    default void resetMaxSpeedWindow() {}

    // Optional heading reset hook (default no-op)
    /**
     */
    default void setYaw(Rotation2d yaw) {}
    /**
     */
    default void setYaw(double yawDegrees) { setYaw(Rotation2d.fromDegrees(yawDegrees)); }

    // Virtual axis helpers (default no-op implementations for compatibility)
    /**
     */
    default void addVirtualAxis(String name, java.util.function.Supplier<Rotation2d> supplier) {}
    default Rotation2d getVirtualAxis(String name) { return new Rotation2d(); }
    /**
     */
    default void setVirtualAxis(String name, Rotation2d value) {}

    // Optional periodic update hook (default no-op)
    default void update() {}

    /**
     * Sectioned config API for interacting with an already-built IMU.
     */
    default Imu config(Consumer<RuntimeSection> section) {
        if (section != null) {
            section.accept(new RuntimeSection(this));
        }
        return this;
    }

    /**
     * Convenience accessor for non-lambda config chaining.
     */
    default RuntimeSection config() {
        return new RuntimeSection(this);
    }

    /**
     * Sectioned runtime API for virtual-axis interactions.
     */
    default Imu virtualAxes(Consumer<VirtualAxesSection> section) {
        if (section != null) {
            section.accept(new VirtualAxesSection(this));
        }
        return this;
    }

    /**
     * Sectioned runtime API for simulation interactions.
     */
    default Imu sim(Consumer<SimulationSection> section) {
        if (section != null) {
            section.accept(new SimulationSection(this));
        }
        return this;
    }

    // Simulation hooks (default no-op)
    /**
     */
    default void setSimulatedHeading(Rotation2d yaw, Rotation2d angularVelocityZ) {}
    /**
     */
    default void setSimulatedReadings(Rotation2d yaw, Rotation2d pitch, Rotation2d roll,
                                      Rotation2d velX, Rotation2d velY, Rotation2d velZ) {}
    /**
     */
    default void disableSimulatedReadings() {}

    ImuConfig getConfig();

    /**
     * Structured read API: {@code imu.angles().yawDegrees()} etc.
     */
    default AnglesView angles() {
        return new AnglesView(this, false);
    }

    default VelocityView velocity() {
        return new VelocityView(this, false);
    }

    default AccelerationView acceleration() {
        return new AccelerationView(this, false);
    }

    default StatusView status() {
        return new StatusView(this, false);
    }

    final class RuntimeSection {
        private final Imu owner;

        RuntimeSection(Imu owner) {
            this.owner = owner;
        }

        public RuntimeSection poll() {
            owner.update();
            return this;
        }

        public RuntimeSection inverted(boolean inverted) {
            owner.setInverted(inverted);
            return this;
        }

        public RuntimeSection yaw(Rotation2d yaw) {
            owner.setYaw(yaw);
            return this;
        }

        public RuntimeSection yawDegrees(double yawDegrees) {
            owner.setYaw(yawDegrees);
            return this;
        }

        public RuntimeSection maxSpeedWindowSeconds(double windowSeconds) {
            owner.setMaxSpeedWindowSeconds(windowSeconds);
            return this;
        }

        public RuntimeSection resetMaxSpeedWindow() {
            owner.resetMaxSpeedWindow();
            return this;
        }
    }

    final class VirtualAxesSection {
        private final Imu owner;

        VirtualAxesSection(Imu owner) {
            this.owner = owner;
        }

        public VirtualAxesSection add(String name, Supplier<Rotation2d> supplier) {
            owner.addVirtualAxis(name, supplier);
            return this;
        }

        public VirtualAxesSection set(String name, Rotation2d value) {
            owner.setVirtualAxis(name, value);
            return this;
        }
    }

    final class SimulationSection {
        private final Imu owner;

        SimulationSection(Imu owner) {
            this.owner = owner;
        }

        public SimulationSection heading(Rotation2d yaw, Rotation2d angularVelocityZ) {
            owner.setSimulatedHeading(yaw, angularVelocityZ);
            return this;
        }

        public SimulationSection readings(
                Rotation2d yaw,
                Rotation2d pitch,
                Rotation2d roll,
                Rotation2d velX,
                Rotation2d velY,
                Rotation2d velZ) {
            owner.setSimulatedReadings(yaw, pitch, roll, velX, velY, velZ);
            return this;
        }

        public SimulationSection disable() {
            owner.disableSimulatedReadings();
            return this;
        }
    }

    final class AnglesView {
        private final Imu owner;
        private final boolean poll;

        AnglesView(Imu owner, boolean poll) {
            this.owner = owner;
            this.poll = poll;
        }

        public AnglesView poll() {
            return new AnglesView(owner, true);
        }

        public Rotation2d yaw() {
            return owner.getYaw(poll);
        }

        public Rotation2d pitch() {
            return owner.getPitch(poll);
        }

        public Rotation2d roll() {
            return owner.getRoll(poll);
        }

        public double yawDegrees() {
            Rotation2d yaw = yaw();
            return yaw != null ? yaw.getDegrees() : 0.0;
        }

        public double pitchDegrees() {
            Rotation2d pitch = pitch();
            return pitch != null ? pitch.getDegrees() : 0.0;
        }

        public double rollDegrees() {
            Rotation2d roll = roll();
            return roll != null ? roll.getDegrees() : 0.0;
        }
    }

    final class VelocityView {
        private final Imu owner;
        private final boolean poll;

        VelocityView(Imu owner, boolean poll) {
            this.owner = owner;
            this.poll = poll;
        }

        public VelocityView poll() {
            return new VelocityView(owner, true);
        }

        public Rotation2d x() {
            return owner.getVelocityX(poll);
        }

        public Rotation2d y() {
            return owner.getVelocityY(poll);
        }

        public Rotation2d z() {
            return owner.getVelocityZ(poll);
        }

        public double xRadiansPerSecond() {
            Rotation2d x = x();
            return x != null ? x.getRadians() : 0.0;
        }

        public double yRadiansPerSecond() {
            Rotation2d y = y();
            return y != null ? y.getRadians() : 0.0;
        }

        public double zRadiansPerSecond() {
            Rotation2d z = z();
            return z != null ? z.getRadians() : 0.0;
        }

        public double xMetersPerSecond() {
            return owner.getXSpeedMetersPerSecond();
        }

        public double yMetersPerSecond() {
            return owner.getYSpeedMetersPerSecond();
        }

        public double movementMetersPerSecond() {
            return owner.getMovementSpeedMetersPerSecond();
        }

        public double normalizedMovement() {
            return owner.getNormalizedMovementSpeed();
        }
    }

    final class AccelerationView {
        private final Imu owner;
        private final boolean poll;

        AccelerationView(Imu owner, boolean poll) {
            this.owner = owner;
            this.poll = poll;
        }

        public AccelerationView poll() {
            return new AccelerationView(owner, true);
        }

        public double x() {
            return owner.getAccelerationX(poll);
        }

        public double y() {
            return owner.getAccelerationY(poll);
        }

        public double z() {
            return owner.getAccelerationZ(poll);
        }

        public double angularZRadiansPerSecondSquared() {
            return owner.getAngularAccelerationZRadiansPerSecondSquared();
        }

        public double angularZDegreesPerSecondSquared() {
            return owner.getAngularAccelerationZDegreesPerSecondSquared();
        }
    }

    final class StatusView {
        private final Imu owner;
        private final boolean poll;

        StatusView(Imu owner, boolean poll) {
            this.owner = owner;
            this.poll = poll;
        }

        public StatusView poll() {
            return new StatusView(owner, true);
        }

        public boolean connected() {
            return owner.isConnected(poll);
        }

        public boolean inverted() {
            return owner.isInverted();
        }

        public double maxLinearSpeed() {
            return owner.getMaxLinearSpeed();
        }

        public double maxRadialSpeed() {
            return owner.getMaxRadialSpeed();
        }

        public double maxSpeedWindowSeconds() {
            return owner.getMaxSpeedWindowSeconds();
        }

        public int canId() {
            ImuConfig cfg = owner.getConfig();
            return cfg != null ? cfg.id() : -1;
        }

        public String canbus() {
            ImuConfig cfg = owner.getConfig();
            return cfg != null ? cfg.canbus() : "";
        }

        public String typeKey() {
            ImuConfig cfg = owner.getConfig();
            return cfg != null && cfg.type() != null ? cfg.type().getKey() : "unknown";
        }
    }

    @Override
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        Rotation2d yaw = getYaw();
        Rotation2d pitch = getPitch();
        Rotation2d roll = getRoll();
        Rotation2d velX = getVelocityX();
        Rotation2d velY = getVelocityY();
        Rotation2d velZ = getVelocityZ();

        node.putDouble("yawDeg", yaw != null ? yaw.getDegrees() : 0.0);
        node.putDouble("pitchDeg", pitch != null ? pitch.getDegrees() : 0.0);
        node.putDouble("rollDeg", roll != null ? roll.getDegrees() : 0.0);
        node.putDouble("velXDegPerSec", velX != null ? velX.getDegrees() : 0.0);
        node.putDouble("velYDegPerSec", velY != null ? velY.getDegrees() : 0.0);
        node.putDouble("velZDegPerSec", velZ != null ? velZ.getDegrees() : 0.0);
        node.putDouble("xSpeedRadPerSec", getXSpeedRadiansPerSecond());
        node.putDouble("ySpeedRadPerSec", getYSpeedRadiansPerSecond());
        node.putDouble("thetaSpeedRadPerSec", getThetaSpeedRadiansPerSecond());
        node.putDouble("movementSpeedRadPerSec", getMovementSpeedRadiansPerSecond());
        node.putDouble("xSpeedMps", getXSpeedMetersPerSecond());
        node.putDouble("ySpeedMps", getYSpeedMetersPerSecond());
        node.putDouble("movementSpeedMps", getMovementSpeedMetersPerSecond());
        node.putDouble("movementSpeedNormalized", getNormalizedMovementSpeed());
        node.putDouble("normalizedSpeed", getNormalizedSpeed());
        node.putDouble("accelX", getAccelerationX());
        node.putDouble("accelY", getAccelerationY());
        node.putDouble("accelZ", getAccelerationZ());
        node.putBoolean("connected", isConnected());

        if (nt.enabled(RobotNetworkTables.Flag.HW_IMU_TUNING_WIDGETS)) {
            node.putBoolean("inverted", isInverted());
            node.putDouble("maxLinearSpeed", getMaxLinearSpeed());
            node.putDouble("maxRadialSpeed", getMaxRadialSpeed());
            node.putDouble("maxSpeedWindowSec", getMaxSpeedWindowSeconds());
            node.putDouble("angularAccelZDegPerSec2", getAngularAccelerationZDegreesPerSecondSquared());

            ImuConfig cfg = getConfig();
            if (cfg != null) {
                node.putDouble("canId", cfg.id());
                node.putString("canbus", cfg.canbus() != null ? cfg.canbus() : "");
                node.putString("type", cfg.type() != null ? cfg.type().getKey() : "unknown");
            }
        }

        return node;
    }
}
