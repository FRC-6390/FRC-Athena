package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/**
 * Factories for derived and custom IMU sources.
 */
public final class Imus {
    private Imus() {
    }

    public static ImuSource relative(ImuSource source) {
        return new RelativeSource(Objects.requireNonNull(source, "source"));
    }

    /** Returns an IMU view using the opposite sign convention for every reading. */
    public static ImuSource inverted(ImuSource source) {
        Objects.requireNonNull(source, "source");
        return source instanceof InvertedSource inverted ? inverted.source : new InvertedSource(source);
    }

    /** Returns a source with a signed vendor-axis convention applied. */
    public static ImuSource transform(ImuSource source, ImuTransform transform) {
        return new TransformedSource(
                Objects.requireNonNull(source, "source"),
                Objects.requireNonNull(transform, "transform"));
    }

    /** Returns a source corrected for its physical mounting orientation. */
    public static ImuSource mounting(ImuSource source, ImuMount mount) {
        return new MountedSource(
                Objects.requireNonNull(source, "source"),
                Objects.requireNonNull(mount, "mount"));
    }

    public static ImuSource yaw(DoubleSupplier yawDegrees) {
        return new YawSource(Objects.requireNonNull(yawDegrees, "yawDegrees"));
    }

    @SuppressWarnings("unchecked")
    static <T extends DeviceAction> T setYawAction(ImuSource source, double yawDegrees) {
        if (!Double.isFinite(yawDegrees)) {
            throw new IllegalArgumentException("IMU yaw must be finite.");
        }
        try {
            Class<?> actions = Class.forName("ca.frc6390.athena.mechanism.core.Actions");
            var factory = actions.getDeclaredMethod("setYaw", ImuSource.class, double.class);
            if (!factory.canAccess(null)) {
                factory.setAccessible(true);
            }
            return (T) factory.invoke(null, source, yawDegrees);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException("IMU actions require athena-mechanisms on the classpath.", exception);
        }
    }

    @SuppressWarnings("unchecked")
    static <T extends DeviceAction> T setYawAction(ImuSource source, DoubleSupplier yawDegrees) {
        Objects.requireNonNull(yawDegrees, "yawDegrees");
        try {
            Class<?> actions = Class.forName("ca.frc6390.athena.mechanism.core.Actions");
            var factory = actions.getDeclaredMethod("setYaw", ImuSource.class, DoubleSupplier.class);
            if (!factory.canAccess(null)) {
                factory.setAccessible(true);
            }
            return (T) factory.invoke(null, source, yawDegrees);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException("IMU actions require athena-mechanisms on the classpath.", exception);
        }
    }

    private abstract static class DerivedSource implements ImuSource {
        @Override
        public double pitchDegrees() {
            throw unsupported("pitch");
        }

        @Override
        public double rollDegrees() {
            throw unsupported("roll");
        }

        @Override
        public double yawRateDegreesPerSecond() {
            throw unsupported("yaw rate");
        }

        @Override
        public double pitchRateDegreesPerSecond() {
            throw unsupported("pitch rate");
        }

        @Override
        public double rollRateDegreesPerSecond() {
            throw unsupported("roll rate");
        }

        @Override
        public double linearAccelerationXG() {
            throw unsupported("X acceleration");
        }

        @Override
        public double linearAccelerationYG() {
            throw unsupported("Y acceleration");
        }

        @Override
        public double linearAccelerationZG() {
            throw unsupported("Z acceleration");
        }

        private UnsupportedOperationException unsupported(String reading) {
            return new UnsupportedOperationException("IMU source does not provide " + reading + ".");
        }
    }

    private static final class RelativeSource implements ImuSource {
        private final ImuSource source;
        private double yawOffset = Double.NaN;
        private double angleOffset = Double.NaN;

        private RelativeSource(ImuSource source) {
            this.source = source;
        }

        @Override
        public double yawDegrees() {
            initializeOffsets();
            return source.yawDegrees() - yawOffset;
        }

        @Override
        public double pitchDegrees() {
            return source.pitchDegrees();
        }

        @Override
        public double rollDegrees() {
            return source.rollDegrees();
        }

        @Override
        public double angleDegrees() {
            initializeOffsets();
            return source.angleDegrees() - angleOffset;
        }

        @Override
        public double yawRateDegreesPerSecond() {
            return source.yawRateDegreesPerSecond();
        }

        @Override
        public double pitchRateDegreesPerSecond() {
            return source.pitchRateDegreesPerSecond();
        }

        @Override
        public double rollRateDegreesPerSecond() {
            return source.rollRateDegreesPerSecond();
        }

        @Override
        public double linearAccelerationXG() {
            return source.linearAccelerationXG();
        }

        @Override
        public double linearAccelerationYG() {
            return source.linearAccelerationYG();
        }

        @Override
        public double linearAccelerationZG() {
            return source.linearAccelerationZG();
        }

        @Override
        public List<?> dependencies() {
            return source.dependencies();
        }

        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double lastUpdateSeconds() { return source.lastUpdateSeconds(); }

        @Override
        public void applyYaw(ActionContext context, double yawDegrees) {
            yawOffset = source.yawDegrees() - yawDegrees;
            angleOffset = source.angleDegrees() - yawDegrees;
        }

        private void initializeOffsets() {
            if (!Double.isFinite(yawOffset)) {
                yawOffset = source.yawDegrees();
                angleOffset = source.angleDegrees();
            }
        }
    }

    private static final class InvertedSource implements ImuSource {
        private final ImuSource source;

        private InvertedSource(ImuSource source) {
            this.source = source;
        }

        @Override
        public double yawDegrees() {
            return -source.yawDegrees();
        }

        @Override
        public double pitchDegrees() {
            return -source.pitchDegrees();
        }

        @Override
        public double rollDegrees() {
            return -source.rollDegrees();
        }

        @Override
        public double angleDegrees() {
            return -source.angleDegrees();
        }

        @Override
        public double yawRateDegreesPerSecond() {
            return -source.yawRateDegreesPerSecond();
        }

        @Override
        public double pitchRateDegreesPerSecond() {
            return -source.pitchRateDegreesPerSecond();
        }

        @Override
        public double rollRateDegreesPerSecond() {
            return -source.rollRateDegreesPerSecond();
        }

        @Override
        public double linearAccelerationXG() {
            return -source.linearAccelerationXG();
        }

        @Override
        public double linearAccelerationYG() {
            return -source.linearAccelerationYG();
        }

        @Override
        public double linearAccelerationZG() {
            return -source.linearAccelerationZG();
        }

        @Override
        public List<?> dependencies() {
            return source.dependencies();
        }

        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double lastUpdateSeconds() { return source.lastUpdateSeconds(); }

        @Override
        public void applyYaw(ActionContext context, double yawDegrees) {
            source.applyYaw(context, -yawDegrees);
        }
    }

    private static final class TransformedSource implements ImuSource {
        private final ImuSource source;
        private final ImuTransform transform;
        private final ContinuousAngle angle = new ContinuousAngle();

        private TransformedSource(ImuSource source, ImuTransform transform) {
            this.source = source;
            this.transform = transform;
        }

        @Override public double yawDegrees() { return orientation(transform.z()); }
        @Override public double pitchDegrees() { return orientation(transform.y()); }
        @Override public double rollDegrees() { return orientation(transform.x()); }

        @Override
        public double angleDegrees() {
            return transform.preservesYawAxis()
                    ? source.angleDegrees() * transform.z().sign()
                    : angle.update(yawDegrees());
        }

        @Override public double yawRateDegreesPerSecond() { return rate(transform.z()); }
        @Override public double pitchRateDegreesPerSecond() { return rate(transform.y()); }
        @Override public double rollRateDegreesPerSecond() { return rate(transform.x()); }
        @Override public double linearAccelerationXG() { return acceleration(transform.x()); }
        @Override public double linearAccelerationYG() { return acceleration(transform.y()); }
        @Override public double linearAccelerationZG() { return acceleration(transform.z()); }
        @Override public List<?> dependencies() { return source.dependencies(); }
        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double lastUpdateSeconds() { return source.lastUpdateSeconds(); }

        @Override
        public void applyYaw(ActionContext context, double yawDegrees) {
            source.applyYaw(context, transform.sourceYaw(yawDegrees));
            angle.reset(yawDegrees);
        }

        private double orientation(ImuDirection direction) {
            double value = switch (direction.axis()) {
                case X -> source.rollDegrees();
                case Y -> source.pitchDegrees();
                case Z -> source.yawDegrees();
            };
            return value * direction.sign();
        }

        private double rate(ImuDirection direction) {
            double value = switch (direction.axis()) {
                case X -> source.rollRateDegreesPerSecond();
                case Y -> source.pitchRateDegreesPerSecond();
                case Z -> source.yawRateDegreesPerSecond();
            };
            return value * direction.sign();
        }

        private double acceleration(ImuDirection direction) {
            double value = switch (direction.axis()) {
                case X -> source.linearAccelerationXG();
                case Y -> source.linearAccelerationYG();
                case Z -> source.linearAccelerationZG();
            };
            return value * direction.sign();
        }
    }

    private static final class MountedSource implements ImuSource {
        private final ImuSource source;
        private final ImuMount mount;
        private final ContinuousAngle angle = new ContinuousAngle();

        private MountedSource(ImuSource source, ImuMount mount) {
            this.source = source;
            this.mount = mount;
        }

        @Override public double yawDegrees() { return orientation().yawDegrees(); }
        @Override public double pitchDegrees() { return orientation().pitchDegrees(); }
        @Override public double rollDegrees() { return orientation().rollDegrees(); }
        @Override
        public double angleDegrees() {
            double robotYaw = yawDegrees();
            return angle.update(
                    robotYaw,
                    mount.initialContinuousYaw(source.yawDegrees(), source.angleDegrees(), robotYaw));
        }
        @Override public double yawRateDegreesPerSecond() { return rates().z(); }
        @Override public double pitchRateDegreesPerSecond() { return rates().y(); }
        @Override public double rollRateDegreesPerSecond() { return rates().x(); }
        @Override public double linearAccelerationXG() { return acceleration().x(); }
        @Override public double linearAccelerationYG() { return acceleration().y(); }
        @Override public double linearAccelerationZG() { return acceleration().z(); }
        @Override public List<?> dependencies() { return source.dependencies(); }
        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double lastUpdateSeconds() { return source.lastUpdateSeconds(); }

        @Override
        public void applyYaw(ActionContext context, double yawDegrees) {
            ImuMount.Orientation robot = orientation();
            ImuMount.Orientation sensor = mount.sensorOrientation(
                    robot.rollDegrees(), robot.pitchDegrees(), yawDegrees);
            source.applyYaw(context, sensor.yawDegrees());
            angle.reset(yawDegrees);
        }

        private ImuMount.Orientation orientation() {
            return mount.robotOrientation(
                    source.rollDegrees(), source.pitchDegrees(), source.yawDegrees());
        }

        private ImuMount.Vector rates() {
            return mount.rotateVector(
                    source.rollRateDegreesPerSecond(),
                    source.pitchRateDegreesPerSecond(),
                    source.yawRateDegreesPerSecond());
        }

        private ImuMount.Vector acceleration() {
            return mount.rotateVector(
                    source.linearAccelerationXG(),
                    source.linearAccelerationYG(),
                    source.linearAccelerationZG());
        }
    }

    private static final class ContinuousAngle {
        private double previous = Double.NaN;
        private double continuous;

        private synchronized double update(double wrappedDegrees) {
            return update(wrappedDegrees, wrappedDegrees);
        }

        private synchronized double update(double wrappedDegrees, double initialDegrees) {
            if (!Double.isFinite(previous)) {
                previous = wrappedDegrees;
                continuous = initialDegrees;
                return continuous;
            }
            double delta = Math.IEEEremainder(wrappedDegrees - previous, 360.0);
            continuous += delta;
            previous = wrappedDegrees;
            return continuous;
        }

        private synchronized void reset(double degrees) {
            previous = degrees;
            continuous = degrees;
        }
    }

    private static final class YawSource extends DerivedSource {
        private final DoubleSupplier source;
        private double offset;

        private YawSource(DoubleSupplier source) {
            this.source = source;
        }

        @Override
        public double yawDegrees() {
            return source.getAsDouble() - offset;
        }

        @Override
        public double angleDegrees() {
            return yawDegrees();
        }

        @Override
        public void applyYaw(ActionContext context, double yawDegrees) {
            offset = source.getAsDouble() - yawDegrees;
        }
    }
}
