package ca.frc6390.athena.hardware.signal;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.DeviceAction;
import ca.frc6390.athena.hardware.runtime.DeviceActionFactories;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

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

    /** Returns a source whose yaw is read from a supplier, relative to the yaw it had when first set. */
    public static ImuSource yaw(Supplier<Angle> yaw) {
        return new YawSource(Objects.requireNonNull(yaw, "yaw"));
    }

    @SuppressWarnings("unchecked")
    static <T extends DeviceAction> T setYawAction(ImuSource source, Angle yaw) {
        double degrees = yaw.in(Units.Degrees);
        if (!Double.isFinite(degrees)) {
            throw new IllegalArgumentException("IMU yaw must be finite.");
        }
        return (T) DeviceActionFactories.get().imuSetYaw(source, degrees);
    }

    @SuppressWarnings("unchecked")
    static <T extends DeviceAction> T setYawAction(ImuSource source, Supplier<Angle> yaw) {
        Objects.requireNonNull(yaw, "yaw");
        return (T) DeviceActionFactories.get().imuSetYaw(source, () -> yaw.get().in(Units.Degrees));
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
        public void applyYawDegrees(ActionContext context, double yawDegrees) {
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
        public void applyYawDegrees(ActionContext context, double yawDegrees) {
            source.applyYawDegrees(context, -yawDegrees);
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
        public void applyYawDegrees(ActionContext context, double yawDegrees) {
            source.applyYawDegrees(context, transform.sourceYaw(yawDegrees));
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
        public void applyYawDegrees(ActionContext context, double yawDegrees) {
            ImuMount.Orientation robot = orientation();
            ImuMount.Orientation sensor = mount.sensorOrientation(
                    robot.rollDegrees(), robot.pitchDegrees(), yawDegrees);
            source.applyYawDegrees(context, sensor.yawDegrees());
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
        private final Supplier<Angle> source;
        private double offset;

        private YawSource(Supplier<Angle> source) {
            this.source = source;
        }

        @Override
        public double yawDegrees() {
            return source.get().in(Units.Degrees) - offset;
        }

        @Override
        public double angleDegrees() {
            return yawDegrees();
        }

        @Override
        public void applyYawDegrees(ActionContext context, double yawDegrees) {
            offset = source.get().in(Units.Degrees) - yawDegrees;
        }
    }
}
