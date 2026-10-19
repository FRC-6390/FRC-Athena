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
        public double readPitchDegrees() {
            throw unsupported("pitch");
        }

        @Override
        public double readRollDegrees() {
            throw unsupported("roll");
        }

        @Override
        public double readYawRateDegreesPerSecond() {
            throw unsupported("yaw rate");
        }

        @Override
        public double readPitchRateDegreesPerSecond() {
            throw unsupported("pitch rate");
        }

        @Override
        public double readRollRateDegreesPerSecond() {
            throw unsupported("roll rate");
        }

        @Override
        public double readLinearAccelerationXG() {
            throw unsupported("X acceleration");
        }

        @Override
        public double readLinearAccelerationYG() {
            throw unsupported("Y acceleration");
        }

        @Override
        public double readLinearAccelerationZG() {
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
        public double readYawDegrees() {
            initializeOffsets();
            return source.readYawDegrees() - yawOffset;
        }

        @Override
        public double readPitchDegrees() {
            return source.readPitchDegrees();
        }

        @Override
        public double readRollDegrees() {
            return source.readRollDegrees();
        }

        @Override
        public double readAngleDegrees() {
            initializeOffsets();
            return source.readAngleDegrees() - angleOffset;
        }

        @Override
        public double readYawRateDegreesPerSecond() {
            return source.readYawRateDegreesPerSecond();
        }

        @Override
        public double readPitchRateDegreesPerSecond() {
            return source.readPitchRateDegreesPerSecond();
        }

        @Override
        public double readRollRateDegreesPerSecond() {
            return source.readRollRateDegreesPerSecond();
        }

        @Override
        public double readLinearAccelerationXG() {
            return source.readLinearAccelerationXG();
        }

        @Override
        public double readLinearAccelerationYG() {
            return source.readLinearAccelerationYG();
        }

        @Override
        public double readLinearAccelerationZG() {
            return source.readLinearAccelerationZG();
        }

        @Override
        public List<?> dependencies() {
            return source.dependencies();
        }

        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double readLastUpdateSeconds() { return source.readLastUpdateSeconds(); }

        @Override
        public void writeYawDegrees(ActionContext context, double yawDegrees) {
            yawOffset = source.readYawDegrees() - yawDegrees;
            angleOffset = source.readAngleDegrees() - yawDegrees;
        }

        private void initializeOffsets() {
            if (!Double.isFinite(yawOffset)) {
                yawOffset = source.readYawDegrees();
                angleOffset = source.readAngleDegrees();
            }
        }
    }

    private static final class InvertedSource implements ImuSource {
        private final ImuSource source;

        private InvertedSource(ImuSource source) {
            this.source = source;
        }

        @Override
        public double readYawDegrees() {
            return -source.readYawDegrees();
        }

        @Override
        public double readPitchDegrees() {
            return -source.readPitchDegrees();
        }

        @Override
        public double readRollDegrees() {
            return -source.readRollDegrees();
        }

        @Override
        public double readAngleDegrees() {
            return -source.readAngleDegrees();
        }

        @Override
        public double readYawRateDegreesPerSecond() {
            return -source.readYawRateDegreesPerSecond();
        }

        @Override
        public double readPitchRateDegreesPerSecond() {
            return -source.readPitchRateDegreesPerSecond();
        }

        @Override
        public double readRollRateDegreesPerSecond() {
            return -source.readRollRateDegreesPerSecond();
        }

        @Override
        public double readLinearAccelerationXG() {
            return -source.readLinearAccelerationXG();
        }

        @Override
        public double readLinearAccelerationYG() {
            return -source.readLinearAccelerationYG();
        }

        @Override
        public double readLinearAccelerationZG() {
            return -source.readLinearAccelerationZG();
        }

        @Override
        public List<?> dependencies() {
            return source.dependencies();
        }

        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double readLastUpdateSeconds() { return source.readLastUpdateSeconds(); }

        @Override
        public void writeYawDegrees(ActionContext context, double yawDegrees) {
            source.writeYawDegrees(context, -yawDegrees);
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

        @Override public double readYawDegrees() { return orientation(transform.z()); }
        @Override public double readPitchDegrees() { return orientation(transform.y()); }
        @Override public double readRollDegrees() { return orientation(transform.x()); }

        @Override
        public double readAngleDegrees() {
            return transform.preservesYawAxis()
                    ? source.readAngleDegrees() * transform.z().sign()
                    : angle.update(readYawDegrees());
        }

        @Override public double readYawRateDegreesPerSecond() { return rate(transform.z()); }
        @Override public double readPitchRateDegreesPerSecond() { return rate(transform.y()); }
        @Override public double readRollRateDegreesPerSecond() { return rate(transform.x()); }
        @Override public double readLinearAccelerationXG() { return acceleration(transform.x()); }
        @Override public double readLinearAccelerationYG() { return acceleration(transform.y()); }
        @Override public double readLinearAccelerationZG() { return acceleration(transform.z()); }
        @Override public List<?> dependencies() { return source.dependencies(); }
        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double readLastUpdateSeconds() { return source.readLastUpdateSeconds(); }

        @Override
        public void writeYawDegrees(ActionContext context, double yawDegrees) {
            source.writeYawDegrees(context, transform.sourceYaw(yawDegrees));
            angle.reset(yawDegrees);
        }

        private double orientation(ImuDirection direction) {
            double value = switch (direction.axis()) {
                case X -> source.readRollDegrees();
                case Y -> source.readPitchDegrees();
                case Z -> source.readYawDegrees();
            };
            return value * direction.sign();
        }

        private double rate(ImuDirection direction) {
            double value = switch (direction.axis()) {
                case X -> source.readRollRateDegreesPerSecond();
                case Y -> source.readPitchRateDegreesPerSecond();
                case Z -> source.readYawRateDegreesPerSecond();
            };
            return value * direction.sign();
        }

        private double acceleration(ImuDirection direction) {
            double value = switch (direction.axis()) {
                case X -> source.readLinearAccelerationXG();
                case Y -> source.readLinearAccelerationYG();
                case Z -> source.readLinearAccelerationZG();
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

        @Override public double readYawDegrees() { return orientation().yawDegrees(); }
        @Override public double readPitchDegrees() { return orientation().pitchDegrees(); }
        @Override public double readRollDegrees() { return orientation().rollDegrees(); }
        @Override
        public double readAngleDegrees() {
            double robotYaw = readYawDegrees();
            return angle.update(
                    robotYaw,
                    mount.initialContinuousYaw(source.readYawDegrees(), source.readAngleDegrees(), robotYaw));
        }
        @Override public double readYawRateDegreesPerSecond() { return rates().z(); }
        @Override public double readPitchRateDegreesPerSecond() { return rates().y(); }
        @Override public double readRollRateDegreesPerSecond() { return rates().x(); }
        @Override public double readLinearAccelerationXG() { return acceleration().x(); }
        @Override public double readLinearAccelerationYG() { return acceleration().y(); }
        @Override public double readLinearAccelerationZG() { return acceleration().z(); }
        @Override public List<?> dependencies() { return source.dependencies(); }
        @Override public boolean isConnected() { return source.isConnected(); }
        @Override public boolean isCalibrating() { return source.isCalibrating(); }
        @Override public double readLastUpdateSeconds() { return source.readLastUpdateSeconds(); }

        @Override
        public void writeYawDegrees(ActionContext context, double yawDegrees) {
            ImuMount.Orientation robot = orientation();
            ImuMount.Orientation sensor = mount.sensorOrientation(
                    robot.rollDegrees(), robot.pitchDegrees(), yawDegrees);
            source.writeYawDegrees(context, sensor.yawDegrees());
            angle.reset(yawDegrees);
        }

        private ImuMount.Orientation orientation() {
            return mount.robotOrientation(
                    source.readRollDegrees(), source.readPitchDegrees(), source.readYawDegrees());
        }

        private ImuMount.Vector rates() {
            return mount.rotateVector(
                    source.readRollRateDegreesPerSecond(),
                    source.readPitchRateDegreesPerSecond(),
                    source.readYawRateDegreesPerSecond());
        }

        private ImuMount.Vector acceleration() {
            return mount.rotateVector(
                    source.readLinearAccelerationXG(),
                    source.readLinearAccelerationYG(),
                    source.readLinearAccelerationZG());
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
        public double readYawDegrees() {
            return source.get().in(Units.Degrees) - offset;
        }

        @Override
        public double readAngleDegrees() {
            return readYawDegrees();
        }

        @Override
        public void writeYawDegrees(ActionContext context, double yawDegrees) {
            offset = source.get().in(Units.Degrees) - yawDegrees;
        }
    }
}
