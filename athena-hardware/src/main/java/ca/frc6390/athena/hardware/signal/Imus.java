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
