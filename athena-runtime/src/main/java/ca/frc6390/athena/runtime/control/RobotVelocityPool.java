package ca.frc6390.athena.runtime.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Combines independently controlled velocity channels into one chassis request. */
public final class RobotVelocityPool {
    private final List<Channel> channels = new CopyOnWriteArrayList<>();
    private final List<AngularChannel> angularChannels = new CopyOnWriteArrayList<>();
    private final java.util.concurrent.atomic.AtomicLong activationSequence =
            new java.util.concurrent.atomic.AtomicLong();

    public Channel channel() {
        Channel channel = new Channel();
        channels.add(channel);
        return channel;
    }

    /** Creates a channel that replaces only angular velocity while active. */
    public AngularChannel angularChannel() {
        AngularChannel channel = new AngularChannel();
        angularChannels.add(channel);
        return channel;
    }

    public List<Channel> channels() {
        return List.copyOf(channels);
    }

    public RobotVelocity robotRelative(Rotation2d heading) {
        return combined(VelocityFrame.ROBOT, heading);
    }

    public RobotVelocity fieldRelative(Rotation2d heading) {
        return combined(VelocityFrame.FIELD, heading);
    }

    public RobotVelocity combined(VelocityFrame frame, Rotation2d heading) {
        VelocityFrame safeFrame = Objects.requireNonNull(frame, "frame");
        RobotVelocity combined = RobotVelocity.zero(safeFrame);
        for (Channel channel : channels) {
            RobotVelocity contribution = channel.contribution();
            if (contribution != null) {
                combined = combined.plus(contribution.inFrame(safeFrame, heading));
            }
        }
        AngularChannel angular = null;
        for (AngularChannel candidate : angularChannels) {
            if (candidate.isActive() && (angular == null || candidate.sequence > angular.sequence)) {
                angular = candidate;
            }
        }
        if (angular != null) {
            combined = combined.withAngular(angular.angularVelocity());
        }
        return combined;
    }

    /** Scalar control destination that owns pooled angular velocity while active. */
    public final class AngularChannel implements ControlSink {
        private volatile double value;
        private volatile boolean active;
        private volatile long sequence;
        private volatile boolean inverted;

        /** Applies a scalar control-loop result, interpreted as radians per second. */
        @Override
        public void apply(double radiansPerSecond) {
            if (!Double.isFinite(radiansPerSecond)) {
                throw new IllegalArgumentException("Angular velocity must be finite.");
            }
            this.value = radiansPerSecond;
            active = true;
            sequence = activationSequence.incrementAndGet();
        }

        /** Applies a typed angular velocity request. */
        public void apply(AngularVelocity angularVelocity) {
            apply(Objects.requireNonNull(angularVelocity, "angularVelocity").in(Units.RadiansPerSecond));
        }

        @Override
        public void release() {
            value = 0.0;
            active = false;
        }

        public boolean isActive() {
            return active;
        }

        /** Returns the angular velocity currently owned by this channel, or zero when inactive. */
        public AngularVelocity angularVelocity() {
            if (!active) {
                return Units.RadiansPerSecond.zero();
            }
            return Units.RadiansPerSecond.of(inverted ? -value : value);
        }

        /** Reverses values applied by a control binding. */
        public AngularChannel inverted() {
            return inverted(true);
        }

        /** Configures whether values applied by a control binding are reversed. */
        public AngularChannel inverted(boolean inverted) {
            this.inverted = inverted;
            return this;
        }

        public boolean isInverted() {
            return inverted;
        }
    }

    /** One independently enabled and weighted velocity contribution. */
    public static final class Channel {
        private volatile Supplier<RobotVelocity> source;
        private volatile BooleanSupplier enabled = () -> true;
        private volatile DoubleSupplier weight = () -> 1.0;
        private volatile boolean invertX;
        private volatile boolean invertY;
        private volatile boolean invertAngular;

        public Channel set(RobotVelocity velocity) {
            RobotVelocity safe = Objects.requireNonNull(velocity, "velocity");
            source = () -> safe;
            return this;
        }

        public Channel set(Supplier<RobotVelocity> velocity) {
            source = Objects.requireNonNull(velocity, "velocity");
            return this;
        }

        public Channel clear() {
            source = null;
            return this;
        }

        public Channel enabled(BooleanSupplier condition) {
            enabled = Objects.requireNonNull(condition, "condition");
            return this;
        }

        public Channel weight(double value) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("Velocity channel weight must be finite.");
            }
            weight = () -> value;
            return this;
        }

        public Channel weight(DoubleSupplier value) {
            weight = Objects.requireNonNull(value, "value");
            return this;
        }

        /** Reverses every component contributed by this channel. */
        public Channel inverted() {
            return inverted(true);
        }

        /** Configures whether every component contributed by this channel is reversed. */
        public Channel inverted(boolean inverted) {
            return inverted(inverted, inverted, inverted);
        }

        /** Configures inversion independently for each velocity component. */
        public Channel inverted(boolean x, boolean y, boolean angular) {
            invertX = x;
            invertY = y;
            invertAngular = angular;
            return this;
        }

        public boolean isActive() {
            return source != null && enabled.getAsBoolean();
        }

        public RobotVelocity velocity() {
            RobotVelocity value = contribution();
            return value == null ? RobotVelocity.zero() : value;
        }

        private RobotVelocity contribution() {
            Supplier<RobotVelocity> current = source;
            if (current == null || !enabled.getAsBoolean()) {
                return null;
            }
            RobotVelocity velocity = current.get();
            if (velocity == null) {
                return null;
            }
            double currentWeight = weight.getAsDouble();
            return velocity
                    .inverted(invertX, invertY, invertAngular)
                    .times(Double.isFinite(currentWeight) ? currentWeight : 0.0);
        }
    }
}
