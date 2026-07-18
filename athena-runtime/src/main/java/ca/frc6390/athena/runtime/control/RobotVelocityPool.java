package ca.frc6390.athena.runtime.control;

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

    public RobotVelocity robotRelative(double headingRadians) {
        return combined(VelocityFrame.ROBOT, headingRadians);
    }

    public RobotVelocity fieldRelative(double headingRadians) {
        return combined(VelocityFrame.FIELD, headingRadians);
    }

    public RobotVelocity combined(VelocityFrame frame, double headingRadians) {
        VelocityFrame safeFrame = Objects.requireNonNull(frame, "frame");
        RobotVelocity combined = RobotVelocity.zero(safeFrame);
        for (Channel channel : channels) {
            RobotVelocity contribution = channel.contribution();
            if (contribution != null) {
                combined = combined.plus(contribution.inFrame(safeFrame, headingRadians));
            }
        }
        AngularChannel angular = null;
        for (AngularChannel candidate : angularChannels) {
            if (candidate.isActive() && (angular == null || candidate.sequence > angular.sequence)) {
                angular = candidate;
            }
        }
        if (angular != null) {
            combined = combined.withAngular(angular.radiansPerSecond());
        }
        return combined;
    }

    /** Scalar control destination that owns pooled angular velocity while active. */
    public final class AngularChannel implements ControlSink {
        private volatile double value;
        private volatile boolean active;
        private volatile long sequence;
        private volatile boolean inverted;

        @Override
        public void apply(double value) {
            if (!Double.isFinite(value)) {
                throw new IllegalArgumentException("Angular velocity must be finite.");
            }
            this.value = value;
            active = true;
            sequence = activationSequence.incrementAndGet();
        }

        @Override
        public void release() {
            value = 0.0;
            active = false;
        }

        public boolean isActive() {
            return active;
        }

        public double radiansPerSecond() {
            if (!active) {
                return 0.0;
            }
            return inverted ? -value : value;
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
