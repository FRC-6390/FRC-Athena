package ca.frc6390.athena.mechanism.motion;

import java.util.Objects;

/**
 * Stateful scalar trapezoid-profile runtime.
 */
public final class MotionProfileRuntime {
    private final MotionProfile profile;
    private MotionReference reference;

    public MotionProfileRuntime(MotionProfile profile) {
        this.profile = Objects.requireNonNull(profile, "profile");
    }

    public MotionReference step(double measuredPosition, double measuredVelocity, double goal, double dtSeconds) {
        if (!Double.isFinite(measuredPosition) || !Double.isFinite(measuredVelocity) || !Double.isFinite(goal)) {
            throw new IllegalArgumentException("Profile inputs must be finite.");
        }
        double dt = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.02;
        if (reference == null) {
            reference = new MotionReference(measuredPosition, measuredVelocity, 0.0);
        }

        double position = reference.position();
        double velocity = reference.velocity();
        double error = goal - position;
        if (Math.abs(error) <= 1.0e-9 && Math.abs(velocity) <= 1.0e-9) {
            reference = MotionReference.stationary(goal);
            return reference;
        }

        double direction = Math.signum(error);
        double safeVelocity = Math.sqrt(2.0 * profile.maxAcceleration() * Math.abs(error));
        double desiredVelocity = direction * Math.min(profile.maxVelocity(), safeVelocity);
        double nextVelocity = moveToward(
                velocity,
                desiredVelocity,
                profile.maxAcceleration() * dt);
        double nextPosition = position + (velocity + nextVelocity) * 0.5 * dt;

        if ((goal - position) * (goal - nextPosition) <= 0.0) {
            nextPosition = goal;
            nextVelocity = 0.0;
        }
        double acceleration = (nextVelocity - velocity) / dt;
        reference = new MotionReference(nextPosition, nextVelocity, acceleration);
        return reference;
    }

    public void reset(double position, double velocity) {
        reference = new MotionReference(position, velocity, 0.0);
    }

    public MotionReference reference() {
        return reference;
    }

    private static double moveToward(double current, double target, double maximumChange) {
        double difference = target - current;
        if (Math.abs(difference) <= maximumChange) {
            return target;
        }
        return current + Math.copySign(maximumChange, difference);
    }
}
