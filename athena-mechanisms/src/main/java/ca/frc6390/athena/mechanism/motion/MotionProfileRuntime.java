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
            reference = MotionReference.stationary(measuredPosition);
        }

        MotionReference current = reference;
        if (Math.abs(goal - current.position()) <= 1.0e-9
                && Math.abs(current.velocity()) <= 1.0e-9) {
            reference = MotionReference.stationary(goal);
            return reference;
        }

        MotionReference calculated = calculate(dt, current, goal);
        double maximumVelocityChange = profile.maxAcceleration() * dt;
        double nextVelocity = moveToward(
                current.velocity(), calculated.velocity(), maximumVelocityChange);
        double nextPosition = calculated.position();
        if (Double.compare(nextVelocity, calculated.velocity()) != 0) {
            // A suddenly changed goal can be physically unreachable from the current velocity.
            // Preserve acceleration continuity and allow the reference to overshoot and recover.
            nextPosition = current.position() + (current.velocity() + nextVelocity) * 0.5 * dt;
        }
        double acceleration = (nextVelocity - current.velocity()) / dt;
        reference = new MotionReference(nextPosition, nextVelocity, acceleration);
        return reference;
    }

    public void reset(double position, double velocity) {
        if (!Double.isFinite(position) || !Double.isFinite(velocity)) {
            throw new IllegalArgumentException("Profile reset inputs must be finite.");
        }
        reference = new MotionReference(
                position,
                clamp(velocity, -profile.maxVelocity(), profile.maxVelocity()),
                0.0);
    }

    public MotionReference reference() {
        return reference;
    }

    private MotionReference calculate(double dt, MotionReference initial, double goalPosition) {
        double direction = initial.position() > goalPosition ? -1.0 : 1.0;
        double position = initial.position() * direction;
        double velocity = clamp(initial.velocity() * direction, -profile.maxVelocity(), profile.maxVelocity());
        double goal = goalPosition * direction;

        double cutoffBegin = velocity / profile.maxAcceleration();
        double cutoffDistanceBegin = cutoffBegin * cutoffBegin * profile.maxAcceleration() / 2.0;
        double fullDistance = cutoffDistanceBegin + goal - position;
        double accelerationTime = profile.maxVelocity() / profile.maxAcceleration();
        double fullSpeedDistance = fullDistance
                - accelerationTime * accelerationTime * profile.maxAcceleration();
        if (fullSpeedDistance < 0.0) {
            accelerationTime = Math.sqrt(Math.max(0.0, fullDistance) / profile.maxAcceleration());
            fullSpeedDistance = 0.0;
        }

        double endAcceleration = accelerationTime - cutoffBegin;
        double endFullSpeed = endAcceleration + fullSpeedDistance / profile.maxVelocity();
        double endDeceleration = endFullSpeed + accelerationTime;
        double nextPosition;
        double nextVelocity;
        if (dt < endAcceleration) {
            nextVelocity = velocity + dt * profile.maxAcceleration();
            nextPosition = position + (velocity + dt * profile.maxAcceleration() / 2.0) * dt;
        } else if (dt < endFullSpeed) {
            nextVelocity = profile.maxVelocity();
            nextPosition = position
                    + (velocity + endAcceleration * profile.maxAcceleration() / 2.0) * endAcceleration
                    + profile.maxVelocity() * (dt - endAcceleration);
        } else if (dt <= endDeceleration) {
            double timeLeft = endDeceleration - dt;
            nextVelocity = timeLeft * profile.maxAcceleration();
            nextPosition = goal - timeLeft * timeLeft * profile.maxAcceleration() / 2.0;
        } else {
            nextPosition = goal;
            nextVelocity = 0.0;
        }
        return new MotionReference(nextPosition * direction, nextVelocity * direction, 0.0);
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private static double moveToward(double current, double target, double maximumChange) {
        double difference = target - current;
        if (Math.abs(difference) <= maximumChange) {
            return target;
        }
        return current + Math.copySign(maximumChange, difference);
    }
}
