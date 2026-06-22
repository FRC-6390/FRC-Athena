package ca.frc6390.athena.runtime.control;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Aggregates motion limits from base configuration and runtime providers.
 */
public final class MotionLimits {
    private DriveLimits baseDriveLimits = DriveLimits.none();
    private final List<DriveLimitsProvider> driveProviders = new ArrayList<>();
    private final Map<String, AxisGroup> axisGroups = new LinkedHashMap<>();

    /**
     * Motion constraints for differential or holonomic drive motion.
     *
     * @param maxLinearVelocity max linear velocity
     * @param maxLinearAcceleration max linear acceleration
     * @param maxAngularVelocity max angular velocity
     * @param maxAngularAcceleration max angular acceleration
     */
    public record DriveLimits(
            double maxLinearVelocity,
            double maxLinearAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration) {
        public DriveLimits {
            maxLinearVelocity = sanitizePositive(maxLinearVelocity);
            maxLinearAcceleration = sanitizePositive(maxLinearAcceleration);
            maxAngularVelocity = sanitizePositive(maxAngularVelocity);
            maxAngularAcceleration = sanitizePositive(maxAngularAcceleration);
        }

        /**
         * Creates unconstrained drive limits.
         *
         * @return empty drive limits
         */
        public static DriveLimits none() {
            return new DriveLimits(0.0, 0.0, 0.0, 0.0);
        }
    }

    /**
     * Motion constraints for a single mechanism axis.
     *
     * @param maxVelocity max velocity
     * @param maxAcceleration max acceleration
     */
    public record AxisLimits(double maxVelocity, double maxAcceleration) {
        public AxisLimits {
            maxVelocity = sanitizePositive(maxVelocity);
            maxAcceleration = sanitizePositive(maxAcceleration);
        }

        /**
         * Creates unconstrained axis limits.
         *
         * @return empty axis limits
         */
        public static AxisLimits none() {
            return new AxisLimits(0.0, 0.0);
        }
    }

    /**
     * Supplies current drive motion limits.
     */
    @FunctionalInterface
    public interface DriveLimitsProvider {
        /**
         * Returns current limits.
         *
         * @return drive limits
         */
        DriveLimits get();
    }

    /**
     * Supplies current axis motion limits.
     */
    @FunctionalInterface
    public interface AxisLimitsProvider {
        /**
         * Returns current limits.
         *
         * @return axis limits
         */
        AxisLimits get();
    }

    /**
     * Sets base drive limits.
     *
     * @param limits base limits
     * @return this aggregator
     */
    public MotionLimits baseDriveLimits(DriveLimits limits) {
        baseDriveLimits = limits == null ? DriveLimits.none() : limits;
        return this;
    }

    /**
     * Registers a runtime drive limits provider.
     *
     * @param provider provider to add
     * @return this aggregator
     */
    public MotionLimits driveProvider(DriveLimitsProvider provider) {
        driveProviders.add(Objects.requireNonNull(provider, "provider"));
        return this;
    }

    /**
     * Resolves the most conservative valid drive limits.
     *
     * @return resolved drive limits
     */
    public DriveLimits resolveDrive() {
        DriveLimits resolved = baseDriveLimits;
        for (DriveLimitsProvider provider : driveProviders) {
            resolved = combine(resolved, provider.get());
        }
        return resolved;
    }

    /**
     * Sets base limits for a named axis.
     *
     * @param axisId axis identifier
     * @param limits base axis limits
     * @return this aggregator
     */
    public MotionLimits baseAxisLimits(String axisId, AxisLimits limits) {
        axisGroup(axisId).base = limits == null ? AxisLimits.none() : limits;
        return this;
    }

    /**
     * Registers a runtime provider for a named axis.
     *
     * @param axisId axis identifier
     * @param provider provider to add
     * @return this aggregator
     */
    public MotionLimits axisProvider(String axisId, AxisLimitsProvider provider) {
        axisGroup(axisId).providers.add(Objects.requireNonNull(provider, "provider"));
        return this;
    }

    /**
     * Resolves the most conservative valid limits for a named axis.
     *
     * @param axisId axis identifier
     * @return resolved axis limits
     */
    public AxisLimits resolveAxis(String axisId) {
        AxisGroup group = axisGroups.get(axisId);
        if (group == null) {
            return AxisLimits.none();
        }
        AxisLimits resolved = group.base;
        for (AxisLimitsProvider provider : group.providers) {
            resolved = combine(resolved, provider.get());
        }
        return resolved;
    }

    private AxisGroup axisGroup(String axisId) {
        Objects.requireNonNull(axisId, "axisId");
        return axisGroups.computeIfAbsent(axisId, key -> new AxisGroup());
    }

    private static DriveLimits combine(DriveLimits current, DriveLimits candidate) {
        DriveLimits safeCandidate = candidate == null ? DriveLimits.none() : candidate;
        return new DriveLimits(
                combineLimit(current.maxLinearVelocity(), safeCandidate.maxLinearVelocity()),
                combineLimit(current.maxLinearAcceleration(), safeCandidate.maxLinearAcceleration()),
                combineLimit(current.maxAngularVelocity(), safeCandidate.maxAngularVelocity()),
                combineLimit(current.maxAngularAcceleration(), safeCandidate.maxAngularAcceleration()));
    }

    private static AxisLimits combine(AxisLimits current, AxisLimits candidate) {
        AxisLimits safeCandidate = candidate == null ? AxisLimits.none() : candidate;
        return new AxisLimits(
                combineLimit(current.maxVelocity(), safeCandidate.maxVelocity()),
                combineLimit(current.maxAcceleration(), safeCandidate.maxAcceleration()));
    }

    private static double combineLimit(double current, double candidate) {
        if (candidate <= 0.0) {
            return current;
        }
        if (current <= 0.0) {
            return candidate;
        }
        return Math.min(current, candidate);
    }

    private static double sanitizePositive(double value) {
        return Double.isFinite(value) && value > 0.0 ? value : 0.0;
    }

    private static final class AxisGroup {
        private AxisLimits base = AxisLimits.none();
        private final List<AxisLimitsProvider> providers = new ArrayList<>();
    }
}
