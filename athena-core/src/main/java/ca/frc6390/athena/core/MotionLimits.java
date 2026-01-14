package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public final class MotionLimits {

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

        public static DriveLimits fromRobotSpeeds(RobotSpeeds speeds) {
            if (speeds == null) {
                return none();
            }
            return new DriveLimits(speeds.getMaxVelocity(), 0.0, speeds.getMaxAngularVelocity(), 0.0);
        }

        public static DriveLimits none() {
            return new DriveLimits(0.0, 0.0, 0.0, 0.0);
        }
    }

    public record AxisLimits(double maxVelocity, double maxAcceleration) {
        public AxisLimits {
            maxVelocity = sanitizePositive(maxVelocity);
            maxAcceleration = sanitizePositive(maxAcceleration);
        }

        public static AxisLimits none() {
            return new AxisLimits(0.0, 0.0);
        }
    }

    @FunctionalInterface
    public interface DriveLimitsProvider {
        DriveLimits get();
    }

    @FunctionalInterface
    public interface AxisLimitsProvider {
        AxisLimits get();
    }

    private DriveLimits baseDriveLimits = DriveLimits.none();
    private final List<DriveLimitsProvider> driveProviders = new ArrayList<>();
    private final Map<String, AxisGroup> axisGroups = new HashMap<>();

    public MotionLimits setBaseDriveLimits(DriveLimits limits) {
        baseDriveLimits = limits != null ? limits : DriveLimits.none();
        return this;
    }

    public DriveLimits getBaseDriveLimits() {
        return baseDriveLimits;
    }

    public MotionLimits registerDriveProvider(DriveLimitsProvider provider) {
        Objects.requireNonNull(provider, "provider");
        driveProviders.add(provider);
        return this;
    }

    public DriveLimits resolveDriveLimits() {
        DriveLimits resolved = baseDriveLimits;
        for (DriveLimitsProvider provider : driveProviders) {
            DriveLimits candidate = safeDrive(provider.get());
            resolved = combine(resolved, candidate);
        }
        return resolved;
    }

    public MotionLimits setBaseAxisLimits(String axisId, AxisLimits limits) {
        axisGroup(axisId).base = limits != null ? limits : AxisLimits.none();
        return this;
    }

    public MotionLimits registerAxisProvider(String axisId, AxisLimitsProvider provider) {
        Objects.requireNonNull(provider, "provider");
        axisGroup(axisId).providers.add(provider);
        return this;
    }

    public AxisLimits resolveAxisLimits(String axisId) {
        AxisGroup group = axisGroups.get(axisId);
        if (group == null) {
            return AxisLimits.none();
        }
        AxisLimits resolved = group.base;
        for (AxisLimitsProvider provider : group.providers) {
            AxisLimits candidate = safeAxis(provider.get());
            resolved = combine(resolved, candidate);
        }
        return resolved;
    }

    private AxisGroup axisGroup(String axisId) {
        Objects.requireNonNull(axisId, "axisId");
        return axisGroups.computeIfAbsent(axisId, key -> new AxisGroup());
    }

    private static DriveLimits combine(DriveLimits base, DriveLimits candidate) {
        return new DriveLimits(
                combineLimit(base.maxLinearVelocity(), candidate.maxLinearVelocity()),
                combineLimit(base.maxLinearAcceleration(), candidate.maxLinearAcceleration()),
                combineLimit(base.maxAngularVelocity(), candidate.maxAngularVelocity()),
                combineLimit(base.maxAngularAcceleration(), candidate.maxAngularAcceleration()));
    }

    private static AxisLimits combine(AxisLimits base, AxisLimits candidate) {
        return new AxisLimits(
                combineLimit(base.maxVelocity(), candidate.maxVelocity()),
                combineLimit(base.maxAcceleration(), candidate.maxAcceleration()));
    }

    private static double combineLimit(double current, double candidate) {
        if (!Double.isFinite(candidate) || candidate <= 0.0) {
            return current;
        }
        if (!Double.isFinite(current) || current <= 0.0) {
            return candidate;
        }
        return Math.min(current, candidate);
    }

    private static DriveLimits safeDrive(DriveLimits limits) {
        return limits != null ? limits : DriveLimits.none();
    }

    private static AxisLimits safeAxis(AxisLimits limits) {
        return limits != null ? limits : AxisLimits.none();
    }

    private static double sanitizePositive(double value) {
        return Double.isFinite(value) && value > 0.0 ? value : 0.0;
    }

    private static final class AxisGroup {
        private AxisLimits base = AxisLimits.none();
        private final List<AxisLimitsProvider> providers = new ArrayList<>();
    }
}
