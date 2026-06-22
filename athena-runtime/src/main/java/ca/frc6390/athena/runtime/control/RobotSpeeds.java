package ca.frc6390.athena.runtime.control;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Blends named robot velocity sources into one commanded output.
 */
public final class RobotSpeeds {
    /** Driver control source. */
    public static final String DRIVE_SOURCE = "drive";

    /** Autonomous path source. */
    public static final String AUTO_SOURCE = "auto";

    /** Feedback correction source. */
    public static final String FEEDBACK_SOURCE = "feedback";

    /**
     * Blend operation.
     */
    public enum BlendMode {
        /** Adds right to left. */
        ADD,

        /** Subtracts right from left. */
        SUBTRACT,

        /** Multiplies left by right. */
        MULTIPLY,

        /** Divides left by right when safe. */
        DIVIDE,

        /** Averages left and right. */
        AVERAGE,

        /** Uses the smaller value. */
        MIN,

        /** Uses the larger value. */
        MAX,

        /** Keeps the existing output. */
        A_SUPERSEDES_B,

        /** Replaces the existing output with the source value. */
        B_SUPERSEDES_A
    }

    /**
     * Velocity axes.
     */
    public enum SpeedAxis {
        /** Forward velocity. */
        X,

        /** Left velocity. */
        Y,

        /** Angular velocity. */
        THETA,

        /** All axes. */
        ALL
    }

    private final double maxLinearMetersPerSecond;
    private final double maxAngularRadiansPerSecond;
    private final Map<String, SpeedSource> sources = new LinkedHashMap<>();
    private final List<SourceBlendRule> sourceBlends = new ArrayList<>();
    private final List<OutputBlendRule> outputBlends = new ArrayList<>();
    private double nowSeconds;

    /**
     * Creates a robot speed blender with default drive, auto, and feedback sources.
     *
     * @param maxLinearMetersPerSecond max linear output
     * @param maxAngularRadiansPerSecond max angular output
     */
    public RobotSpeeds(double maxLinearMetersPerSecond, double maxAngularRadiansPerSecond) {
        this.maxLinearMetersPerSecond = sanitizePositive(maxLinearMetersPerSecond);
        this.maxAngularRadiansPerSecond = sanitizePositive(maxAngularRadiansPerSecond);
        registerSource(DRIVE_SOURCE);
        registerSource(AUTO_SOURCE);
        registerSource(FEEDBACK_SOURCE);
        outputBlends.add(new OutputBlendRule(DRIVE_SOURCE, BlendMode.ADD, EnumSet.of(SpeedAxis.ALL)));
        outputBlends.add(new OutputBlendRule(AUTO_SOURCE, BlendMode.ADD, EnumSet.of(SpeedAxis.ALL)));
        outputBlends.add(new OutputBlendRule(FEEDBACK_SOURCE, BlendMode.ADD, EnumSet.of(SpeedAxis.ALL)));
    }

    /**
     * Sets a deterministic timestamp used for source update tracking.
     *
     * @param nowSeconds current timestamp
     * @return this speed blender
     */
    public RobotSpeeds nowSeconds(double nowSeconds) {
        this.nowSeconds = Double.isFinite(nowSeconds) ? nowSeconds : this.nowSeconds;
        return this;
    }

    /**
     * Registers a named speed source.
     *
     * @param name source name
     * @return this speed blender
     */
    public RobotSpeeds registerSource(String name) {
        sources.computeIfAbsent(normalizeName(name), SpeedSource::new);
        return this;
    }

    /**
     * Sets robot-relative speeds for a source.
     *
     * @param source source name
     * @param xMetersPerSecond forward velocity
     * @param yMetersPerSecond left velocity
     * @param angularRadiansPerSecond angular velocity
     * @return this speed blender
     */
    public RobotSpeeds setSpeeds(
            String source,
            double xMetersPerSecond,
            double yMetersPerSecond,
            double angularRadiansPerSecond) {
        requireSource(source).set(new RobotVelocity(xMetersPerSecond, yMetersPerSecond, angularRadiansPerSecond), false, nowSeconds);
        return this;
    }

    /**
     * Sets field-relative speeds for a source.
     *
     * @param source source name
     * @param xMetersPerSecond field-forward velocity
     * @param yMetersPerSecond field-left velocity
     * @param angularRadiansPerSecond angular velocity
     * @return this speed blender
     */
    public RobotSpeeds setFieldRelativeSpeeds(
            String source,
            double xMetersPerSecond,
            double yMetersPerSecond,
            double angularRadiansPerSecond) {
        requireSource(source).set(new RobotVelocity(xMetersPerSecond, yMetersPerSecond, angularRadiansPerSecond), true, nowSeconds);
        return this;
    }

    /**
     * Clears output and source blend rules.
     *
     * @return this speed blender
     */
    public RobotSpeeds clearBlends() {
        sourceBlends.clear();
        outputBlends.clear();
        return this;
    }

    /**
     * Adds a source-to-source blend rule.
     *
     * @param target source to write
     * @param left left input source
     * @param right right input source
     * @param mode blend mode
     * @param axes axes to apply
     * @return this speed blender
     */
    public RobotSpeeds blend(String target, String left, String right, BlendMode mode, SpeedAxis... axes) {
        SourceBlendRule rule = new SourceBlendRule(
                normalizeName(target),
                normalizeName(left),
                normalizeName(right),
                mode == null ? BlendMode.ADD : mode,
                axisSet(axes));
        requireSource(rule.target());
        requireSource(rule.left());
        requireSource(rule.right());
        sourceBlends.add(rule);
        if (hasCycle()) {
            sourceBlends.remove(sourceBlends.size() - 1);
            throw new IllegalStateException("Source blend creates a cycle.");
        }
        return this;
    }

    /**
     * Adds a source-to-output blend rule.
     *
     * @param source source to apply
     * @param mode blend mode
     * @param axes axes to apply
     * @return this speed blender
     */
    public RobotSpeeds blendToOutput(String source, BlendMode mode, SpeedAxis... axes) {
        requireSource(source);
        outputBlends.add(new OutputBlendRule(normalizeName(source), mode == null ? BlendMode.ADD : mode, axisSet(axes)));
        return this;
    }

    /**
     * Calculates the output using robot-relative sources only.
     *
     * @return output velocity
     */
    public RobotVelocity calculate() {
        return calculate(0.0);
    }

    /**
     * Calculates output, converting field-relative sources using heading.
     *
     * @param headingRadians robot heading in radians
     * @return output velocity
     */
    public RobotVelocity calculate(double headingRadians) {
        Map<String, RobotVelocity> values = new LinkedHashMap<>();
        sources.forEach((name, source) -> values.put(name, source.output(headingRadians)));
        for (SourceBlendRule rule : sourceBlends) {
            values.put(rule.target(), apply(values.get(rule.target()), values.get(rule.left()), values.get(rule.right()), rule));
        }
        RobotVelocity output = RobotVelocity.zero();
        for (OutputBlendRule rule : outputBlends) {
            output = apply(output, output, values.get(rule.source()), rule);
        }
        return output.clamp(maxLinearMetersPerSecond, maxAngularRadiansPerSecond);
    }

    /**
     * Returns source update time.
     *
     * @param source source name
     * @return last update timestamp
     */
    public double lastUpdateSeconds(String source) {
        return requireSource(source).lastUpdateSeconds();
    }

    /**
     * Returns whether a source is currently field-relative.
     *
     * @param source source name
     * @return true when field-relative
     */
    public boolean isFieldRelative(String source) {
        return requireSource(source).fieldRelative();
    }

    private RobotVelocity apply(RobotVelocity target, RobotVelocity left, RobotVelocity right, BlendRule rule) {
        return new RobotVelocity(
                rule.appliesTo(SpeedAxis.X) ? blendValue(left.xMetersPerSecond(), right.xMetersPerSecond(), rule.mode()) : target.xMetersPerSecond(),
                rule.appliesTo(SpeedAxis.Y) ? blendValue(left.yMetersPerSecond(), right.yMetersPerSecond(), rule.mode()) : target.yMetersPerSecond(),
                rule.appliesTo(SpeedAxis.THETA)
                        ? blendValue(left.angularRadiansPerSecond(), right.angularRadiansPerSecond(), rule.mode())
                        : target.angularRadiansPerSecond());
    }

    private double blendValue(double left, double right, BlendMode mode) {
        return switch (mode) {
            case ADD -> left + right;
            case SUBTRACT -> left - right;
            case MULTIPLY -> left * right;
            case DIVIDE -> Math.abs(right) < 1.0e-9 ? left : left / right;
            case AVERAGE -> (left + right) / 2.0;
            case MIN -> Math.min(left, right);
            case MAX -> Math.max(left, right);
            case A_SUPERSEDES_B -> left;
            case B_SUPERSEDES_A -> right;
        };
    }

    private boolean hasCycle() {
        for (String source : sources.keySet()) {
            if (visitsDependency(source, source, new java.util.HashSet<>())) {
                return true;
            }
        }
        return false;
    }

    private boolean visitsDependency(String start, String current, java.util.Set<String> visited) {
        if (!visited.add(current)) {
            return false;
        }
        for (SourceBlendRule rule : sourceBlends) {
            if (!rule.target().equals(current)) {
                continue;
            }
            if ((!rule.left().equals(current) && (rule.left().equals(start) || visitsDependency(start, rule.left(), visited)))
                    || (!rule.right().equals(current) && (rule.right().equals(start)
                    || visitsDependency(start, rule.right(), visited)))) {
                return true;
            }
        }
        return false;
    }

    private SpeedSource requireSource(String source) {
        SpeedSource speedSource = sources.get(normalizeName(source));
        if (speedSource == null) {
            throw new IllegalArgumentException("Unknown speed source " + source + ".");
        }
        return speedSource;
    }

    private EnumSet<SpeedAxis> axisSet(SpeedAxis... axes) {
        if (axes == null || axes.length == 0) {
            return EnumSet.of(SpeedAxis.ALL);
        }
        EnumSet<SpeedAxis> set = EnumSet.noneOf(SpeedAxis.class);
        for (SpeedAxis axis : axes) {
            set.add(axis == null ? SpeedAxis.ALL : axis);
        }
        return set.isEmpty() ? EnumSet.of(SpeedAxis.ALL) : set;
    }

    private static String normalizeName(String source) {
        if (source == null || source.isBlank()) {
            throw new IllegalArgumentException("Speed source name cannot be blank.");
        }
        return source;
    }

    private static double sanitizePositive(double value) {
        return Double.isFinite(value) && value > 0.0 ? value : 0.0;
    }

    private interface BlendRule {
        BlendMode mode();

        EnumSet<SpeedAxis> axes();

        default boolean appliesTo(SpeedAxis axis) {
            return axes().contains(SpeedAxis.ALL) || axes().contains(axis);
        }
    }

    private record SourceBlendRule(
            String target,
            String left,
            String right,
            BlendMode mode,
            EnumSet<SpeedAxis> axes) implements BlendRule {
    }

    private record OutputBlendRule(
            String source,
            BlendMode mode,
            EnumSet<SpeedAxis> axes) implements BlendRule {
    }

    private static final class SpeedSource {
        private final String name;
        private RobotVelocity velocity = RobotVelocity.zero();
        private boolean enabled = true;
        private boolean fieldRelative;
        private double lastUpdateSeconds = Double.NaN;

        private SpeedSource(String name) {
            this.name = Objects.requireNonNull(name, "name");
        }

        private void set(RobotVelocity velocity, boolean fieldRelative, double nowSeconds) {
            this.velocity = velocity;
            this.fieldRelative = fieldRelative;
            this.lastUpdateSeconds = nowSeconds;
        }

        private RobotVelocity output(double headingRadians) {
            if (!enabled) {
                return RobotVelocity.zero();
            }
            return fieldRelative ? velocity.fieldToRobot(headingRadians) : velocity;
        }

        private boolean fieldRelative() {
            return fieldRelative;
        }

        private double lastUpdateSeconds() {
            return lastUpdateSeconds;
        }
    }
}
