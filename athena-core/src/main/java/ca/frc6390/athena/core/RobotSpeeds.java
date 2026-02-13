package ca.frc6390.athena.core;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotSpeeds {

    public static final String DRIVE_SOURCE = "drive";
    public static final String AUTO_SOURCE = "auto";
    public static final String FEEDBACK_SOURCE = "feedback";

    public enum BlendMode {
        ADD,
        SUBTRACT,
        MULTIPLY,
        DIVIDE,
        AVERAGE,
        MIN,
        MAX,
        A_SUPERSEDES_B,
        B_SUPERSEDES_A
    }

    public enum SpeedAxis {
        X,
        Y,
        Theta,
        ALL
    }

    public static final class SourceBlendRule {
        private final String target;
        private final String left;
        private final String right;
        private final BlendMode mode;
        private final EnumSet<SpeedAxis> axes;

        private SourceBlendRule(String target, String left, String right, BlendMode mode, EnumSet<SpeedAxis> axes) {
            this.target = target;
            this.left = left;
            this.right = right;
            this.mode = mode != null ? mode : BlendMode.ADD;
            this.axes = axes != null && !axes.isEmpty() ? axes.clone() : EnumSet.of(SpeedAxis.ALL);
        }

        public String target() {
            return target;
        }

        public String left() {
            return left;
        }

        public String right() {
            return right;
        }

        public BlendMode mode() {
            return mode;
        }

        public Set<SpeedAxis> axes() {
            return EnumSet.copyOf(axes);
        }

        private boolean appliesToAxis(SpeedAxis axis) {
            return axes.contains(SpeedAxis.ALL) || axes.contains(axis);
        }
    }

    public static final class OutputBlendRule {
        private final String source;
        private final BlendMode mode;
        private final EnumSet<SpeedAxis> axes;

        private OutputBlendRule(String source, BlendMode mode, EnumSet<SpeedAxis> axes) {
            this.source = source;
            this.mode = mode != null ? mode : BlendMode.ADD;
            this.axes = axes != null && !axes.isEmpty() ? axes.clone() : EnumSet.of(SpeedAxis.ALL);
        }

        public String source() {
            return source;
        }

        public BlendMode mode() {
            return mode;
        }

        public Set<SpeedAxis> axes() {
            return EnumSet.copyOf(axes);
        }

        private boolean appliesToAxis(SpeedAxis axis) {
            return axes.contains(SpeedAxis.ALL) || axes.contains(axis);
        }
    }

    public static class SpeedSource {

        private final String name;
        private double vxMetersPerSecond;
        private double vyMetersPerSecond;
        private double omegaRadiansPerSecond;
        private boolean enabled;
        private boolean enableX;
        private boolean enableY;
        private boolean enableTheta;

        public SpeedSource(String name) {
            this.name = name;
            this.enabled = true;
            this.enableX = true;
            this.enableY = true;
            this.enableTheta = true;
        }

        public String getName() {
            return name;
        }

        public void setInputSpeeds(ChassisSpeeds speeds) {
            if (speeds == null) {
                setInputSpeeds(0.0, 0.0, 0.0);
                return;
            }
            setInputSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond);
        }

        public void setInputSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
            this.vxMetersPerSecond = vxMetersPerSecond;
            this.vyMetersPerSecond = vyMetersPerSecond;
            this.omegaRadiansPerSecond = omegaRadiansPerSecond;
        }

        public double outputVx() {
            if (!enabled || !enableX) {
                return 0.0;
            }
            return vxMetersPerSecond;
        }

        public double outputVy() {
            if (!enabled || !enableY) {
                return 0.0;
            }
            return vyMetersPerSecond;
        }

        public double outputOmega() {
            if (!enabled || !enableTheta) {
                return 0.0;
            }
            return omegaRadiansPerSecond;
        }

        public ChassisSpeeds getOutputSpeeds() {
            return new ChassisSpeeds(outputVx(), outputVy(), outputOmega());
        }

        public void stop() {
            vxMetersPerSecond = 0.0;
            vyMetersPerSecond = 0.0;
            omegaRadiansPerSecond = 0.0;
        }

        public boolean isEnabled() {
            return enabled;
        }

        public void setEnabled(boolean enabled) {
            this.enabled = enabled;
        }

        public void setAxisState(SpeedAxis axis, boolean enabled) {
            switch (axis) {
                case X:
                    enableX = enabled;
                    break;
                case Y:
                    enableY = enabled;
                    break;
                case Theta:
                    enableTheta = enabled;
                    break;
                case ALL:
                    enableX = enabled;
                    enableY = enabled;
                    enableTheta = enabled;
                    break;
                default:
                    break;
            }
        }

        public boolean isAxisActive(SpeedAxis axis) {
            switch (axis) {
                case X:
                    return enableX;
                case Y:
                    return enableY;
                case Theta:
                    return enableTheta;
                case ALL:
                    return enableX && enableY && enableTheta;
                default:
                    return false;
            }
        }

        public boolean contributesOnAxis(SpeedAxis axis) {
            return enabled && isAxisActive(axis);
        }

        public double output(SpeedAxis axis) {
            switch (axis) {
                case X:
                    return outputVx();
                case Y:
                    return outputVy();
                case Theta:
                    return outputOmega();
                default:
                    return 0.0;
            }
        }
    }

    private static final double EPSILON = 1E-9;
    private static final SpeedAxis[] AXES = new SpeedAxis[] { SpeedAxis.X, SpeedAxis.Y, SpeedAxis.Theta };

    private final LinkedHashMap<String, SpeedSource> sources;
    private final List<SourceBlendRule> sourceBlendRules;
    private final List<OutputBlendRule> outputBlendRules;

    private final double maxVelocity;
    private final double maxAngularVelocity;

    public RobotSpeeds(double maxVelocity, double maxAngularVelocity) {
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.sources = new LinkedHashMap<>();
        this.sourceBlendRules = new ArrayList<>();
        this.outputBlendRules = new ArrayList<>();

        add(DRIVE_SOURCE, true);
        add(AUTO_SOURCE, true);
        add(FEEDBACK_SOURCE, true);
        applyDefaultOutputBlends();
    }

    public void add(String name, boolean enabledByDefault) {
        String key = normalizeKey(name);
        if (key.isBlank()) {
            throw new IllegalArgumentException("speed source name must not be blank");
        }
        SpeedSource source = sources.get(key);
        if (source == null) {
            source = new SpeedSource(name);
            sources.put(key, source);
        }
        source.setEnabled(enabledByDefault);
        if (!enabledByDefault) {
            source.stop();
        }
    }

    public void add(String name) {
        add(name, true);
    }

    public void registerSpeedSource(String name) {
        add(name, true);
    }

    public void registerSpeedSource(String name, boolean enabledByDefault) {
        add(name, enabledByDefault);
    }

    public boolean hasSpeedSource(String name) {
        return sources.containsKey(normalizeKey(name));
    }

    public void blend(String target, String source, BlendMode mode, SpeedAxis axis) {
        blend(target, target, source, mode, axis);
    }

    public void blend(String target, String source, BlendMode mode, SpeedAxis... axes) {
        blend(target, target, source, mode, axes);
    }

    public void blend(String target, String left, String right, BlendMode mode, SpeedAxis axis) {
        blend(target, left, right, mode, new SpeedAxis[] { axis });
    }

    public void blend(String target, String left, String right, BlendMode mode, SpeedAxis... axes) {
        String keyTarget = normalizeKey(target);
        String keyLeft = normalizeKey(left);
        String keyRight = normalizeKey(right);

        speedSource(keyTarget);
        speedSource(keyLeft);
        speedSource(keyRight);

        SourceBlendRule rule = new SourceBlendRule(
                keyTarget,
                keyLeft,
                keyRight,
                mode,
                normalizeAxes(axes));

        sourceBlendRules.add(rule);
        try {
            validateNoCircularBlends();
        } catch (RuntimeException ex) {
            sourceBlendRules.remove(sourceBlendRules.size() - 1);
            throw ex;
        }
    }

    public void blendToOutput(String source, BlendMode mode, SpeedAxis axis) {
        blendToOutput(source, mode, new SpeedAxis[] { axis });
    }

    public void blendToOutput(String source, BlendMode mode, SpeedAxis... axes) {
        String key = normalizeKey(source);
        speedSource(key);
        outputBlendRules.add(new OutputBlendRule(key, mode, normalizeAxes(axes)));
    }

    public void clearSourceBlends() {
        sourceBlendRules.clear();
    }

    public void clearOutputBlends() {
        outputBlendRules.clear();
    }

    public void clearBlends() {
        clearSourceBlends();
        clearOutputBlends();
    }

    public void resetBlendsToDefaults() {
        clearBlends();
        applyDefaultOutputBlends();
    }

    public List<SourceBlendRule> getSourceBlends() {
        return List.copyOf(sourceBlendRules);
    }

    public List<OutputBlendRule> getOutputBlends() {
        return List.copyOf(outputBlendRules);
    }

    // Compatibility helper (legacy interaction API now maps to source-target blend).
    public void addInteraction(String sourceA, String sourceB, BlendMode blendMode, SpeedAxis... axes) {
        blend(sourceA, sourceA, sourceB, blendMode, axes);
    }

    public void setSpeeds(String source, ChassisSpeeds speeds) {
        speedSource(source).setInputSpeeds(speeds);
    }

    public void setSpeeds(String source, double x, double y, double theta) {
        speedSource(source).setInputSpeeds(x, y, theta);
    }

    public ChassisSpeeds getSpeeds(String source) {
        return speedSource(source).getOutputSpeeds();
    }

    public void stop() {
        sources.forEach((key, value) -> value.stop());
    }

    public void stopSpeeds(String source) {
        speedSource(source).stop();
    }

    public void setSpeedSourceState(String source, boolean enabled) {
        SpeedSource speedSource = speedSource(source);
        speedSource.setEnabled(enabled);
        if (!enabled) {
            speedSource.stop();
        }
    }

    public boolean isSpeedsSourceActive(String source) {
        return speedSource(source).isEnabled();
    }

    public void setAxisState(String source, SpeedAxis axis, boolean enabled) {
        speedSource(source).setAxisState(axis, enabled);
    }

    public void setAllAxisState(String source, SpeedAxis axis, boolean enabled) {
        sources.forEach((key, value) -> value.setAxisState(axis, enabled));
    }

    public boolean isAxisActive(String source, SpeedAxis axis) {
        return speedSource(source).isAxisActive(axis);
    }

    public ChassisSpeeds calculate() {
        Map<String, double[]> sourceValues = initializeSourceValues();
        applySourceBlends(sourceValues);

        double[] output = new double[] { 0.0, 0.0, 0.0 };
        applyOutputBlends(sourceValues, output);

        return new ChassisSpeeds(
                clamp(output[axisIndex(SpeedAxis.X)], maxVelocity),
                clamp(output[axisIndex(SpeedAxis.Y)], maxVelocity),
                clamp(output[axisIndex(SpeedAxis.Theta)], maxAngularVelocity));
    }

    private Map<String, double[]> initializeSourceValues() {
        Map<String, double[]> values = new HashMap<>();
        for (Map.Entry<String, SpeedSource> entry : sources.entrySet()) {
            SpeedSource source = entry.getValue();
            values.put(entry.getKey(), new double[] {
                    source.output(SpeedAxis.X),
                    source.output(SpeedAxis.Y),
                    source.output(SpeedAxis.Theta)
            });
        }
        return values;
    }

    private void applySourceBlends(Map<String, double[]> sourceValues) {
        for (SourceBlendRule rule : sourceBlendRules) {
            SpeedSource target = speedSource(rule.target());
            SpeedSource left = speedSource(rule.left());
            SpeedSource right = speedSource(rule.right());

            double[] targetValues = sourceValues.get(rule.target());
            double[] leftValues = sourceValues.get(rule.left());
            double[] rightValues = sourceValues.get(rule.right());

            for (SpeedAxis axis : AXES) {
                if (!rule.appliesToAxis(axis)) {
                    continue;
                }
                if (!target.contributesOnAxis(axis)) {
                    continue;
                }
                if (!left.contributesOnAxis(axis) || !right.contributesOnAxis(axis)) {
                    continue;
                }

                int idx = axisIndex(axis);
                targetValues[idx] = applyBlend(leftValues[idx], rightValues[idx], rule.mode());
            }
        }
    }

    private void applyOutputBlends(Map<String, double[]> sourceValues, double[] output) {
        for (OutputBlendRule rule : outputBlendRules) {
            SpeedSource source = speedSource(rule.source());
            double[] values = sourceValues.get(rule.source());
            for (SpeedAxis axis : AXES) {
                if (!rule.appliesToAxis(axis)) {
                    continue;
                }
                if (!source.contributesOnAxis(axis)) {
                    continue;
                }
                int idx = axisIndex(axis);
                output[idx] = applyBlend(output[idx], values[idx], rule.mode());
            }
        }
    }

    private double applyBlend(double left, double right, BlendMode mode) {
        BlendMode resolved = mode != null ? mode : BlendMode.ADD;
        switch (resolved) {
            case ADD:
                return left + right;
            case SUBTRACT:
                return left - right;
            case MULTIPLY:
                return left * right;
            case DIVIDE:
                if (Math.abs(right) < EPSILON) {
                    return left;
                }
                return left / right;
            case AVERAGE:
                return (left + right) * 0.5;
            case MIN:
                return Math.min(left, right);
            case MAX:
                return Math.max(left, right);
            case A_SUPERSEDES_B:
                return left;
            case B_SUPERSEDES_A:
                return right;
            default:
                return left + right;
        }
    }

    private void validateNoCircularBlends() {
        for (SpeedAxis axis : AXES) {
            Map<String, Set<String>> graph = new HashMap<>();
            for (String source : sources.keySet()) {
                graph.put(source, new HashSet<>());
            }

            for (SourceBlendRule rule : sourceBlendRules) {
                if (!rule.appliesToAxis(axis)) {
                    continue;
                }
                addDependencyEdge(graph, rule.left(), rule.target());
                addDependencyEdge(graph, rule.right(), rule.target());
            }

            detectCycle(graph, axis);
        }
    }

    private void addDependencyEdge(Map<String, Set<String>> graph, String from, String to) {
        if (from.equals(to)) {
            return;
        }
        graph.computeIfAbsent(from, unused -> new HashSet<>()).add(to);
        graph.computeIfAbsent(to, unused -> new HashSet<>());
    }

    private void detectCycle(Map<String, Set<String>> graph, SpeedAxis axis) {
        Set<String> visiting = new HashSet<>();
        Set<String> visited = new HashSet<>();

        for (String node : graph.keySet()) {
            if (visited.contains(node)) {
                continue;
            }
            if (dfsCycle(node, graph, visiting, visited)) {
                throw new IllegalStateException("circular speed blends detected on axis " + axis.name());
            }
        }
    }

    private boolean dfsCycle(
            String node,
            Map<String, Set<String>> graph,
            Set<String> visiting,
            Set<String> visited) {
        visiting.add(node);
        for (String next : graph.getOrDefault(node, Set.of())) {
            if (visited.contains(next)) {
                continue;
            }
            if (visiting.contains(next)) {
                return true;
            }
            if (dfsCycle(next, graph, visiting, visited)) {
                return true;
            }
        }
        visiting.remove(node);
        visited.add(node);
        return false;
    }

    private int axisIndex(SpeedAxis axis) {
        switch (axis) {
            case X:
                return 0;
            case Y:
                return 1;
            case Theta:
                return 2;
            default:
                throw new IllegalArgumentException("invalid axis for value lookup: " + axis);
        }
    }

    private double clamp(double value, double maxValue) {
        return Math.copySign(Math.min(Math.abs(value), maxValue), value);
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    private SpeedSource speedSource(String source) {
        SpeedSource resolved = speedSourceOrNull(source);
        if (resolved == null) {
            throw new IllegalArgumentException("unknown speed source: " + source);
        }
        return resolved;
    }

    private SpeedSource speedSourceOrNull(String source) {
        return sources.get(normalizeKey(source));
    }

    private EnumSet<SpeedAxis> normalizeAxes(SpeedAxis... axes) {
        if (axes == null || axes.length == 0) {
            return EnumSet.of(SpeedAxis.ALL);
        }
        EnumSet<SpeedAxis> resolved = EnumSet.noneOf(SpeedAxis.class);
        for (SpeedAxis axis : axes) {
            if (axis != null) {
                resolved.add(axis);
            }
        }
        if (resolved.isEmpty()) {
            return EnumSet.of(SpeedAxis.ALL);
        }
        return resolved;
    }

    private void applyDefaultOutputBlends() {
        addDefaultOutputBlend(DRIVE_SOURCE);
        addDefaultOutputBlend(AUTO_SOURCE);
        addDefaultOutputBlend(FEEDBACK_SOURCE);
    }

    private void addDefaultOutputBlend(String source) {
        String key = normalizeKey(source);
        if (!sources.containsKey(key)) {
            return;
        }
        outputBlendRules.add(new OutputBlendRule(key, BlendMode.ADD, EnumSet.of(SpeedAxis.ALL)));
    }

    private String normalizeKey(String source) {
        if (source == null) {
            throw new IllegalArgumentException("speed source cannot be null");
        }
        return source.toLowerCase(Locale.ROOT);
    }
}
