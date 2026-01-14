package ca.frc6390.athena.mechanisms.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.mechanisms.Mechanism;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Declarative description of a mechanism's visualization hierarchy. Each node defines a relative
 * pose from its parent, evaluated at runtime to render both 2D and 3D representations.
 */
public final class MechanismVisualizationConfig {

    /**
     * Represents a visualization node. Each node has a unique name, a parent (root or another node),
     * and a supplier that returns its pose relative to the parent when rendered.
     */
    public record Node(String name,
                       String parent,
                       Function<Mechanism, Pose3d> relativePoseSupplier,
                       Color8Bit color,
                       double lineWeight) {
    }

    private static final double DEFAULT_VECTOR_LINE_LENGTH_METERS = 0.22;
    private static final double DEFAULT_VECTOR_LINE_WEIGHT = 4.0;
    private static final double DEFAULT_VECTOR_LINE_HEADING_OFFSET_RAD = 0.0;
    private static final String VECTOR_LINE_SUFFIX = "VectorLine";
    private static final Color8Bit DEFAULT_VECTOR_LINE_COLOR = new Color8Bit(Color.kOrange);

    private final String rootName;
    private final Function<Mechanism, Pose3d> rootPoseSupplier;
    private final List<Node> nodes;

    private MechanismVisualizationConfig(String rootName,
                                         Function<Mechanism, Pose3d> rootPoseSupplier,
                                         List<Node> nodes) {
        this.rootName = Objects.requireNonNull(rootName);
        this.rootPoseSupplier = Objects.requireNonNull(rootPoseSupplier);
        this.nodes = List.copyOf(nodes);
    }

    public String rootName() {
        return rootName;
    }

    public Function<Mechanism, Pose3d> rootPoseSupplier() {
        return rootPoseSupplier;
    }

    public List<Node> nodes() {
        return nodes;
    }

    public static Builder builder(String rootName) {
        return new Builder(rootName);
    }

    public static final class Builder {
        private final String rootName;
        private Function<Mechanism, Pose3d> rootPoseSupplier = mech -> new Pose3d();
        private final Map<String, Node> nodes = new LinkedHashMap<>();
        private boolean vectorLineEnabled = false;

        private Builder(String rootName) {
            this.rootName = Objects.requireNonNull(rootName);
        }

        /**
         * Supplies a dynamic pose for the root node. Called every loop with the mechanism instance.
         */
        public Builder withRootPose(Function<Mechanism, Pose3d> poseSupplier) {
            this.rootPoseSupplier = Objects.requireNonNull(poseSupplier);
            return this;
        }

        /**
         * Sets a static pose for the root node.
         */
        public Builder withStaticRootPose(Pose3d pose) {
            return withRootPose(mech -> pose);
        }

        /**
         * Adds a visualization node with a dynamic relative pose. The parent must be defined first.
         *
         * @param name unique node name
         * @param parent parent node name or the root name
         * @param relativePoseSupplier pose supplier evaluated relative to the parent
         */
        public Builder addNode(String name,
                               String parent,
                               Function<Mechanism, Pose3d> relativePoseSupplier) {
            return addNode(name, parent, relativePoseSupplier, null, Double.NaN);
        }

        public Builder addNode(String name,
                               String parent,
                               Function<Mechanism, Pose3d> relativePoseSupplier,
                               Color8Bit color) {
            return addNode(name, parent, relativePoseSupplier, color, Double.NaN);
        }

        public Builder addNode(String name,
                               String parent,
                               Function<Mechanism, Pose3d> relativePoseSupplier,
                               Color8Bit color,
                               double lineWeight) {
            Objects.requireNonNull(name);
            Objects.requireNonNull(parent);
            Objects.requireNonNull(relativePoseSupplier);
            if (!parent.equals(rootName) && !nodes.containsKey(parent)) {
                throw new IllegalArgumentException("Parent node " + parent + " must be defined before " + name);
            }
            nodes.put(name, new Node(name, parent, relativePoseSupplier, color, lineWeight));
            return this;
        }

        /**
         * Adds a visualization node with a fixed offset relative to its parent.
         */
        public Builder addStaticNode(String name, String parent, Pose3d relativePose) {
            return addNode(name, parent, mech -> relativePose);
        }

        public Builder addStaticNode(String name,
                                     String parent,
                                     Pose3d relativePose,
                                     Color8Bit color) {
            return addNode(name, parent, mech -> relativePose, color);
        }

        public Builder addStaticNode(String name,
                                     String parent,
                                     Pose3d relativePose,
                                     Color8Bit color,
                                     double lineWeight) {
            return addNode(name, parent, mech -> relativePose, color, lineWeight);
        }

        /**
         * Adds a vector line that points along the mechanism's current rotation.
         */
        public Builder useVectorLine(boolean enabled) {
            this.vectorLineEnabled = enabled;
            return this;
        }

        /**
         * Builds the immutable visualization configuration containing all registered nodes.
         */
        public MechanismVisualizationConfig build() {
            if (vectorLineEnabled) {
                String vectorLineName = rootName + VECTOR_LINE_SUFFIX;
                if (!nodes.containsKey(vectorLineName)) {
                    addNode(
                            vectorLineName,
                            rootName,
                            mech -> vectorLinePose(mech, DEFAULT_VECTOR_LINE_LENGTH_METERS, DEFAULT_VECTOR_LINE_HEADING_OFFSET_RAD),
                            DEFAULT_VECTOR_LINE_COLOR,
                            DEFAULT_VECTOR_LINE_WEIGHT);
                }
            }
            return new MechanismVisualizationConfig(rootName, rootPoseSupplier, new ArrayList<>(nodes.values()));
        }
    }

    private static Pose3d vectorLinePose(Mechanism mechanism, double lengthMeters, double headingOffsetRad) {
        if (mechanism == null || !Double.isFinite(lengthMeters) || lengthMeters <= 0.0) {
            return Pose3d.kZero;
        }
        double angle = toRadians(mechanism, mechanism.getPosition()) + headingOffsetRad;
        if (!Double.isFinite(angle)) {
            return Pose3d.kZero;
        }
        double x = lengthMeters * Math.cos(angle);
        double y = lengthMeters * Math.sin(angle);
        return new Pose3d(x, y, 0.0, new Rotation3d());
    }

    private static double toRadians(Mechanism mechanism, double value) {
        if (mechanism != null && mechanism.getEncoder() != null) {
            double conversion = mechanism.getEncoder().getConversion();
            if (Math.abs(conversion - 360.0) < 1e-3) {
                return Math.toRadians(value);
            }
            if (Math.abs(conversion - (2.0 * Math.PI)) < 1e-3) {
                return value;
            }
        }
        if (Math.abs(value) > 2.0 * Math.PI) {
            return Math.toRadians(value);
        }
        return value;
    }
}
