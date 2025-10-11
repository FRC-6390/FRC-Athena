package ca.frc6390.athena.mechanisms.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.mechanisms.Mechanism;
import edu.wpi.first.math.geometry.Pose3d;
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
         * Builds the immutable visualization configuration containing all registered nodes.
         */
        public MechanismVisualizationConfig build() {
            return new MechanismVisualizationConfig(rootName, rootPoseSupplier, new ArrayList<>(nodes.values()));
        }
    }
}
