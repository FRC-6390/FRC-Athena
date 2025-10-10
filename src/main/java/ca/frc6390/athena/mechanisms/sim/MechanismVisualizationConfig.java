package ca.frc6390.athena.mechanisms.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

import ca.frc6390.athena.mechanisms.Mechanism;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Declarative description of a mechanism's visualization hierarchy. Each node defines a relative
 * pose from its parent, evaluated at runtime to render both 2D and 3D representations.
 */
public final class MechanismVisualizationConfig {

    public record Node(String name,
                       String parent,
                       Function<Mechanism, Pose3d> relativePoseSupplier) {
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

        public Builder withRootPose(Function<Mechanism, Pose3d> poseSupplier) {
            this.rootPoseSupplier = Objects.requireNonNull(poseSupplier);
            return this;
        }

        public Builder withStaticRootPose(Pose3d pose) {
            return withRootPose(mech -> pose);
        }

        public Builder addNode(String name,
                               String parent,
                               Function<Mechanism, Pose3d> relativePoseSupplier) {
            Objects.requireNonNull(name);
            Objects.requireNonNull(parent);
            Objects.requireNonNull(relativePoseSupplier);
            if (!parent.equals(rootName) && !nodes.containsKey(parent)) {
                throw new IllegalArgumentException("Parent node " + parent + " must be defined before " + name);
            }
            nodes.put(name, new Node(name, parent, relativePoseSupplier));
            return this;
        }

        public Builder addStaticNode(String name, String parent, Pose3d relativePose) {
            return addNode(name, parent, mech -> relativePose);
        }

        public MechanismVisualizationConfig build() {
            return new MechanismVisualizationConfig(rootName, rootPoseSupplier, new ArrayList<>(nodes.values()));
        }
    }
}
