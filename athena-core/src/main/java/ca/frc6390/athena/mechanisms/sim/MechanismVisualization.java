package ca.frc6390.athena.mechanisms.sim;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.mechanisms.Mechanism;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Maintains 2D and 3D visualization state for a mechanism. */
public final class MechanismVisualization {

    private final MechanismVisualizationConfig config;
    private final Mechanism2d mechanism2d;
    private final MechanismRoot2d root2d;
    private final Map<String, MechanismLigament2d> ligaments = new HashMap<>();
    private final Map<String, Object> graphObjects = new HashMap<>();
    private final Map<String, Pose3d> poses = new HashMap<>();
    // Preserve absolute heading for each ligament so descendants can stay aligned with their parent.
    private final Map<String, Double> absoluteAngles = new HashMap<>();

    public MechanismVisualization(MechanismVisualizationConfig config) {
        this.config = Objects.requireNonNull(config);
        this.mechanism2d = new Mechanism2d(3.0, 3.0);
        this.root2d = mechanism2d.getRoot(config.rootName(), 0.0, 0.0);

        graphObjects.put(config.rootName(), root2d);

        for (MechanismVisualizationConfig.Node node : config.nodes()) {
            Object parentObject = graphObjects.getOrDefault(node.parent(), root2d);
            MechanismLigament2d ligament;
            if (parentObject instanceof MechanismRoot2d parentRoot) {
                ligament = parentRoot.append(new MechanismLigament2d(node.name(), 0.0, 0.0));
            } else if (parentObject instanceof MechanismLigament2d parentLigament) {
                ligament = parentLigament.append(new MechanismLigament2d(node.name(), 0.0, 0.0));
            } else {
                ligament = root2d.append(new MechanismLigament2d(node.name(), 0.0, 0.0));
            }
            ligaments.put(node.name(), ligament);
            graphObjects.put(node.name(), ligament);
            Color8Bit color = node.color();
            if (color != null) {
                ligament.setColor(color);
            }
            if (!Double.isNaN(node.lineWeight())) {
                ligament.setLineWeight(node.lineWeight());
            }
        }
    }

    public void update(Mechanism mechanism) {
        Pose3d rootPose = config.rootPoseSupplier().apply(mechanism);
        if (rootPose == null) {
            rootPose = new Pose3d();
        }
        root2d.setPosition(rootPose.getX(), rootPose.getY());
        poses.clear();
        poses.put(config.rootName(), rootPose);
        absoluteAngles.clear();
        absoluteAngles.put(config.rootName(), 0.0);

        for (MechanismVisualizationConfig.Node node : config.nodes()) {
            Pose3d parentPose = poses.getOrDefault(node.parent(), rootPose);
            Pose3d relative = node.relativePoseSupplier().apply(mechanism);
            if (relative == null) {
                relative = new Pose3d();
            }
            Pose3d absolute = parentPose.transformBy(new Transform3d(relative.getTranslation(), relative.getRotation()));
            poses.put(node.name(), absolute);

            MechanismLigament2d ligament = ligaments.get(node.name());
            if (ligament != null) {
                double dx = absolute.getX() - parentPose.getX();
                double dy = absolute.getY() - parentPose.getY();
                double length = Math.hypot(dx, dy);
                double angleAbsolute = Math.toDegrees(Math.atan2(dy, dx));
                double parentAngle = absoluteAngles.getOrDefault(node.parent(), 0.0);
                double angleRelative = angleAbsolute - parentAngle;
                ligament.setLength(length);
                ligament.setAngle(angleRelative);
                absoluteAngles.put(node.name(), angleAbsolute);
            }
        }
    }

    public Mechanism2d mechanism2d() {
        return mechanism2d;
    }

    public Map<String, Pose3d> poses() {
        return Map.copyOf(poses);
    }
}
