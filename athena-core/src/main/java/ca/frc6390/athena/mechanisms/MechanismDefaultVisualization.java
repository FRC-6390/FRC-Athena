package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Provides a simple default Mechanism2d visualization for mechanisms that do not supply a custom
 * configuration. The visualization renders a vertical track with bars for the current position and
 * the commanded setpoint, scaled to the mechanism travel range.
 */
final class MechanismDefaultVisualization {

    private MechanismDefaultVisualization() {
        /* utility */
    }

    private static final double ROOT_X = 1.2;
    private static final double ROOT_Y = 0.1;
    private static final double TRACK_HEIGHT_METERS = 1.8;
    private static final double TRACK_WIDTH_METERS = 0.05;
    private static final double POSITION_HEIGHT_METERS = 0.32;
    private static final double SETPOINT_HEIGHT_METERS = 0.24;

    static MechanismVisualizationConfig create() {
        Color8Bit trackColor = new Color8Bit(Color.kLightGray);
        Color8Bit positionColor = new Color8Bit(Color.kBlue);
        Color8Bit setpointColor = new Color8Bit(Color.kOrange);

        return MechanismVisualizationConfig.builder("Mechanism")
                .withStaticRootPose(new Pose3d(new Translation3d(ROOT_X, ROOT_Y, 0.0), new Rotation3d()))
                .addStaticNode("TrackBottom", "Mechanism", new Pose3d(new Translation3d(-TRACK_WIDTH_METERS / 2.0, 0.0, 0.0), new Rotation3d()), trackColor, 4.0)
                .addStaticNode("TrackTop", "TrackBottom", new Pose3d(new Translation3d(0.0, TRACK_HEIGHT_METERS, 0.0), new Rotation3d()), trackColor, 4.0)
                .addNode("PositionBottom", "Mechanism", MechanismDefaultVisualization::positionBottomPose, positionColor)
                .addStaticNode("PositionTop", "PositionBottom", new Pose3d(new Translation3d(0.0, POSITION_HEIGHT_METERS, 0.0), new Rotation3d()), positionColor, 6.0)
                .addNode("SetpointBottom", "Mechanism", MechanismDefaultVisualization::setpointBottomPose, setpointColor)
                .addStaticNode("SetpointTop", "SetpointBottom", new Pose3d(new Translation3d(0.0, SETPOINT_HEIGHT_METERS, 0.0), new Rotation3d()), setpointColor, 5.0)
                .build();
    }

    private static Pose3d positionBottomPose(Mechanism mechanism) {
        double normalized = MechanismTravelRange.normalize(mechanism, mechanism.position());
        double bottom = bottomFor(normalized, POSITION_HEIGHT_METERS);
        return new Pose3d(new Translation3d(0.0, bottom, 0.0), new Rotation3d());
    }

    private static Pose3d setpointBottomPose(Mechanism mechanism) {
        double normalized = MechanismTravelRange.normalize(mechanism, mechanism.setpoint());
        double bottom = bottomFor(normalized, SETPOINT_HEIGHT_METERS);
        return new Pose3d(new Translation3d(0.0, bottom, 0.0), new Rotation3d());
    }

    private static double bottomFor(double normalized, double height) {
        double center = normalized * TRACK_HEIGHT_METERS;
        return MathUtil.clamp(center - (height / 2.0), 0.0, TRACK_HEIGHT_METERS - height);
    }
}
