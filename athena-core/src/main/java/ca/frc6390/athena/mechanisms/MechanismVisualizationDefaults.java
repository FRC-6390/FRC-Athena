package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Provides simple, mechanism-aware default visualizations so Mechanism2d reflects the expected
 * motion for each mechanism type without requiring custom configs.
 */
public final class MechanismVisualizationDefaults {

    private MechanismVisualizationDefaults() {
        /* utility */
    }

    private static final Color8Bit ACTUAL_COLOR = new Color8Bit(Color.kDodgerBlue);
    private static final Color8Bit SETPOINT_COLOR = new Color8Bit(Color.kOrange);
    private static final double DEFAULT_LINE_WEIGHT = 4.0;

    static MechanismVisualizationConfig forMechanism(Mechanism mechanism) {
        if (mechanism instanceof ElevatorMechanism) {
            return elevatorVisualization();
        }
        if (mechanism instanceof ArmMechanism) {
            return armVisualization();
        }
        if (mechanism instanceof SimpleMotorMechanism) {
            return simpleMotorVisualization();
        }
        return MechanismDefaultVisualization.create();
    }

    private static MechanismVisualizationConfig elevatorVisualization() {
        final double trackHeightMeters = 2.0;

        return MechanismVisualizationConfig.builder("Elevator")
                .withStaticRootPose(new Pose3d(new Translation3d(0.3, 0.1, 0.0), new Rotation3d()))
                .addStaticNode("TrackTop", "Elevator",
                        new Pose3d(new Translation3d(0.0, trackHeightMeters, 0.0), new Rotation3d()),
                        new Color8Bit(Color.kLightGray), DEFAULT_LINE_WEIGHT)
                .addNode("Carriage", "Elevator",
                        mech -> {
                            double normalized = MechanismTravelRange.normalize(mech, mech.getPosition());
                            double height = MathUtil.clamp(normalized, 0.0, 1.0) * trackHeightMeters;
                            return new Pose3d(new Translation3d(0.0, height, 0.0), new Rotation3d());
                        },
                        ACTUAL_COLOR, DEFAULT_LINE_WEIGHT)
                .addNode("CarriageSetpoint", "Elevator",
                        mech -> {
                            double normalized = MechanismTravelRange.normalize(mech, mech.getSetpoint());
                            double height = MathUtil.clamp(normalized, 0.0, 1.0) * trackHeightMeters;
                            return new Pose3d(new Translation3d(0.0, height, 0.0), new Rotation3d());
                        },
                        SETPOINT_COLOR, DEFAULT_LINE_WEIGHT - 1.0)
                .build();
    }

    private static MechanismVisualizationConfig armVisualization() {
        final double armLengthMeters = 0.7;

        return MechanismVisualizationConfig.builder("Arm")
                .withStaticRootPose(new Pose3d(new Translation3d(1.0, 0.4, 0.0), new Rotation3d()))
                .addNode("ArmActual", "Arm",
                        mech -> {
                            double angle = mech.getPosition(); // radians
                            return new Pose3d(
                                    new Translation3d(armLengthMeters, 0.0, 0.0),
                                    new Rotation3d(0.0, 0.0, angle));
                        },
                        ACTUAL_COLOR, DEFAULT_LINE_WEIGHT + 1.0)
                .addNode("ArmSetpoint", "Arm",
                        mech -> {
                            double angle = mech.getSetpoint();
                            return new Pose3d(
                                    new Translation3d(armLengthMeters, 0.0, 0.0),
                                    new Rotation3d(0.0, 0.0, angle));
                        },
                        SETPOINT_COLOR, DEFAULT_LINE_WEIGHT)
                .build();
    }

    private static MechanismVisualizationConfig simpleMotorVisualization() {
        final double radiusMeters = 0.45;

        return MechanismVisualizationConfig.builder("Rotor")
                .withStaticRootPose(new Pose3d(new Translation3d(1.0, 1.0, 0.0), new Rotation3d()))
                .addNode("RotorActual", "Rotor",
                        mech -> {
                            double angle = mech.getPosition();
                            return new Pose3d(
                                    new Translation3d(radiusMeters, 0.0, 0.0),
                                    new Rotation3d(0.0, 0.0, angle));
                        },
                        ACTUAL_COLOR, DEFAULT_LINE_WEIGHT + 1.0)
                .addNode("RotorSetpoint", "Rotor",
                        mech -> {
                            double angle = mech.getSetpoint();
                            return new Pose3d(
                                    new Translation3d(radiusMeters, 0.0, 0.0),
                                    new Rotation3d(0.0, 0.0, angle));
                        },
                        SETPOINT_COLOR, DEFAULT_LINE_WEIGHT)
                .build();
    }
}
