package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.MathUtil;
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
    private static final double TURRET_FORWARD_OFFSET_RAD = Math.PI / 2.0;
    private static final boolean TURRET_RING_XZ_PLANE = false;

    static MechanismVisualizationConfig forMechanism(Mechanism mechanism) {
        if (mechanism instanceof ElevatorMechanism) {
            return elevatorVisualization();
        }
        if (mechanism instanceof TurretMechanism) {
            return turretVisualization((TurretMechanism) mechanism);
        }
        if (mechanism instanceof ArmMechanism) {
            return armVisualization();
        }
        if (mechanism instanceof FlywheelMechanism) {
            return flywheelVisualization();
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
                            double angle = toRadians(mech, mech.getPosition());
                            return new Pose3d(
                                    new Translation3d(armLengthMeters, 0.0, 0.0),
                                    new Rotation3d(0.0, 0.0, angle));
                        },
                        ACTUAL_COLOR, DEFAULT_LINE_WEIGHT + 1.0)
                .addNode("ArmSetpoint", "Arm",
                        mech -> {
                            double angle = toRadians(mech, mech.getSetpoint());
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

    private static MechanismVisualizationConfig flywheelVisualization() {
        final double radiusMeters = 0.25;
        Color8Bit ringColor = new Color8Bit(Color.kDimGray);
        Color8Bit markerColor = new Color8Bit(Color.kRed);
        MechanismVisualizationConfig.Builder builder = MechanismVisualizationConfig.builder("Flywheel")
                .withStaticRootPose(new Pose3d(new Translation3d(1.0, 1.1, 0.0), new Rotation3d()));
        addRing(builder, "Flywheel", "FlywheelRing", radiusMeters, 24, ringColor, false);
        builder.addNode("FlywheelMarker", "Flywheel",
                mech -> {
                    double angle = wrapRadians(toRadians(mech, mech.getPosition()));
                    double x = radiusMeters * Math.cos(angle);
                    double y = radiusMeters * Math.sin(angle);
                    return new Pose3d(new Translation3d(x, y, 0.0), new Rotation3d());
                },
                markerColor, DEFAULT_LINE_WEIGHT);
        return builder.build();
    }

    private static MechanismVisualizationConfig turretVisualization(TurretMechanism turret) {
        final double radiusMeters = 0.22;
        Color8Bit ringColor = new Color8Bit(Color.kDarkGray);
        Color8Bit markerColor = new Color8Bit(Color.kOrange);
        MechanismVisualizationConfig.Builder builder = MechanismVisualizationConfig.builder("Turret")
                .withStaticRootPose(new Pose3d(new Translation3d(1.0, 0.8, 0.0), new Rotation3d(0,0,0)));
        addRing(builder, "Turret", "TurretRing", radiusMeters, 32, ringColor, TURRET_RING_XZ_PLANE);
        builder.addNode("TurretForward", "Turret",
                mech -> {
                    double angle = wrapRadians(toRadians(turret, turret.getPosition()) + TURRET_FORWARD_OFFSET_RAD);
                    double x = radiusMeters * Math.cos(angle);
                    double y = radiusMeters * Math.sin(angle);
                    return new Pose3d(new Translation3d(x, y, 0.0), new Rotation3d(0,0,0));
                },
                markerColor, DEFAULT_LINE_WEIGHT + 1.0);
        return builder.build();
    }

    private static double wrapRadians(double radians) {
        return MathUtil.inputModulus(radians, 0.0, 2.0 * Math.PI);
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

    private static void addRing(MechanismVisualizationConfig.Builder builder,
                                String root,
                                String prefix,
                                double radius,
                                int segments,
                                Color8Bit color,
                                boolean useXZPlane) {
        double angleStep = (2.0 * Math.PI) / segments;
        double startX = radius;
        double startY = 0.0;
        String prev = prefix + "Start";
        builder.addStaticNode(prev, root,
                new Pose3d(new Translation3d(startX, useXZPlane ? 0.0 : startY, useXZPlane ? startY : 0.0), new Rotation3d()),
                color, 1.5);
        double prevX = startX;
        double prevY = startY;
        for (int i = 1; i <= segments; i++) {
            double angle = angleStep * i;
            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);
            double dx = x - prevX;
            double dy = y - prevY;
            String name = prefix + i;
            builder.addStaticNode(name, prev,
                    new Pose3d(new Translation3d(dx, useXZPlane ? 0.0 : dy, useXZPlane ? dy : 0.0), new Rotation3d()),
                    color, 1.5);
            prev = name;
            prevX = x;
            prevY = y;
        }
        double closeDx = startX - prevX;
        double closeDy = startY - prevY;
        builder.addStaticNode(prefix + "Close", prev,
                new Pose3d(new Translation3d(closeDx, useXZPlane ? 0.0 : closeDy, useXZPlane ? closeDy : 0.0), new Rotation3d()),
                color, 1.5);
    }
}
