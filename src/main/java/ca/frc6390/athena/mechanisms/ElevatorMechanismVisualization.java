package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Objects;
import java.util.WeakHashMap;
import java.util.function.Function;

final class ElevatorMechanismVisualization {

    private static final double ROOT_X = 1.5;
    private static final double ROOT_Y = 0.1;

    private static final double POLE_HEIGHT_METERS = 1.8;
    private static final double CARRIAGE_HEIGHT_METERS = 0.36;
    private static final double SETPOINT_HEIGHT_METERS = 0.36;

    private static final double POLE_LINE_WEIGHT = 4.0;
    private static final double CARRIAGE_LINE_WEIGHT = 8.0;
    private static final double SETPOINT_LINE_WEIGHT = 6.0;

    private static final double RENDER_SPAN_METERS = POLE_HEIGHT_METERS;
    private static final WeakHashMap<ElevatorMechanism, State> STATE_MAP = new WeakHashMap<>();

    private ElevatorMechanismVisualization() {
        /* util class */ 
    }

    static MechanismConfig<? extends ElevatorMechanism> prepare(MechanismConfig<? extends ElevatorMechanism> config) {
        Objects.requireNonNull(config);
        if (config.visualizationConfig == null) {
            config.visualizationConfig = createVisualizationConfig();
        }
        return config;
    }

    private static MechanismVisualizationConfig createVisualizationConfig() {
        Function<Mechanism, Pose3d> carriageBottom = mech -> {
            if (!(mech instanceof ElevatorMechanism elevator)) {
                return Pose3d.kZero;
            }
            return carriageBottomPose(elevator);
        };

        Function<Mechanism, Pose3d> setpointBottom = mech -> {
            if (!(mech instanceof ElevatorMechanism elevator)) {
                return Pose3d.kZero;
            }
            return setpointBottomPose(elevator);
        };

        Color8Bit poleColor = new Color8Bit(Color.kGray);
        Color8Bit carriageColor = new Color8Bit(Color.kCyan);
        Color8Bit setpointColor = new Color8Bit(Color.kOrange);

        return MechanismVisualizationConfig.builder("Elevator")
                .withStaticRootPose(new Pose3d(new Translation3d(ROOT_X, ROOT_Y, 0.0), new Rotation3d()))
                .addStaticNode("PoleTop", "Elevator", new Pose3d(new Translation3d(0.0, POLE_HEIGHT_METERS, 0.0), new Rotation3d()), poleColor, POLE_LINE_WEIGHT)
                .addNode("CarriageBottom", "Elevator", carriageBottom, (Color8Bit) null, 0.0)
                .addStaticNode("CarriageTop", "CarriageBottom", new Pose3d(new Translation3d(0.0, CARRIAGE_HEIGHT_METERS, 0.0), new Rotation3d()), carriageColor, CARRIAGE_LINE_WEIGHT)
                .addNode("SetpointBottom", "Elevator", setpointBottom, (Color8Bit) null, 0.0)
                .addStaticNode("SetpointTop", "SetpointBottom", new Pose3d(new Translation3d(0.0, SETPOINT_HEIGHT_METERS, 0.0), new Rotation3d()), setpointColor, SETPOINT_LINE_WEIGHT)
                .build();
    }

    private static Pose3d carriageBottomPose(ElevatorMechanism mechanism) {
        State state = stateFor(mechanism);
        double centerY = state.computeY(mechanism, mechanism.getPosition());
        double bottom = MathUtil.clamp(centerY - (CARRIAGE_HEIGHT_METERS / 2.0), 0.0, POLE_HEIGHT_METERS - CARRIAGE_HEIGHT_METERS);
        return new Pose3d(new Translation3d(0.0, bottom, 0.0), new Rotation3d());
    }

    private static Pose3d setpointBottomPose(ElevatorMechanism mechanism) {
        State state = stateFor(mechanism);
        double centerY = state.computeY(mechanism, mechanism.getSetpoint());
        double bottom = MathUtil.clamp(centerY - (SETPOINT_HEIGHT_METERS / 2.0), 0.0, POLE_HEIGHT_METERS - SETPOINT_HEIGHT_METERS);
        return new Pose3d(new Translation3d(0.0, bottom, 0.0), new Rotation3d());
    }

    private static State stateFor(ElevatorMechanism mechanism) {
        synchronized (STATE_MAP) {
            return STATE_MAP.computeIfAbsent(mechanism, key -> new State());
        }
    }

    private static final class State {
        private static final double DEFAULT_RANGE_GUESS = 1.0;
        private boolean seeded = false;
        private double min = Double.POSITIVE_INFINITY;
        private double max = Double.NEGATIVE_INFINITY;

        double computeY(ElevatorMechanism mechanism, double value) {
            seedIfNeeded(mechanism, value);
            if (!Double.isFinite(value)) {
                return 0.0;
            }
            observe(value);
            double span = max - min;
            if (!(span > 1e-6)) {
                return 0.0;
            }
            double normalized = MathUtil.clamp((value - min) / span, 0.0, 1.0);
            return normalized * RENDER_SPAN_METERS;
        }

        /** Seeds the visualization range using any known travel limits or current values. */
        private void seedIfNeeded(ElevatorMechanism mechanism, double initialValue) {
            if (seeded) {
                return;
            }
            seeded = true;

            double seededMin = Double.POSITIVE_INFINITY;
            double seededMax = Double.NEGATIVE_INFINITY;

            for (GenericLimitSwitch limit : mechanism.getLimitSwitches()) {
                double position = limit.getPosition();
                if (!Double.isFinite(position)) {
                    continue;
                }
                if (position < seededMin) {
                    seededMin = position;
                }
                if (position > seededMax) {
                    seededMax = position;
                }
            }

            if (!(seededMax > seededMin)) {
                double position = mechanism.getPosition();
                double setpoint = mechanism.getSetpoint();
                if (Double.isFinite(position) && Double.isFinite(setpoint) && Math.abs(position - setpoint) > 1e-6) {
                    seededMin = Math.min(position, setpoint);
                    seededMax = Math.max(position, setpoint);
                } else if (Double.isFinite(position)) {
                    seededMin = position;
                    seededMax = position + DEFAULT_RANGE_GUESS;
                } else if (Double.isFinite(setpoint)) {
                    seededMin = setpoint;
                    seededMax = setpoint + DEFAULT_RANGE_GUESS;
                } else if (Double.isFinite(initialValue)) {
                    seededMin = initialValue;
                    seededMax = initialValue + DEFAULT_RANGE_GUESS;
                }
            }

            if (!(seededMax > seededMin)) {
                seededMin = 0.0;
                seededMax = DEFAULT_RANGE_GUESS;
            }

            min = seededMin;
            max = seededMax;
        }

        private void observe(double value) {
            if (!Double.isFinite(value)) {
                return;
            }
            if (value < min) {
                min = value;
            }
            if (value > max) {
                max = value;
            }
        }
    }
}
