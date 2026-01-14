package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Simple motor-based turret mechanism that supports continuous rotation inputs.
 * Wraps {@link SimpleMotorMechanism} with Rotation2d helpers and a stateful variant.
 */
public class TurretMechanism extends SimpleMotorMechanism {

    public static final class FieldHeadingVisualization {
        private final Supplier<FieldObject2d> objectSupplier;
        private final Supplier<Pose2d> poseSupplier;
        private final DoubleSupplier lengthSupplier;
        private final Rotation2d headingOffset;

        public FieldHeadingVisualization(Supplier<FieldObject2d> objectSupplier,
                                         Supplier<Pose2d> poseSupplier,
                                         DoubleSupplier lengthSupplier,
                                         Rotation2d headingOffset) {
            this.objectSupplier = Objects.requireNonNull(objectSupplier, "objectSupplier");
            this.poseSupplier = Objects.requireNonNull(poseSupplier, "poseSupplier");
            this.lengthSupplier = Objects.requireNonNull(lengthSupplier, "lengthSupplier");
            this.headingOffset = headingOffset != null ? headingOffset : Rotation2d.kZero;
        }

        public static FieldHeadingVisualization fromField(Supplier<Field2d> fieldSupplier,
                                                          String objectName,
                                                          Supplier<Pose2d> poseSupplier,
                                                          DoubleSupplier lengthSupplier,
                                                          Rotation2d headingOffset) {
            Objects.requireNonNull(fieldSupplier, "fieldSupplier");
            Objects.requireNonNull(objectName, "objectName");
            return new FieldHeadingVisualization(
                    () -> {
                        Field2d field = fieldSupplier.get();
                        return field != null ? field.getObject(objectName) : null;
                    },
                    poseSupplier,
                    lengthSupplier,
                    headingOffset);
        }
    }

    private Rotation2d minRotation = Rotation2d.fromRadians(Double.NEGATIVE_INFINITY);
    private Rotation2d maxRotation = Rotation2d.fromRadians(Double.POSITIVE_INFINITY);
    private boolean enforceBounds = false;
    private Supplier<FieldHeadingVisualization> fieldHeadingVisualizationSupplier;
    private Supplier<Pose2d> fieldPoseSupplier;
    private DoubleSupplier fieldHeadingLengthSupplier;
    private FieldObject2d fieldHeadingObject;
    private Rotation2d fieldHeadingOffset = Rotation2d.kZero;

    public TurretMechanism(MechanismConfig<? extends TurretMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config, feedforward);
        if (config != null && config.turretHeadingVisualization != null) {
            setFieldHeadingVisualization(config.turretHeadingVisualization);
        }
    }

    /** Current turret heading. */
    public Rotation2d getRotation() {
        return getRotation2d();
    }

    /** Convenience for retrieving the current heading in radians. */
    public double getRotationRadians() {
        return getRotation().getRadians();
    }

    /** Sets the turret setpoint in radians. */
    public void setRotationRadians(double radians) {
        setSetpoint(applyBounds(radians));
    }

    /**
     * Sets the turret setpoint using {@link Rotation2d}.
     *
     * @param rotation desired absolute heading
     */
    public void setRotation(Rotation2d rotation) {
        setRotation(rotation, false);
    }

    /**
     * Sets the turret setpoint using {@link Rotation2d} and optionally snaps to the closest
     * equivalent angle to reduce travel.
     *
     * @param rotation desired heading
     * @param minimizeMotion when true, wraps the target to the nearest equivalent angle
     */
    public void setRotation(Rotation2d rotation, boolean minimizeMotion) {
        double target = rotation.getRadians();
        if (minimizeMotion) {
            double current = getRotationRadians();
            target = current + MathUtil.angleModulus(target - current);
        }
        setRotationRadians(target);
    }

    /** Nudges the turret relative to its current setpoint. */
    public void incrementRotation(Rotation2d delta) {
        setRotationRadians(getSetpoint() + delta.getRadians());
    }

    /**
     * Computes the turret heading needed to face a field position and applies it.
     *
     * @param robotPose current robot pose on the field
     * @param target field-space position to face
     * @param headingOffset additional robot-relative offset to apply (e.g., camera offset)
     * @param minimizeMotion when true, snaps to the nearest equivalent heading to avoid long spins
     */
    public void faceFieldPosition(Pose2d robotPose, Translation2d target, Rotation2d headingOffset, boolean minimizeMotion) {
        Rotation2d targetHeading = getFacingRotation(robotPose, target, headingOffset);
        setRotation(targetHeading, minimizeMotion);
    }

    /** Faces a field position with no additional offset. */
    public void faceFieldPosition(Pose2d robotPose, Translation2d target, boolean minimizeMotion) {
        faceFieldPosition(robotPose, target, Rotation2d.kZero, minimizeMotion);
    }

    /**
     * Computes the turret heading needed to face a field position without changing the setpoint.
     *
     * @param robotPose current robot pose on the field
     * @param target field-space position to face
     * @param headingOffset additional robot-relative offset to apply (e.g., camera offset)
     * @return robot-relative heading to face the target
     */
    public Rotation2d getFacingRotation(Pose2d robotPose, Translation2d target, Rotation2d headingOffset) {
        Translation2d delta = target.minus(robotPose.getTranslation());
        return delta.getAngle().minus(robotPose.getRotation()).plus(headingOffset);
    }

    /**
     * Publishes a field-space line showing the turret heading (for AdvantageScope Field2d).
     */
    public TurretMechanism setFieldHeadingVisualization(Field2d field,
                                                        String objectName,
                                                        Supplier<Pose2d> poseSupplier,
                                                        DoubleSupplier lengthSupplier) {
        Objects.requireNonNull(field, "field");
        Objects.requireNonNull(objectName, "objectName");
        return setFieldHeadingVisualization(
                new FieldHeadingVisualization(() -> field.getObject(objectName), poseSupplier, lengthSupplier, Rotation2d.kZero));
    }

    /**
     * Publishes a field-space line showing the turret heading (for AdvantageScope Field2d).
     */
    public TurretMechanism setFieldHeadingVisualization(Field2d field,
                                                        String objectName,
                                                        Supplier<Pose2d> poseSupplier,
                                                        DoubleSupplier lengthSupplier,
                                                        Rotation2d headingOffset) {
        Objects.requireNonNull(field, "field");
        Objects.requireNonNull(objectName, "objectName");
        return setFieldHeadingVisualization(
                new FieldHeadingVisualization(() -> field.getObject(objectName), poseSupplier, lengthSupplier, headingOffset));
    }

    /**
     * Publishes a field-space line showing the turret heading (for AdvantageScope Field2d).
     */
    public TurretMechanism setFieldHeadingVisualization(FieldObject2d object,
                                                        Supplier<Pose2d> poseSupplier,
                                                        DoubleSupplier lengthSupplier,
                                                        Rotation2d headingOffset) {
        return setFieldHeadingVisualization(
                new FieldHeadingVisualization(() -> Objects.requireNonNull(object, "object"),
                        poseSupplier,
                        lengthSupplier,
                        headingOffset));
    }

    /**
     * Publishes a field-space line showing the turret heading (for AdvantageScope Field2d).
     */
    public TurretMechanism setFieldHeadingVisualization(FieldHeadingVisualization visualization) {
        Objects.requireNonNull(visualization, "visualization");
        this.fieldHeadingVisualizationSupplier = () -> visualization;
        applyFieldHeadingVisualization(visualization);
        return this;
    }

    /**
     * Publishes a field-space line showing the turret heading (for AdvantageScope Field2d).
     */
    public TurretMechanism setFieldHeadingVisualization(Supplier<FieldHeadingVisualization> visualizationSupplier) {
        this.fieldHeadingVisualizationSupplier = Objects.requireNonNull(visualizationSupplier, "visualizationSupplier");
        FieldHeadingVisualization visualization = visualizationSupplier.get();
        if (visualization != null) {
            applyFieldHeadingVisualization(visualization);
        }
        return this;
    }

    /**
     * Clears any field-space turret heading visualization.
     */
    public TurretMechanism clearFieldHeadingVisualization() {
        fieldHeadingObject = null;
        fieldHeadingVisualizationSupplier = null;
        fieldPoseSupplier = null;
        fieldHeadingLengthSupplier = null;
        fieldHeadingOffset = Rotation2d.kZero;
        return this;
    }

    /** Restricts turret travel to a bounded window to protect wiring. */
    public TurretMechanism setRotationBounds(Rotation2d min, Rotation2d max) {
        double minRad = min.getRadians();
        double maxRad = max.getRadians();
        if (maxRad <= minRad) {
            throw new IllegalArgumentException("maxRotation must be greater than minRotation");
        }
        this.minRotation = min;
        this.maxRotation = max;
        this.enforceBounds = true;
        setBounds(minRad, maxRad);
        return this;
    }

    /** Removes any configured rotation bounds. */
    public TurretMechanism clearRotationBounds() {
        enforceBounds = false;
        minRotation = Rotation2d.fromRadians(Double.NEGATIVE_INFINITY);
        maxRotation = Rotation2d.fromRadians(Double.POSITIVE_INFINITY);
        clearBounds();
        return this;
    }

    private double applyBounds(double targetRadians) {
        if (!enforceBounds) {
            return targetRadians;
        }

        double min = minRotation.getRadians();
        double max = maxRotation.getRadians();
        double span = max - min;
        if (!(span > 0)) {
            return targetRadians;
        }

        double current = getRotationRadians();
        double twoPi = 2.0 * Math.PI;
        long kMin = (long) Math.ceil((min - targetRadians) / twoPi);
        long kMax = (long) Math.floor((max - targetRadians) / twoPi);

        double best = MathUtil.clamp(targetRadians, min, max);
        double bestError = Math.abs(best - current);

        if (kMin <= kMax) {
            for (long k = kMin; k <= kMax; k++) {
                double candidate = targetRadians + k * twoPi;
                double error = Math.abs(candidate - current);
                if (error < bestError) {
                    bestError = error;
                    best = candidate;
                }
            }
        }

        return MathUtil.clamp(best, min, max);
    }

    @Override
    public TurretMechanism shuffleboard(String tab, SendableLevel level) {
        return (TurretMechanism) super.shuffleboard(tab, level);
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        return super.shuffleboard(tab, level);
    }

    @Override
    public void periodic() {
        super.periodic();
        updateFieldHeadingVisualization();
    }

    private void updateFieldHeadingVisualization() {
        if (fieldHeadingObject == null && fieldHeadingVisualizationSupplier != null) {
            FieldHeadingVisualization visualization = fieldHeadingVisualizationSupplier.get();
            if (visualization != null) {
                applyFieldHeadingVisualization(visualization);
            }
        }
        if (fieldHeadingObject == null || fieldPoseSupplier == null || fieldHeadingLengthSupplier == null) {
            return;
        }
        Pose2d basePose = fieldPoseSupplier.get();
        if (basePose == null) {
            return;
        }
        double length = fieldHeadingLengthSupplier.getAsDouble();
        if (!Double.isFinite(length) || length <= 0.0) {
            return;
        }
        Rotation2d heading = basePose.getRotation().plus(getRotation()).plus(fieldHeadingOffset);
        Translation2d start = basePose.getTranslation();
        Translation2d end = start.plus(new Translation2d(length, heading));
        fieldHeadingObject.setPoses(new Pose2d(start, heading), new Pose2d(end, heading));
    }

    private void applyFieldHeadingVisualization(FieldHeadingVisualization visualization) {
        FieldObject2d object = visualization.objectSupplier.get();
        if (object == null) {
            return;
        }
        this.fieldHeadingObject = object;
        this.fieldPoseSupplier = visualization.poseSupplier;
        this.fieldHeadingLengthSupplier = visualization.lengthSupplier;
        this.fieldHeadingOffset = visualization.headingOffset;
    }

    public static class StatefulTurretMechanism<E extends Enum<E> & SetpointProvider<Double>> extends TurretMechanism implements StatefulLike<E> {

        private final StatefulMechanismCore<StatefulTurretMechanism<E>, E> stateCore;

        public StatefulTurretMechanism(MechanismConfig<StatefulTurretMechanism<E>> config, SimpleMotorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay,
                    config.stateActions, config.stateHooks, config.alwaysHooks, config.inputs, config.doubleInputs, config.objectInputs);
        }

        @Override
        public double getSetpoint() {
            return stateCore.getSetpoint();
        }

        public void setSetpointOverride(DoubleSupplier override) {
            stateCore.setSetpointOverride(override);
        }

        public void clearSetpointOverride() {
            stateCore.clearSetpointOverride();
        }

        public void setOutputSuppressor(BooleanSupplier suppressor) {
            stateCore.setOutputSuppressor(suppressor);
        }

        public void clearOutputSuppressor() {
            stateCore.clearOutputSuppressor();
        }

        @Override
        public void update() {
            setSuppressMotorOutput(stateCore.update(this));
            super.update();
        }

        public StateMachine<Double, E> getStateMachine() {
            return stateCore.getStateMachine();
        }

        public void setStateGraph(StateGraph<E> stateGraph) {
            stateCore.setStateGraph(stateGraph);
        }

        @Override
        public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
            stateCore.shuffleboard(tab, level);
            return super.shuffleboard(tab, level);
        }

        @Override
        @SuppressWarnings("unchecked")
        public StatefulTurretMechanism<E> shuffleboard(String tab, SendableLevel level) {
            return (StatefulTurretMechanism<E>) super.shuffleboard(tab, level);
        }
    }
}
