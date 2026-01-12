package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism.StatefulMechanismCore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Simple motor-based turret mechanism that supports continuous rotation inputs.
 * Wraps {@link SimpleMotorMechanism} with Rotation2d helpers and a stateful variant.
 */
public class TurretMechanism extends SimpleMotorMechanism {

    private Rotation2d minRotation = Rotation2d.fromRadians(Double.NEGATIVE_INFINITY);
    private Rotation2d maxRotation = Rotation2d.fromRadians(Double.POSITIVE_INFINITY);
    private boolean enforceBounds = false;

    public TurretMechanism(MechanismConfig<? extends TurretMechanism> config, SimpleMotorFeedforward feedforward) {
        super(config, feedforward);
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
        return this;
    }

    /** Removes any configured rotation bounds. */
    public TurretMechanism clearRotationBounds() {
        enforceBounds = false;
        minRotation = Rotation2d.fromRadians(Double.NEGATIVE_INFINITY);
        maxRotation = Rotation2d.fromRadians(Double.POSITIVE_INFINITY);
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

    public static class StatefulTurretMechanism<E extends Enum<E> & SetpointProvider<Double>> extends TurretMechanism {

        private final StatefulMechanismCore<StatefulTurretMechanism<E>, E> stateCore;

        public StatefulTurretMechanism(MechanismConfig<StatefulTurretMechanism<E>> config, SimpleMotorFeedforward feedforward, E initialState) {
            super(config, feedforward);
            stateCore = new StatefulMechanismCore<>(initialState, this::atSetpoint, config.stateMachineDelay, config.stateActions);
        }

        @Override
        public double getSetpoint() {
            return stateCore.getSetpoint();
        }

        @Override
        public void update() {
            setSuppressMotorOutput(!stateCore.update(this));
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
