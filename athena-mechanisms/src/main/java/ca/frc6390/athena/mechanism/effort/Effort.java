package ca.frc6390.athena.mechanism.effort;

import edu.wpi.first.units.Measure;
import java.util.List;
import java.util.Objects;

/**
 * Effort ceiling for a control binding (B5): the largest torque, force or current the binding may
 * drive, with optional load responses.
 *
 * <pre>{@code
 * public final ControlBinding<?> angle = Controls.position(motor, Units.Degrees)
 *         .feedback(encoder)
 *         .model(characterization)
 *         .effort(Effort.max(Units.NewtonMeters.of(12))
 *                 .gearing(25.0)                       // motor turns per mechanism turn
 *                 .onLoad(LoadResponse.YIELD.recoverAfter(Units.Seconds.of(0.5))));
 * }</pre>
 *
 * <p>Torque and force need known gearing ({@link #gearing(double)}, and {@link #radius(Measure)} for
 * linear forces), plus the motor's constants. Current needs neither. Units are checked when the
 * binding is declared.</p>
 */
public final class Effort {
    private final Measure<?> ceiling;
    private final double motorTurnsPerMechanismTurn;
    private final Measure<?> radius;
    private final List<LoadResponse> responses;

    private Effort(Measure<?> ceiling, double motorTurnsPerMechanismTurn, Measure<?> radius,
            List<LoadResponse> responses) {
        this.ceiling = ceiling;
        this.motorTurnsPerMechanismTurn = motorTurnsPerMechanismTurn;
        this.radius = radius;
        this.responses = responses;
    }

    /**
     * Starts an effort ceiling.
     *
     * @param ceiling maximum torque, force or current; the kind must suit the binding
     * @return declaration
     */
    public static Effort max(Measure<?> ceiling) {
        return new Effort(Objects.requireNonNull(ceiling, "ceiling"), Double.NaN, null, List.of());
    }

    /**
     * Declares the motor-to-mechanism gearing used to convert torque or force into motor current.
     *
     * @param motorTurnsPerMechanismTurn motor rotations per mechanism rotation (10 for a 10:1 reduction)
     * @return declaration
     */
    public Effort gearing(double motorTurnsPerMechanismTurn) {
        if (!Double.isFinite(motorTurnsPerMechanismTurn) || motorTurnsPerMechanismTurn <= 0.0) {
            throw new IllegalArgumentException("Effort gearing must be positive and finite.");
        }
        return new Effort(ceiling, motorTurnsPerMechanismTurn, radius, responses);
    }

    /**
     * Declares the drum or pulley radius of a linear mechanism, so force converts to torque at the motor.
     *
     * @param radius radius; a distance measure
     * @return declaration
     */
    public Effort radius(Measure<?> radius) {
        return new Effort(ceiling, motorTurnsPerMechanismTurn, Objects.requireNonNull(radius, "radius"), responses);
    }

    /**
     * Declares what happens while the external load is at or above the ceiling. Replaces earlier
     * responses. Needs a model on the binding; without one the responses are ignored and only the
     * ceiling applies.
     *
     * @param responses load responses, at most one of each kind
     * @return declaration
     */
    public Effort onLoad(LoadResponse... responses) {
        Objects.requireNonNull(responses, "responses");
        List<LoadResponse> copy = List.of(responses.clone());
        for (LoadResponse response : copy) {
            Objects.requireNonNull(response, "response");
        }
        return new Effort(ceiling, motorTurnsPerMechanismTurn, radius, copy);
    }

    /** Returns the declared ceiling measure. */
    public Measure<?> ceiling() {
        return ceiling;
    }

    /** Returns the declared gearing, or NaN when unknown. */
    public double gearing() {
        return motorTurnsPerMechanismTurn;
    }

    /** Returns the declared linear radius, or null when unknown. */
    public Measure<?> radius() {
        return radius;
    }

    /** Returns the declared load responses. */
    public List<LoadResponse> responses() {
        return responses;
    }
}
