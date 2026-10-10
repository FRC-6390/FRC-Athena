package ca.frc6390.athena.hardware.sim;

import java.util.OptionalDouble;

/**
 * Plant parameters a simulation model can take from an identified or declared source.
 *
 * <p>{@link SimModel#model(SimDynamicsSource)} applies the values it provides. An empty value leaves
 * the model's own default unchanged.</p>
 */
public interface SimDynamicsSource {
    /** Returns the mechanism-side moment of inertia in base units (kg m^2 for rotary mechanisms). */
    OptionalDouble momentOfInertia();
}
