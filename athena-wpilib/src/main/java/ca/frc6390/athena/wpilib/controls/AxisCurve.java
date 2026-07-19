package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.interpolation.CurveMapping;

/**
 * Maps one normalized controller-axis value to another.
 *
 * <p>Curve declarations are reusable. Multiple axes may share one curve so live parameter changes
 * update every attached axis, while each axis retains independent sampling and slew state.</p>
 */
@FunctionalInterface
public interface AxisCurve extends CurveMapping { }
