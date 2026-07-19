package ca.frc6390.athena.mechanism.interpolation;

/** General-purpose curve mappings that do not require WPILib controller declarations. */
public final class CurveMappings {
    /** Identity mapping used by linear interpolation and unshaped inputs. */
    public static final CurveMapping LINEAR = new CurveMapping() {
        @Override public double apply(double input) { return input; }
        @Override public String type() { return "linear"; }
    };

    private CurveMappings() {
    }
}
