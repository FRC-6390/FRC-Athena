package ca.frc6390.athena.mechanism.interpolation;

/** Built-in interpolation models. */
public final class InterpolationKinds {
    /** Clamped piecewise-linear interpolation. */
    public static final InterpolationModel LINEAR = (input, data) -> {
        int lower = data.lowerIndex(input);
        int upper = data.upperIndex(input);
        if (lower == upper) {
            return data.value(lower);
        }
        double lowerPoint = data.point(lower);
        double amount = (input - lowerPoint) / (data.point(upper) - lowerPoint);
        return data.value(lower) + amount * (data.value(upper) - data.value(lower));
    };

    private InterpolationKinds() {
    }
}
