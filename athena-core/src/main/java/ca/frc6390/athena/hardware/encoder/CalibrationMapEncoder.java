package ca.frc6390.athena.hardware.encoder;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Applies a monotonic piecewise-linear calibration map to one encoder signal.
 */
public final class CalibrationMapEncoder extends AbstractDerivedEncoder {
    public record Point(double input, double output) {
    }

    private final DerivedEncoderInput input;
    private final List<Point> points;

    public CalibrationMapEncoder(
            EncoderConfig config,
            DerivedEncoderInput input,
            List<Point> points) {
        super(config, OutputKind.POSITION, input.signal() == EncoderSignalType.ABSOLUTE);
        this.input = input;
        this.points = sanitizePoints(points);
    }

    @Override
    public Encoder update() {
        double value = input.read(true);
        cachedValue = interpolate(value);
        return this;
    }

    private double interpolate(double value) {
        if (points.isEmpty()) {
            return 0.0;
        }
        if (value <= points.get(0).input()) {
            return points.get(0).output();
        }
        int last = points.size() - 1;
        if (value >= points.get(last).input()) {
            return points.get(last).output();
        }
        for (int i = 1; i < points.size(); i++) {
            Point lower = points.get(i - 1);
            Point upper = points.get(i);
            if (value > upper.input()) {
                continue;
            }
            double span = upper.input() - lower.input();
            if (!Double.isFinite(span) || span == 0.0) {
                return lower.output();
            }
            double t = (value - lower.input()) / span;
            return lower.output() + (upper.output() - lower.output()) * t;
        }
        return points.get(last).output();
    }

    private static List<Point> sanitizePoints(List<Point> raw) {
        List<Point> sorted = new ArrayList<>(raw != null ? raw : List.of());
        sorted.sort(Comparator.comparingDouble(Point::input));
        double previous = Double.NEGATIVE_INFINITY;
        for (Point point : sorted) {
            if (point == null || !Double.isFinite(point.input()) || !Double.isFinite(point.output())) {
                throw new IllegalArgumentException("calibration map points must be finite");
            }
            if (point.input() <= previous) {
                throw new IllegalArgumentException("calibration map points must be strictly increasing");
            }
            previous = point.input();
        }
        return List.copyOf(sorted);
    }
}
