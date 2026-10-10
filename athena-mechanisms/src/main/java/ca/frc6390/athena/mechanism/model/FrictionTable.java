package ca.frc6390.athena.mechanism.model;

import java.util.ArrayList;
import java.util.List;

/**
 * Kinetic friction offset in volts as a function of mechanism position.
 *
 * <p>Points are sorted by position. {@link #at(double)} interpolates linearly and holds the end
 * values outside the measured range.</p>
 *
 * @param positions ascending bin centers
 * @param volts kinetic friction offset per bin, in volts
 */
public record FrictionTable(List<Double> positions, List<Double> volts) {
    public static final FrictionTable EMPTY = new FrictionTable(List.of(), List.of());

    public FrictionTable {
        if (positions.size() != volts.size()) {
            throw new IllegalArgumentException("Friction table needs one value per position.");
        }
        List<double[]> pairs = new ArrayList<>(positions.size());
        for (int i = 0; i < positions.size(); i++) {
            pairs.add(new double[] {positions.get(i), volts.get(i)});
        }
        pairs.sort((a, b) -> Double.compare(a[0], b[0]));
        List<Double> sortedPositions = new ArrayList<>(pairs.size());
        List<Double> sortedVolts = new ArrayList<>(pairs.size());
        for (double[] pair : pairs) {
            sortedPositions.add(pair[0]);
            sortedVolts.add(pair[1]);
        }
        positions = List.copyOf(sortedPositions);
        volts = List.copyOf(sortedVolts);
    }

    /** Returns the interpolated friction offset at a position, or 0 for an empty table. */
    public double at(double position) {
        if (positions.isEmpty()) {
            return 0.0;
        }
        if (position <= positions.get(0)) {
            return volts.get(0);
        }
        int last = positions.size() - 1;
        if (position >= positions.get(last)) {
            return volts.get(last);
        }
        for (int i = 1; i <= last; i++) {
            double upper = positions.get(i);
            if (position <= upper) {
                double lower = positions.get(i - 1);
                double span = upper - lower;
                double fraction = span > 0.0 ? (position - lower) / span : 0.0;
                return volts.get(i - 1) + fraction * (volts.get(i) - volts.get(i - 1));
            }
        }
        return volts.get(last);
    }

    public boolean isEmpty() {
        return positions.isEmpty();
    }
}
