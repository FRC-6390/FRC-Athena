package ca.frc6390.athena.mechanism.model;

/**
 * Ordinary least squares for small regressions, solved on column-equilibrated normal equations.
 *
 * <p>For rows {@code x_i} and targets {@code y_i} the coefficients are
 * {@code beta = (X'X)^-1 X'y}. The residual variance is {@code s^2 = RSS / (n - p)}, and the standard
 * error of coefficient {@code j} is {@code sqrt(s^2 * [(X'X)^-1]_jj)}. The condition number is the
 * 1-norm condition of the equilibrated Gram matrix, so it is independent of the units of each column.</p>
 */
public final class LeastSquares {
    private static final double SINGULAR_PIVOT = 1.0e-12;

    /**
     * Result of one regression.
     *
     * @param coefficients fitted coefficients, one per column
     * @param standardErrors standard error per coefficient, NaN when there are no spare degrees of freedom
     * @param rSquared coefficient of determination, 0 when the target has no variance
     * @param conditionNumber 1-norm condition number of the equilibrated Gram matrix
     * @param samples number of rows
     */
    public record Fit(
            double[] coefficients,
            double[] standardErrors,
            double rSquared,
            double conditionNumber,
            int samples) {
        public Fit {
            coefficients = coefficients.clone();
            standardErrors = standardErrors.clone();
        }

        @Override
        public double[] coefficients() {
            return coefficients.clone();
        }

        @Override
        public double[] standardErrors() {
            return standardErrors.clone();
        }
    }

    private LeastSquares() {
    }

    /**
     * Fits {@code y = X beta}.
     *
     * @param x rows of regressors, all of the same length
     * @param y targets, one per row
     * @return the fit
     * @throws IllegalArgumentException when there are fewer rows than columns, or a column is
     *     constant or collinear with the others
     */
    public static Fit fit(double[][] x, double[] y) {
        int n = requireRows(x, y);
        int p = x[0].length;
        if (n <= p) {
            throw new IllegalArgumentException("Least squares needs more samples (" + n
                    + ") than parameters (" + p + ").");
        }
        double[][] gram = gram(x);
        double[] xty = new double[p];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < p; j++) {
                xty[j] += x[i][j] * y[i];
            }
        }
        double[] scale = scale(gram);
        double[][] scaled = equilibrate(gram, scale);
        double[][] inverseScaled = invert(scaled);
        double[] scaledRhs = new double[p];
        for (int j = 0; j < p; j++) {
            scaledRhs[j] = xty[j] * scale[j];
        }
        double[] beta = new double[p];
        for (int j = 0; j < p; j++) {
            double sum = 0.0;
            for (int k = 0; k < p; k++) {
                sum += inverseScaled[j][k] * scaledRhs[k];
            }
            beta[j] = sum * scale[j];
        }

        double mean = 0.0;
        for (double value : y) {
            mean += value;
        }
        mean /= n;
        double rss = 0.0;
        double tss = 0.0;
        for (int i = 0; i < n; i++) {
            double predicted = 0.0;
            for (int j = 0; j < p; j++) {
                predicted += x[i][j] * beta[j];
            }
            double residual = y[i] - predicted;
            rss += residual * residual;
            tss += (y[i] - mean) * (y[i] - mean);
        }
        double residualVariance = rss / (n - p);
        double[] standardErrors = new double[p];
        for (int j = 0; j < p; j++) {
            double unscaledDiagonal = inverseScaled[j][j] * scale[j] * scale[j];
            standardErrors[j] = Math.sqrt(Math.max(0.0, residualVariance * unscaledDiagonal));
        }
        double rSquared = tss > 0.0 ? 1.0 - rss / tss : 0.0;
        return new Fit(beta, standardErrors, rSquared, norm1(scaled) * norm1(inverseScaled), n);
    }

    /**
     * Returns the 1-norm condition number of the equilibrated Gram matrix of {@code x}, or positive
     * infinity when the columns are linearly dependent.
     *
     * @param x rows of regressors
     * @return condition number
     */
    public static double conditionNumber(double[][] x) {
        if (x == null || x.length == 0 || x[0].length == 0) {
            return Double.POSITIVE_INFINITY;
        }
        double[][] gram = gram(x);
        double[] scale;
        try {
            scale = scale(gram);
        } catch (IllegalArgumentException degenerate) {
            return Double.POSITIVE_INFINITY;
        }
        double[][] scaled = equilibrate(gram, scale);
        try {
            return norm1(scaled) * norm1(invert(scaled));
        } catch (IllegalArgumentException singular) {
            return Double.POSITIVE_INFINITY;
        }
    }

    private static int requireRows(double[][] x, double[] y) {
        if (x == null || y == null || x.length != y.length || x.length == 0) {
            throw new IllegalArgumentException("Regression needs one target per row.");
        }
        int p = x[0].length;
        if (p == 0) {
            throw new IllegalArgumentException("Regression needs at least one column.");
        }
        for (int i = 0; i < x.length; i++) {
            if (x[i].length != p) {
                throw new IllegalArgumentException("Regression rows must have equal length.");
            }
            for (double value : x[i]) {
                if (!Double.isFinite(value)) {
                    throw new IllegalArgumentException("Regression regressors must be finite.");
                }
            }
            if (!Double.isFinite(y[i])) {
                throw new IllegalArgumentException("Regression targets must be finite.");
            }
        }
        return x.length;
    }

    private static double[][] gram(double[][] x) {
        int p = x[0].length;
        double[][] gram = new double[p][p];
        for (double[] row : x) {
            for (int j = 0; j < p; j++) {
                for (int k = 0; k <= j; k++) {
                    gram[j][k] += row[j] * row[k];
                }
            }
        }
        for (int j = 0; j < p; j++) {
            for (int k = j + 1; k < p; k++) {
                gram[j][k] = gram[k][j];
            }
        }
        return gram;
    }

    private static double[] scale(double[][] gram) {
        double[] scale = new double[gram.length];
        for (int j = 0; j < gram.length; j++) {
            double diagonal = gram[j][j];
            if (!(diagonal > 0.0) || !Double.isFinite(diagonal)) {
                throw new IllegalArgumentException("Regression column " + j
                        + " has no excitation (it is constant or zero).");
            }
            scale[j] = 1.0 / Math.sqrt(diagonal);
        }
        return scale;
    }

    private static double[][] equilibrate(double[][] gram, double[] scale) {
        double[][] scaled = new double[gram.length][gram.length];
        for (int j = 0; j < gram.length; j++) {
            for (int k = 0; k < gram.length; k++) {
                scaled[j][k] = gram[j][k] * scale[j] * scale[k];
            }
        }
        return scaled;
    }

    /** Gauss-Jordan inversion with partial pivoting; the matrix is small (one row per parameter). */
    private static double[][] invert(double[][] matrix) {
        int p = matrix.length;
        double[][] a = new double[p][2 * p];
        for (int i = 0; i < p; i++) {
            System.arraycopy(matrix[i], 0, a[i], 0, p);
            a[i][p + i] = 1.0;
        }
        for (int column = 0; column < p; column++) {
            int pivot = column;
            for (int row = column + 1; row < p; row++) {
                if (Math.abs(a[row][column]) > Math.abs(a[pivot][column])) {
                    pivot = row;
                }
            }
            if (Math.abs(a[pivot][column]) < SINGULAR_PIVOT) {
                throw new IllegalArgumentException("Regression is singular: columns are collinear.");
            }
            double[] swap = a[column];
            a[column] = a[pivot];
            a[pivot] = swap;
            double divisor = a[column][column];
            for (int k = 0; k < 2 * p; k++) {
                a[column][k] /= divisor;
            }
            for (int row = 0; row < p; row++) {
                if (row == column) {
                    continue;
                }
                double factor = a[row][column];
                if (factor == 0.0) {
                    continue;
                }
                for (int k = 0; k < 2 * p; k++) {
                    a[row][k] -= factor * a[column][k];
                }
            }
        }
        double[][] inverse = new double[p][p];
        for (int i = 0; i < p; i++) {
            System.arraycopy(a[i], p, inverse[i], 0, p);
        }
        return inverse;
    }

    private static double norm1(double[][] matrix) {
        double max = 0.0;
        for (int k = 0; k < matrix.length; k++) {
            double sum = 0.0;
            for (double[] row : matrix) {
                sum += Math.abs(row[k]);
            }
            max = Math.max(max, sum);
        }
        return max;
    }
}
