package ca.frc6390.athena.mechanism.model;

/**
 * Recursive least squares with exponential forgetting.
 *
 * <p>For each regressor {@code x} and target {@code y}: the gain is {@code k = P x / (lambda + x' P x)},
 * the coefficients move by {@code k * (y - x' theta)}, and the covariance becomes
 * {@code P = (P - k x' P) / lambda}. The residual variance is an exponentially weighted average of
 * squared innovations, and {@code se_j = sqrt(s^2 P_jj)} is the running standard error.</p>
 *
 * <p>The estimator is a plain value holder with no clock or storage. Callers decide when to update it.</p>
 */
public final class RecursiveLeastSquares {
    private final int parameters;
    private final double forgettingFactor;
    private final double initialCovariance;
    private final double[] theta;
    private final double[][] covariance;
    private double residualVariance;
    private long updates;

    /**
     * Creates an estimator.
     *
     * @param parameters number of coefficients
     * @param forgettingFactor lambda in (0, 1]; values below one discount old data
     * @param initialCovariance diagonal of the initial covariance; large means a weak prior
     */
    public RecursiveLeastSquares(int parameters, double forgettingFactor, double initialCovariance) {
        if (parameters <= 0) {
            throw new IllegalArgumentException("RLS needs at least one parameter.");
        }
        if (!(forgettingFactor > 0.0 && forgettingFactor <= 1.0)) {
            throw new IllegalArgumentException("RLS forgetting factor must be in (0, 1].");
        }
        if (!(initialCovariance > 0.0) || !Double.isFinite(initialCovariance)) {
            throw new IllegalArgumentException("RLS initial covariance must be positive and finite.");
        }
        this.parameters = parameters;
        this.forgettingFactor = forgettingFactor;
        this.initialCovariance = initialCovariance;
        this.theta = new double[parameters];
        this.covariance = new double[parameters][parameters];
        reset();
    }

    /** Starts over from zero coefficients and the initial covariance. */
    public void reset() {
        java.util.Arrays.fill(theta, 0.0);
        for (int i = 0; i < parameters; i++) {
            java.util.Arrays.fill(covariance[i], 0.0);
            covariance[i][i] = initialCovariance;
        }
        residualVariance = 0.0;
        updates = 0L;
    }

    /** Replaces the coefficients (for example from an offline fit) and keeps the covariance. */
    public void seed(double[] coefficients) {
        if (coefficients.length != parameters) {
            throw new IllegalArgumentException("Seed needs " + parameters + " coefficients.");
        }
        System.arraycopy(coefficients, 0, theta, 0, parameters);
    }

    /** Incorporates one observation. */
    public void update(double[] regressor, double target) {
        if (regressor.length != parameters) {
            throw new IllegalArgumentException("Regressor needs " + parameters + " values.");
        }
        if (!Double.isFinite(target)) {
            throw new IllegalArgumentException("RLS target must be finite.");
        }
        double[] px = new double[parameters];
        for (int i = 0; i < parameters; i++) {
            for (int j = 0; j < parameters; j++) {
                px[i] += covariance[i][j] * regressor[j];
            }
        }
        double denominator = forgettingFactor;
        for (int i = 0; i < parameters; i++) {
            denominator += regressor[i] * px[i];
        }
        if (!(denominator > 0.0) || !Double.isFinite(denominator)) {
            return;
        }
        double predicted = 0.0;
        for (int i = 0; i < parameters; i++) {
            predicted += regressor[i] * theta[i];
        }
        double innovation = target - predicted;
        for (int i = 0; i < parameters; i++) {
            double gain = px[i] / denominator;
            theta[i] += gain * innovation;
            for (int j = 0; j < parameters; j++) {
                covariance[i][j] = (covariance[i][j] - gain * px[j]) / forgettingFactor;
            }
        }
        residualVariance = updates == 0
                ? innovation * innovation
                : forgettingFactor * residualVariance + (1.0 - forgettingFactor) * innovation * innovation;
        updates++;
    }

    /** Returns the current coefficients. */
    public double[] coefficients() {
        return theta.clone();
    }

    /** Returns the running standard error of one coefficient. */
    public double standardError(int index) {
        return Math.sqrt(Math.max(0.0, residualVariance * covariance[index][index]));
    }

    /** Returns the number of accepted updates since the last reset. */
    public long updates() {
        return updates;
    }

    /** Returns the number of coefficients. */
    public int parameters() {
        return parameters;
    }
}
