package ca.frc6390.athena.hardware.power;

import java.util.Objects;

/**
 * Lumped winding-temperature estimate for one motor.
 *
 * <p>Each cycle: {@code T += (I^2 R / C - (T - T_ambient) / tau) * dt}, where {@code I} is the
 * stator current. When the vendor controller reports a temperature, the model state is pulled
 * toward it with a first-order gain of {@code dt / FUSE_SECONDS}, and {@link #celsius()} returns the
 * vendor value (the measured value is the better estimate when it exists).</p>
 */
public final class ThermalModel {
    /** Time constant of the pull toward a vendor temperature reading. */
    public static final double FUSE_SECONDS = 2.0;

    private final MotorThermalSpec spec;
    private double modelCelsius;
    private double vendorCelsius = Double.NaN;

    public ThermalModel(MotorThermalSpec spec) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.modelCelsius = spec.ambientCelsius();
    }

    /**
     * Advances the model.
     *
     * @param statorAmps stator (winding) current; non-finite values are treated as zero
     * @param dtSeconds elapsed time, non-negative
     */
    public void step(double statorAmps, double dtSeconds) {
        if (!(dtSeconds > 0.0)) return;
        double current = Double.isFinite(statorAmps) ? statorAmps : 0.0;
        double heating = current * current * spec.resistanceOhms() / spec.heatCapacityJoulesPerCelsius();
        double cooling = (modelCelsius - spec.ambientCelsius()) / spec.coolingSeconds();
        modelCelsius += (heating - cooling) * dtSeconds;
        if (Double.isFinite(vendorCelsius)) {
            modelCelsius += (vendorCelsius - modelCelsius) * Math.min(1.0, dtSeconds / FUSE_SECONDS);
        }
    }

    /**
     * Records a vendor-reported winding temperature.
     *
     * @param celsius reading, or NaN when the controller does not report one
     */
    public void vendorReading(double celsius) {
        vendorCelsius = Double.isFinite(celsius) ? celsius : Double.NaN;
    }

    /** Returns the model-only estimate in Celsius. */
    public double modelCelsius() { return modelCelsius; }

    /** Returns the fused estimate: the vendor reading when available, otherwise the model. */
    public double celsius() { return Double.isFinite(vendorCelsius) ? vendorCelsius : modelCelsius; }

    /** Returns whether a vendor reading is currently fused in. */
    public boolean vendorFused() { return Double.isFinite(vendorCelsius); }

    /** Returns the constants used by this model. */
    public MotorThermalSpec spec() { return spec; }
}
