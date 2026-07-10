package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.runtime.measurement.MeasurementSignal;

/**
 * Measurement signal refreshed from the runtime's cached hardware snapshot.
 */
public interface HardwareMeasurementSignal extends MeasurementSignal {
    /**
     * Refreshes this signal for one runtime cycle.
     *
     * @param context cached hardware access
     * @param timestampSeconds cycle timestamp
     * @param dtSeconds cycle duration
     */
    void refresh(ActionContext context, double timestampSeconds, double dtSeconds);
}
