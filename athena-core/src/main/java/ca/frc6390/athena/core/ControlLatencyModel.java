package ca.frc6390.athena.core;

import edu.wpi.first.math.MathUtil;

/**
 * Shared timing utilities for delay-aware feedforward/compensation logic.
 */
public final class ControlLatencyModel {
    private ControlLatencyModel() {}

    /**
     * Returns how old a source sample is in seconds.
     */
    public static double sourceAgeSeconds(double nowSeconds, double sourceLastUpdateSeconds) {
        if (!Double.isFinite(nowSeconds) || !Double.isFinite(sourceLastUpdateSeconds)) {
            return 0.0;
        }
        double age = nowSeconds - sourceLastUpdateSeconds;
        if (!Double.isFinite(age)) {
            return 0.0;
        }
        return Math.max(0.0, age);
    }

    /**
     * Returns half-period sample-and-hold timing for a source update period.
     */
    public static double sampleAndHoldLeadSeconds(double sourceEstimatedPeriodSeconds) {
        if (!Double.isFinite(sourceEstimatedPeriodSeconds) || sourceEstimatedPeriodSeconds <= 0.0) {
            return 0.0;
        }
        return 0.5 * sourceEstimatedPeriodSeconds;
    }

    /**
     * Computes effective input latency: source age + half source period + pipeline delay.
     */
    public static double effectiveInputLatencySeconds(
            double nowSeconds,
            double sourceLastUpdateSeconds,
            double sourceEstimatedPeriodSeconds,
            double pipelineDelaySeconds,
            double maxLatencySeconds) {
        double sourceAge = sourceAgeSeconds(nowSeconds, sourceLastUpdateSeconds);
        double sampleAndHoldLead = sampleAndHoldLeadSeconds(sourceEstimatedPeriodSeconds);
        double pipelineDelay = (Double.isFinite(pipelineDelaySeconds) && pipelineDelaySeconds > 0.0)
                ? pipelineDelaySeconds
                : 0.0;
        double latency = sourceAge + sampleAndHoldLead + pipelineDelay;
        if (!Double.isFinite(latency)) {
            latency = 0.0;
        }
        if (Double.isFinite(maxLatencySeconds) && maxLatencySeconds > 0.0) {
            latency = MathUtil.clamp(latency, 0.0, maxLatencySeconds);
        } else {
            latency = Math.max(0.0, latency);
        }
        return latency;
    }

    /**
     * Computes a final predictive lead after applying adaptive correction.
     */
    public static double predictiveLeadSeconds(
            double effectiveInputLatencySeconds,
            double adaptiveLeadSeconds,
            double maxLeadSeconds) {
        double effective = (Double.isFinite(effectiveInputLatencySeconds) && effectiveInputLatencySeconds > 0.0)
                ? effectiveInputLatencySeconds
                : 0.0;
        double adaptive = (Double.isFinite(adaptiveLeadSeconds) && adaptiveLeadSeconds > 0.0)
                ? adaptiveLeadSeconds
                : 0.0;
        double lead = effective + adaptive;
        if (Double.isFinite(maxLeadSeconds) && maxLeadSeconds > 0.0) {
            lead = MathUtil.clamp(lead, 0.0, maxLeadSeconds);
        } else {
            lead = Math.max(0.0, lead);
        }
        return lead;
    }
}
