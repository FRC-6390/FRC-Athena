package ca.frc6390.athena.core.context;

import java.util.function.Consumer;

import ca.frc6390.athena.core.ControlLatencyModel;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotMechanisms;
import ca.frc6390.athena.core.RobotTime;
import ca.frc6390.athena.networktables.AthenaNT;
import ca.frc6390.athena.networktables.NtScope;

/**
 * Shared context contract for APIs that can access a {@link RobotCore} instance.
 */
public interface RobotScopedContext {
    RobotCore<?> robotCore();

    /**
     * Default robot-core-scoped Athena NetworkTables view.
     */
    default NtScope nt() {
        return AthenaNT.scope("RobotCore/NetworkTables");
    }

    /**
     * Returns the latest loop timestamp with monotonic projection between base loop updates.
     */
    default double robotNowSeconds() {
        return RobotTime.nowSecondsProjectedIfStale(0.002);
    }

    /**
     * Returns the latest measured base-loop dt, or {@link Double#NaN} if unavailable.
     */
    default double robotLoopDtSeconds() {
        return RobotTime.loopDtSeconds();
    }

    /**
     * Returns the number of base loop cycles observed by {@link RobotTime}.
     */
    default long robotLoopCycleCount() {
        return RobotTime.loopCycleCount();
    }

    /**
     * Returns a point-in-time snapshot of base loop timing values.
     */
    default RobotTime.LoopSnapshot robotLoopSnapshot() {
        return RobotTime.loopSnapshot();
    }

    /**
     * Returns source sample age in seconds relative to {@link #robotNowSeconds()}.
     */
    default double sourceAgeSeconds(double sourceLastUpdateSeconds) {
        return ControlLatencyModel.sourceAgeSeconds(robotNowSeconds(), sourceLastUpdateSeconds);
    }

    /**
     * Computes delay-aware input latency for a source.
     */
    default double effectiveInputLatencySeconds(
            double sourceLastUpdateSeconds,
            double sourceEstimatedPeriodSeconds,
            double pipelineDelaySeconds,
            double maxLatencySeconds) {
        return ControlLatencyModel.effectiveInputLatencySeconds(
                robotNowSeconds(),
                sourceLastUpdateSeconds,
                sourceEstimatedPeriodSeconds,
                pipelineDelaySeconds,
                maxLatencySeconds);
    }

    /**
     * Computes a bounded predictive lead using effective latency and adaptive correction.
     */
    default double predictiveLeadSeconds(
            double effectiveLatencySeconds,
            double adaptiveLeadSeconds,
            double maxLeadSeconds) {
        return ControlLatencyModel.predictiveLeadSeconds(
                effectiveLatencySeconds,
                adaptiveLeadSeconds,
                maxLeadSeconds);
    }

    /**
     * Returns the global robot-wide mechanisms view (lookup by name/config/type).
     */
    default RobotMechanisms robotMechanisms() {
        RobotCore<?> core = robotCore();
        if (core == null) {
            throw new IllegalStateException("No RobotCore available in context");
        }
        return core.mechanisms();
    }

    /**
     * Sectioned interaction helper for other already-built mechanisms/superstructures.
     */
    default void robotMechanisms(Consumer<RobotMechanisms.InteractionSection> section) {
        robotMechanisms().use(section);
    }
}
