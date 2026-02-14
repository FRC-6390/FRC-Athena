package ca.frc6390.athena.mechanisms.config;

import java.util.List;

/**
 * Control configuration for the mechanism.
 *
 * <p>This focuses on "base" control and named controller registrations. Custom loops and hooks
 * remain code-only.
 */
public record MechanismControlConfig(
        String output,
        Boolean setpointAsOutput,
        Boolean pidContinuous,
        Double pidContinuousMin,
        Double pidContinuousMax,
        Double tolerance,
        List<MechanismPidConfig> pidProfiles,
        List<MechanismBangBangConfig> bangBangProfiles,
        List<MechanismFeedforwardConfig> ffProfiles
) {
}
