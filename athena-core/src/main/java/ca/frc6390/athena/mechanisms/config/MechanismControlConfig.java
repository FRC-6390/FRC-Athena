package ca.frc6390.athena.mechanisms.config;

import java.util.List;

/**
 * Control configuration for the mechanism.
 *
 * <p>This focuses on "base" control: output type and optional PID/FF profiles that can be selected
 * by name. Custom loops and hooks remain code-only.
 */
public record MechanismControlConfig(
        String output,
        Boolean setpointAsOutput,
        Boolean pidUseVelocity,
        Boolean pidContinuous,
        Double pidContinuousMin,
        Double pidContinuousMax,
        Double tolerance,
        String pidProfile,
        String ffProfile,
        List<MechanismPidConfig> pidProfiles,
        List<MechanismFeedforwardConfig> ffProfiles
) {
}

