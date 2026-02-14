package ca.frc6390.athena.drivetrains;

import java.util.Objects;

import ca.frc6390.athena.core.RobotSpeeds;

/**
 * Shared speed source/blend section behavior for drivetrain configs.
 */
public abstract class DrivetrainSpeedSectionBase<Self extends DrivetrainSpeedSectionBase<Self>> {
    private final DrivetrainSpeedConfigSupport speedConfig;

    protected DrivetrainSpeedSectionBase(DrivetrainSpeedConfigSupport speedConfig) {
        this.speedConfig = Objects.requireNonNull(speedConfig, "speedConfig");
    }

    protected abstract Self self();

    public final Self source(String name, boolean enabledByDefault) {
        speedConfig.source(name, enabledByDefault);
        return self();
    }

    public final Self blend(
            String target,
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        speedConfig.blend(target, source, blendMode, axes);
        return self();
    }

    public final Self blend(
            String target,
            String left,
            String right,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        speedConfig.blend(target, left, right, blendMode, axes);
        return self();
    }

    public final Self outputBlend(
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        speedConfig.outputBlend(source, blendMode, axes);
        return self();
    }
}
