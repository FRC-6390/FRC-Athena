package ca.frc6390.athena.drivetrains;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;

/**
 * Shared registration/apply logic for drivetrain speed sources and blend rules.
 */
public final class DrivetrainSpeedConfigSupport {

    private final List<SpeedSourceRegistration> speedSources = new ArrayList<>();
    private final List<SpeedSourceBlendRegistration> speedSourceBlends = new ArrayList<>();
    private final List<SpeedOutputBlendRegistration> speedOutputBlends = new ArrayList<>();

    private record SpeedSourceRegistration(String name, boolean enabledByDefault) {}

    private record SpeedSourceBlendRegistration(
            String target,
            String left,
            String right,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis[] axes) {}

    private record SpeedOutputBlendRegistration(
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis[] axes) {}

    public void source(String name, boolean enabledByDefault) {
        speedSources.add(new SpeedSourceRegistration(name, enabledByDefault));
    }

    public void blend(
            String target,
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        blend(target, target, source, blendMode, axes);
    }

    public void blend(
            String target,
            String left,
            String right,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        RobotSpeeds.SpeedAxis[] resolvedAxes =
                axes == null ? new RobotSpeeds.SpeedAxis[0] : axes.clone();
        speedSourceBlends.add(new SpeedSourceBlendRegistration(target, left, right, blendMode, resolvedAxes));
    }

    public void outputBlend(
            String source,
            RobotSpeeds.BlendMode blendMode,
            RobotSpeeds.SpeedAxis... axes) {
        RobotSpeeds.SpeedAxis[] resolvedAxes =
                axes == null ? new RobotSpeeds.SpeedAxis[0] : axes.clone();
        speedOutputBlends.add(new SpeedOutputBlendRegistration(source, blendMode, resolvedAxes));
    }

    public void apply(RobotSpeeds speeds) {
        if (speeds == null) {
            return;
        }
        speeds.resetBlendsToDefaults();
        for (SpeedSourceRegistration source : speedSources) {
            speeds.registerSpeedSource(source.name(), source.enabledByDefault());
        }
        for (SpeedSourceBlendRegistration blend : speedSourceBlends) {
            speeds.blend(
                    blend.target(),
                    blend.left(),
                    blend.right(),
                    blend.blendMode(),
                    blend.axes());
        }
        for (SpeedOutputBlendRegistration blend : speedOutputBlends) {
            speeds.blendToOutput(
                    blend.source(),
                    blend.blendMode(),
                    blend.axes());
        }
    }

    public void apply(SwerveDrivetrainConfig.SpeedSection section) {
        if (section == null) {
            return;
        }
        for (SpeedSourceRegistration source : speedSources) {
            section.source(source.name(), source.enabledByDefault());
        }
        for (SpeedSourceBlendRegistration blend : speedSourceBlends) {
            section.blend(
                    blend.target(),
                    blend.left(),
                    blend.right(),
                    blend.blendMode(),
                    blend.axes());
        }
        for (SpeedOutputBlendRegistration blend : speedOutputBlends) {
            section.outputBlend(
                    blend.source(),
                    blend.blendMode(),
                    blend.axes());
        }
    }

    public void apply(DifferentialDrivetrainConfig.SpeedSection section) {
        if (section == null) {
            return;
        }
        for (SpeedSourceRegistration source : speedSources) {
            section.source(source.name(), source.enabledByDefault());
        }
        for (SpeedSourceBlendRegistration blend : speedSourceBlends) {
            section.blend(
                    blend.target(),
                    blend.left(),
                    blend.right(),
                    blend.blendMode(),
                    blend.axes());
        }
        for (SpeedOutputBlendRegistration blend : speedOutputBlends) {
            section.outputBlend(
                    blend.source(),
                    blend.blendMode(),
                    blend.axes());
        }
    }
}
