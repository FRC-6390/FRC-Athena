package ca.frc6390.athena.hardware.encoder;

import java.util.Arrays;
import java.util.Objects;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.dashboard.ShuffleboardControls;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Aggregates multiple encoders and exposes averaged readings to mimic the legacy encoder group API.
 */
public class EncoderGroup implements RobotSendableDevice {
    private final Encoder[] encoders;
    private double shuffleboardPeriodSecondsOverride = Double.NaN;
    private ShuffleboardLayout lastShuffleboardLayoutComp;
    private ShuffleboardLayout lastShuffleboardLayoutDebug;

    public EncoderGroup(Encoder... encoders) {
        this.encoders = encoders;
    }

    public static EncoderGroup fromConfigs(EncoderConfig... configs) {
        return new EncoderGroup(Arrays.stream(configs).map(HardwareFactories::encoder).toArray(Encoder[]::new));
    }

    public static EncoderGroup fromMotorGroup(MotorControllerGroup motors) {
        Encoder[] encoders = Arrays.stream(motors.getControllers())
                .map(m -> {
                    Encoder encoder = m.getEncoder();
                    if (encoder == null) {
                        DriverStation.reportWarning(
                                "Motor controller '" + m.getName() + "' has no encoder; skipping in EncoderGroup.",
                                false);
                        return null;
                    }
                    encoder.setInverted(m.isInverted());
                    return encoder;
                })
                .filter(Objects::nonNull)
                .toArray(Encoder[]::new);
        if (encoders.length == 0) {
            DriverStation.reportWarning("No encoders available for MotorControllerGroup; EncoderGroup will be empty.", false);
        }
        return new EncoderGroup(encoders);
    }

    public Encoder[] getEncoders() {
        return encoders;
    }

    public double getVelocity() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getVelocity();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    public double getPosition() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getPosition();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    public double getRotations() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getRotations();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    public double getRate() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getRate();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    public void setPosition(double position) {
        for (Encoder encoder : encoders) {
            if (encoder != null) {
                encoder.setPosition(position);
            }
        }
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(getRotations());
    }

    public boolean allEncodersConnected() {
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            if (!encoder.isConnected()) {
                return false;
            }
        }
        return true;
    }

    public void update() {
        for (Encoder encoder : encoders) {
            if (encoder != null) {
                encoder.update();
            }
        }
    }

    @Override
    public double getShuffleboardPeriodSeconds() {
        return Double.isFinite(shuffleboardPeriodSecondsOverride) && shuffleboardPeriodSecondsOverride > 0.0
                ? shuffleboardPeriodSecondsOverride
                : ca.frc6390.athena.core.RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();
    }

    @Override
    public void setShuffleboardPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
            shuffleboardPeriodSecondsOverride = Double.NaN;
            return;
        }
        shuffleboardPeriodSecondsOverride = periodSeconds;
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        if (level == SendableLevel.DEBUG) {
            if (layout == lastShuffleboardLayoutDebug) {
                return layout;
            }
            boolean compAlreadyBuilt = (layout == lastShuffleboardLayoutComp);
            lastShuffleboardLayoutDebug = layout;
            if (!compAlreadyBuilt) {
                lastShuffleboardLayoutComp = layout;
                publishCommon(layout);
            }
            // No debug-only widgets today; avoid re-adding titles.
            return layout;
        }

        if (layout == lastShuffleboardLayoutComp) {
            return layout;
        }
        lastShuffleboardLayoutComp = layout;
        publishCommon(layout);
        return layout;
    }

    private void publishCommon(ShuffleboardLayout layout) {
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        ShuffleboardLayout summary = layout.getLayout("Summary", BuiltInLayouts.kList);
        summary.addDouble("Average Position", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedPosition, period));
        summary.addDouble("Average Velocity", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedVelocity, period));
        summary.addDouble("Average Rotations", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedRotations, period));
        summary.addDouble("Average Rate", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedRate, period));
        summary.addBoolean("All Connected", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::allEncodersCachedConnected, period));

        if (ShuffleboardControls.enabled(ShuffleboardControls.Flag.ENCODER_GROUP_PER_ENCODER)) {
            Arrays.stream(encoders)
                    .filter(Objects::nonNull)
                    .forEach(encoder ->
                            encoder.shuffleboard(layout.getLayout(encoder.getName(), BuiltInLayouts.kList), SendableLevel.COMP));
        }
    }

    private double getCachedPosition() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getCachedPosition();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    private double getCachedVelocity() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getCachedVelocity();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    private double getCachedRotations() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getCachedRotations();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    private double getCachedRate() {
        double sum = 0.0;
        int count = 0;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            sum += encoder.getCachedRate();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    private boolean allEncodersCachedConnected() {
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            if (!encoder.isCachedConnected()) {
                return false;
            }
        }
        return true;
    }
}
