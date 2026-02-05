package ca.frc6390.athena.hardware.encoder;

import java.util.Arrays;
import java.util.Objects;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Aggregates multiple encoders and exposes averaged readings to mimic the legacy encoder group API.
 */
public class EncoderGroup implements RobotSendableDevice {
    private final Encoder[] encoders;
    private double shuffleboardPeriodSeconds = ca.frc6390.athena.core.RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();

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
        return Arrays.stream(encoders).mapToDouble(Encoder::getVelocity).average().orElse(0.0);
    }

    public double getPosition() {
        return Arrays.stream(encoders).mapToDouble(Encoder::getPosition).average().orElse(0.0);
    }

    public double getRotations() {
        return Arrays.stream(encoders).mapToDouble(Encoder::getRotations).average().orElse(0.0);
    }

    public double getRate() {
        return Arrays.stream(encoders).mapToDouble(Encoder::getRate).average().orElse(0.0);
    }

    public void setPosition(double position) {
        Arrays.stream(encoders).forEach(e -> e.setPosition(position));
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(getRotations());
    }

    public boolean allEncodersConnected() {
        return Arrays.stream(encoders).allMatch(Encoder::isConnected);
    }

    public void update() {
        Arrays.stream(encoders).forEach(Encoder::update);
    }

    @Override
    public double getShuffleboardPeriodSeconds() {
        return shuffleboardPeriodSeconds;
    }

    @Override
    public void setShuffleboardPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds)) {
            return;
        }
        shuffleboardPeriodSeconds = periodSeconds;
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        ShuffleboardLayout summary = layout.getLayout("Summary", BuiltInLayouts.kList);
        summary.addDouble("Average Position", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedPosition, period));
        summary.addDouble("Average Velocity", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedVelocity, period));
        summary.addDouble("Average Rotations", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedRotations, period));
        summary.addDouble("Average Rate", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getCachedRate, period));
        summary.addBoolean("All Connected", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::allEncodersCachedConnected, period));

        Arrays.stream(encoders)
                .filter(Objects::nonNull)
                .forEach(encoder ->
                        encoder.shuffleboard(layout.getLayout(encoder.getName(), BuiltInLayouts.kList), level));
        return layout;
    }

    private double getCachedPosition() {
        return Arrays.stream(encoders)
                .mapToDouble(Encoder::getCachedPosition)
                .average()
                .orElse(0.0);
    }

    private double getCachedVelocity() {
        return Arrays.stream(encoders)
                .mapToDouble(Encoder::getCachedVelocity)
                .average()
                .orElse(0.0);
    }

    private double getCachedRotations() {
        return Arrays.stream(encoders)
                .mapToDouble(Encoder::getCachedRotations)
                .average()
                .orElse(0.0);
    }

    private double getCachedRate() {
        return Arrays.stream(encoders)
                .mapToDouble(Encoder::getCachedRate)
                .average()
                .orElse(0.0);
    }

    private boolean allEncodersCachedConnected() {
        return Arrays.stream(encoders).allMatch(Encoder::isCachedConnected);
    }
}
