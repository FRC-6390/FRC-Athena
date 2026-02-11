package ca.frc6390.athena.hardware.encoder;

import java.util.Arrays;
import java.util.Objects;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Aggregates multiple encoders and exposes averaged readings to mimic the legacy encoder group API.
 */
public class EncoderGroup implements RobotSendableDevice {
    private final Encoder[] encoders;

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
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }
        publish(node);
        return node;
    }

    private void publish(RobotNetworkTables.Node node) {
        RobotNetworkTables.Node summary = node.child("Summary");
        summary.putDouble("avgPosition", getAveragePosition());
        summary.putDouble("avgVelocity", getAverageVelocity());
        summary.putDouble("avgRotations", getAverageRotations());
        summary.putDouble("avgRate", getAverageRate());
        summary.putBoolean("allConnected", allEncodersConnected());

        if (node.robot().enabled(RobotNetworkTables.Flag.ENCODER_GROUP_PER_ENCODER)) {
            RobotNetworkTables.Node encNode = node.child("Encoders");
            Arrays.stream(encoders)
                    .filter(Objects::nonNull)
                    .forEach(encoder -> {
                        String key = encoder.getName();
                        key = key != null ? key.replace('\\', '_').replace('/', '_') : "encoder";
                        encoder.networkTables(encNode.child(key));
                    });
        }
    }

    private double getAveragePosition() {
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

    private double getAverageVelocity() {
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

    private double getAverageRotations() {
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

    private double getAverageRate() {
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

}
