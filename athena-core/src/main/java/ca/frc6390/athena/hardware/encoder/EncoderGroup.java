package ca.frc6390.athena.hardware.encoder;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Aggregates multiple encoders and exposes averaged readings to mimic the legacy encoder group API.
 */
public class EncoderGroup implements RobotSendableDevice {
    private final Encoder[] encoders;
    private final String[] encoderTopicKeys;
    private String publishNodePath;
    private RobotNetworkTables.Node summaryNode;
    private RobotNetworkTables.Node[] encoderNodes;

    public EncoderGroup(Encoder... encoders) {
        this.encoders = encoders;
        this.encoderTopicKeys = buildTopicKeys(encoders);
    }

    public static EncoderGroup fromConfigs(EncoderConfig... configs) {
        if (configs == null || configs.length == 0) {
            return new EncoderGroup();
        }
        Encoder[] resolved = new Encoder[configs.length];
        for (int i = 0; i < configs.length; i++) {
            resolved[i] = HardwareFactories.encoder(configs[i]);
        }
        return new EncoderGroup(resolved);
    }

    public static EncoderGroup fromMotorGroup(MotorControllerGroup motors) {
        if (motors == null || motors.getControllers() == null || motors.getControllers().length == 0) {
            DriverStation.reportWarning("No motor controllers available; EncoderGroup will be empty.", false);
            return new EncoderGroup();
        }
        MotorController[] controllers = motors.getControllers();
        Encoder[] buffer = new Encoder[controllers.length];
        int count = 0;
        for (MotorController controller : controllers) {
            if (controller == null) {
                continue;
            }
            Encoder encoder = controller.getEncoder();
            if (encoder == null) {
                DriverStation.reportWarning(
                        "Motor controller '" + controller.getName() + "' has no encoder; skipping in EncoderGroup.",
                        false);
                continue;
            }
            encoder.setInverted(controller.isInverted());
            buffer[count++] = encoder;
        }
        if (count == 0) {
            DriverStation.reportWarning("No encoders available for MotorControllerGroup; EncoderGroup will be empty.", false);
            return new EncoderGroup();
        }
        if (count == buffer.length) {
            return new EncoderGroup(buffer);
        }
        Encoder[] resolved = new Encoder[count];
        System.arraycopy(buffer, 0, resolved, 0, count);
        return new EncoderGroup(resolved);
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
        RobotNetworkTables nt = node.robot();
        ensurePublishNodes(node);
        double positionSum = 0.0;
        double velocitySum = 0.0;
        double rotationsSum = 0.0;
        double rateSum = 0.0;
        int count = 0;
        boolean allConnected = true;
        for (Encoder encoder : encoders) {
            if (encoder == null) {
                continue;
            }
            positionSum += encoder.getPosition();
            velocitySum += encoder.getVelocity();
            rotationsSum += encoder.getRotations();
            rateSum += encoder.getRate();
            allConnected &= encoder.isConnected();
            count++;
        }
        double invCount = count > 0 ? 1.0 / count : 0.0;

        summaryNode.putDouble("avgPosition", positionSum * invCount);
        summaryNode.putDouble("avgVelocity", velocitySum * invCount);
        summaryNode.putDouble("avgRotations", rotationsSum * invCount);
        summaryNode.putDouble("avgRate", rateSum * invCount);
        summaryNode.putBoolean("allConnected", allConnected);

        if (nt.enabled(RobotNetworkTables.Flag.ENCODER_GROUP_PER_ENCODER)) {
            for (int i = 0; i < encoders.length; i++) {
                Encoder encoder = encoders[i];
                if (encoder == null) {
                    continue;
                }
                if (encoderNodes != null && i < encoderNodes.length && encoderNodes[i] != null) {
                    encoder.networkTables(encoderNodes[i]);
                }
            }
        }
    }

    private void ensurePublishNodes(RobotNetworkTables.Node node) {
        String path = node.path();
        if (path != null && path.equals(publishNodePath) && summaryNode != null) {
            return;
        }
        publishNodePath = path;
        summaryNode = node.child("Summary");
        RobotNetworkTables.Node encodersRoot = node.child("Encoders");
        encoderNodes = new RobotNetworkTables.Node[encoders.length];
        for (int i = 0; i < encoders.length; i++) {
            String key = encoderTopicKeys != null && i < encoderTopicKeys.length
                    ? encoderTopicKeys[i]
                    : "encoder";
            encoderNodes[i] = encodersRoot.child(key);
        }
    }

    private static String[] buildTopicKeys(Encoder[] encoders) {
        if (encoders == null || encoders.length == 0) {
            return new String[0];
        }
        String[] keys = new String[encoders.length];
        for (int i = 0; i < encoders.length; i++) {
            Encoder encoder = encoders[i];
            String fallback = "encoder";
            keys[i] = sanitizeTopicKey(encoder != null ? encoder.getName() : null, fallback);
        }
        return keys;
    }

    private static String sanitizeTopicKey(String raw, String fallback) {
        if (raw == null || raw.isBlank()) {
            return fallback;
        }
        return raw.replace('\\', '_').replace('/', '_');
    }

}
