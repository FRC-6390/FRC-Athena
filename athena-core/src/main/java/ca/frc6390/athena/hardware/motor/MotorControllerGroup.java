package ca.frc6390.athena.hardware.motor;

import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.hardware.encoder.EncoderGroup;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.controller.PIDController;

/**
 * Groups multiple {@link MotorController}s together to preserve the legacy group API while routing
 * through the new vendordep-backed hardware layer.
 */
public class MotorControllerGroup implements RobotSendableDevice {
    private final MotorController[] controllers;
    private final String[] controllerTopicKeys;
    private final OutputSection outputSection;
    private final ConfigSection configSection;
    private EncoderGroup encoders;
    private String publishNodePath;
    private RobotNetworkTables.Node summaryNode;
    private RobotNetworkTables.Node encodersNode;
    private RobotNetworkTables.Node[] motorNodes;

    public MotorControllerGroup(MotorController... controllers) {
        this.controllers = controllers;
        this.controllerTopicKeys = buildTopicKeys(controllers);
        this.outputSection = new OutputSection(this);
        this.configSection = new ConfigSection(this);
        this.encoders = EncoderGroup.fromMotorGroup(this);
    }

    public MotorControllerGroup(MotorController leftMotor, MotorController rightMotor) {
        this(new MotorController[] {leftMotor, rightMotor});
    }

    public static MotorControllerGroup fromConfigs(MotorControllerConfig... configs) {
        if (configs == null || configs.length == 0) {
            return new MotorControllerGroup();
        }
        MotorController[] motors = new MotorController[configs.length];
        for (int i = 0; i < configs.length; i++) {
            motors[i] = HardwareFactories.motor(configs[i]);
        }
        return new MotorControllerGroup(motors);
    }

    /**
     * Sectioned output API for already-built motor groups.
     */
    public MotorControllerGroup output(Consumer<OutputSection> section) {
        if (section != null) {
            section.accept(outputSection);
        }
        return this;
    }

    /**
     * Non-lambda section accessor for output interactions.
     */
    public OutputSection output() {
        return outputSection;
    }

    /**
     * Sectioned config API for already-built motor groups.
     */
    public MotorControllerGroup config(Consumer<ConfigSection> section) {
        if (section != null) {
            section.accept(configSection);
        }
        return this;
    }

    /**
     * Non-lambda section accessor for hardware config interactions.
     */
    public ConfigSection config() {
        return configSection;
    }

    public MotorController[] getControllers() {
        return controllers;
    }

    public MotorControllerGroup setNeutralMode(MotorNeutralMode mode) {
        for (MotorController controller : controllers) {
            controller.setNeutralMode(mode);
        }
        return this;
    }

    public MotorControllerGroup setEncoders(EncoderGroup encoders) {
        this.encoders = encoders;
        return this;
    }

    public void setCurrentLimit(double currentLimit) {
        for (MotorController controller : controllers) {
            controller.setCurrentLimit(currentLimit);
        }
    }

    public void setPosition(double position) {
        for (MotorController controller : controllers) {
            controller.setPosition(position);
        }
    }

    public void setVelocity(double rotationsPerSecond) {
        for (MotorController controller : controllers) {
            controller.setVelocity(rotationsPerSecond);
        }
    }

    public void setPid(PIDController pid) {
        for (MotorController controller : controllers) {
            controller.setPid(pid);
        }
    }

    public void setPid(double p, double i, double d) {
        setPid(new PIDController(p, i, d));
    }

    public void setSpeed(double speed) {
        for (MotorController controller : controllers) {
            controller.setSpeed(speed);
        }
    }

    public void setVoltage(double voltage) {
        for (MotorController controller : controllers) {
            controller.setVoltage(voltage);
        }
    }

    public void stopMotors() {
        for (MotorController controller : controllers) {
            controller.stopMotor();
        }
    }

    public boolean allMotorsConnected() {
        for (MotorController controller : controllers) {
            if (!controller.isConnected()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns true when any motor controller in the group reports a stalled condition.
     */
    public boolean isStalling() {
        for (MotorController controller : controllers) {
            if (controller != null && controller.isStalled()) {
                return true;
            }
        }
        return false;
    }

    public void update() {
        for (MotorController controller : controllers) {
            controller.update();
        }
    }

    public EncoderGroup getEncoderGroup() {
        return encoders;
    }

    public double getAverageTemperatureCelsius() {
        return calculateAverageTemperatureCelsius();
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
        summaryNode.putBoolean("allConnected", allMotorsConnected());
        summaryNode.putDouble("avgTempC", calculateAverageTemperatureCelsius());

        if (encoders != null && nt.enabled(RobotNetworkTables.Flag.HW_ENCODER_TUNING_WIDGETS)) {
            encoders.networkTables(encodersNode);
        }

        if (nt.enabled(RobotNetworkTables.Flag.MOTOR_GROUP_PER_MOTOR)) {
            for (int i = 0; i < controllers.length; i++) {
                MotorController motor = controllers[i];
                if (motor == null) {
                    continue;
                }
                if (motorNodes != null && i < motorNodes.length && motorNodes[i] != null) {
                    motor.networkTables(motorNodes[i]);
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
        encodersNode = node.child("Encoders");
        RobotNetworkTables.Node motorsRoot = node.child("Motors");
        motorNodes = new RobotNetworkTables.Node[controllers.length];
        for (int i = 0; i < controllers.length; i++) {
            String key = controllerTopicKeys != null && i < controllerTopicKeys.length
                    ? controllerTopicKeys[i]
                    : "motor";
            motorNodes[i] = motorsRoot.child(key);
        }
    }

    private static String[] buildTopicKeys(MotorController[] controllers) {
        if (controllers == null || controllers.length == 0) {
            return new String[0];
        }
        String[] keys = new String[controllers.length];
        for (int i = 0; i < controllers.length; i++) {
            MotorController controller = controllers[i];
            keys[i] = sanitizeTopicKey(controller != null ? controller.getName() : null, "motor");
        }
        return keys;
    }

    private static String sanitizeTopicKey(String raw, String fallback) {
        if (raw == null || raw.isBlank()) {
            return fallback;
        }
        return raw.replace('\\', '_').replace('/', '_');
    }

    private double calculateAverageTemperatureCelsius() {
        if (controllers.length == 0) {
            return 0.0;
        }
        double sum = 0.0;
        int count = 0;
        for (MotorController controller : controllers) {
            sum += controller.getTemperatureCelsius();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }

    public static final class OutputSection extends MotorSectionSupport.OutputSectionBase<OutputSection> {
        private final MotorControllerGroup owner;

        private OutputSection(MotorControllerGroup owner) {
            this.owner = owner;
        }

        @Override
        protected OutputSection self() {
            return this;
        }

        @Override
        protected void applySpeed(double speed) {
            owner.setSpeed(speed);
        }

        @Override
        protected void applyVoltage(double voltage) {
            owner.setVoltage(voltage);
        }

        @Override
        protected void applyPosition(double position) {
            owner.setPosition(position);
        }

        @Override
        protected void applyVelocity(double rotationsPerSecond) {
            owner.setVelocity(rotationsPerSecond);
        }

        @Override
        protected void applyStop() {
            owner.stopMotors();
        }
    }

    public static final class ConfigSection extends MotorSectionSupport.ConfigSectionBase<ConfigSection> {
        private final MotorControllerGroup owner;

        private ConfigSection(MotorControllerGroup owner) {
            this.owner = owner;
        }

        @Override
        protected ConfigSection self() {
            return this;
        }

        @Override
        protected void applyNeutralMode(MotorNeutralMode mode) {
            owner.setNeutralMode(mode);
        }

        @Override
        protected void applyCurrentLimit(double amps) {
            owner.setCurrentLimit(amps);
        }

        @Override
        protected void applyPid(PIDController pid) {
            owner.setPid(pid);
        }
    }
}
