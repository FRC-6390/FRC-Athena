package ca.frc6390.athena.hardware.motor;

import java.util.Arrays;

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
    private EncoderGroup encoders;

    public MotorControllerGroup(MotorController... controllers) {
        this.controllers = controllers;
        this.encoders = EncoderGroup.fromMotorGroup(this);
    }

    public MotorControllerGroup(MotorController leftMotor, MotorController rightMotor) {
        this(new MotorController[] {leftMotor, rightMotor});
    }

    public static MotorControllerGroup fromConfigs(MotorControllerConfig... configs) {
        return new MotorControllerGroup(
                Arrays.stream(configs).map(HardwareFactories::motor).toArray(MotorController[]::new));
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

    public void update() {
        for (MotorController controller : controllers) {
            controller.update();
        }
    }

    public EncoderGroup getEncoderGroup() {
        return encoders;
    }

    public double getAverageTemperatureCelsius() {
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
        summary.putBoolean("allConnected", allMotorsCachedConnected());
        summary.putDouble("avgTempC", getAverageCachedTemperatureCelsius());

        if (encoders != null && node.robot().enabled(RobotNetworkTables.Flag.HW_ENCODER_TUNING_WIDGETS)) {
            encoders.networkTables(node.child("Encoders"));
        }

        if (node.robot().enabled(RobotNetworkTables.Flag.MOTOR_GROUP_PER_MOTOR)) {
            RobotNetworkTables.Node motorsNode = node.child("Motors");
            for (MotorController motor : controllers) {
                if (motor == null) {
                    continue;
                }
                String key = motor.getName();
                key = key != null ? key.replace('\\', '_').replace('/', '_') : "motor";
                motor.networkTables(motorsNode.child(key));
            }
        }
    }

    public boolean allMotorsCachedConnected() {
        for (MotorController controller : controllers) {
            if (!controller.isCachedConnected()) {
                return false;
            }
        }
        return true;
    }

    private double getAverageCachedTemperatureCelsius() {
        if (controllers.length == 0) {
            return 0.0;
        }
        double sum = 0.0;
        int count = 0;
        for (MotorController controller : controllers) {
            sum += controller.getCachedTemperatureCelsius();
            count++;
        }
        return count > 0 ? sum / count : 0.0;
    }
}
