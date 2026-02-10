package ca.frc6390.athena.hardware.motor;

import java.util.Arrays;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.hardware.encoder.EncoderGroup;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.dashboard.ShuffleboardControls;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Groups multiple {@link MotorController}s together to preserve the legacy group API while routing
 * through the new vendordep-backed hardware layer.
 */
public class MotorControllerGroup implements RobotSendableDevice {
    private final MotorController[] controllers;
    private EncoderGroup encoders;
    private double shuffleboardPeriodSecondsOverride = Double.NaN;
    private ShuffleboardLayout lastShuffleboardLayoutComp;
    private ShuffleboardLayout lastShuffleboardLayoutDebug;

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
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        if (level == SendableLevel.DEBUG) {
            if (layout == lastShuffleboardLayoutDebug) {
                return layout;
            }
            lastShuffleboardLayoutDebug = layout;
            lastShuffleboardLayoutComp = layout;
        } else {
            if (layout == lastShuffleboardLayoutComp) {
                return layout;
            }
            lastShuffleboardLayoutComp = layout;
        }

        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        ShuffleboardLayout summary = layout.getLayout("Summary", BuiltInLayouts.kList);
        summary.addBoolean("All Connected", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::allMotorsCachedConnected, period));
        summary.addDouble("Average Temp (C)", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getAverageCachedTemperatureCelsius, period));
        if (encoders != null && level.equals(SendableLevel.DEBUG)) {
            encoders.shuffleboard(layout.getLayout("Encoders", BuiltInLayouts.kList), level);
        }
        if (ShuffleboardControls.enabled(ShuffleboardControls.Flag.MOTOR_GROUP_PER_MOTOR)) {
            Arrays.stream(controllers).forEach(motorController ->
                    motorController.shuffleboard(layout.getLayout(motorController.getName(), BuiltInLayouts.kList), level));
        }
        return layout;
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
