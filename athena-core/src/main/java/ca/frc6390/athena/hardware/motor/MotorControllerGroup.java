package ca.frc6390.athena.hardware.motor;

import java.util.Arrays;

import ca.frc6390.athena.core.RobotSendableSystem.RobotSendableDevice;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.hardware.encoder.EncoderGroup;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
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
    private double shuffleboardPeriodSeconds = ca.frc6390.athena.core.RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();

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
        Arrays.stream(controllers).forEach(motorController -> motorController.setNeutralMode(mode));
        return this;
    }

    public MotorControllerGroup setEncoders(EncoderGroup encoders) {
        this.encoders = encoders;
        return this;
    }

    public void setCurrentLimit(double currentLimit) {
        Arrays.stream(controllers).forEach(motorController -> motorController.setCurrentLimit(currentLimit));
    }

    public void setPosition(double position) {
        Arrays.stream(controllers).forEach(motorController -> motorController.setPosition(position));
    }

    public void setPid(PIDController pid) {
        Arrays.stream(controllers).forEach(motorController -> motorController.setPid(pid));
    }

    public void setPid(double p, double i, double d) {
        setPid(new PIDController(p, i, d));
    }

    public void setSpeed(double speed) {
        Arrays.stream(controllers).forEach(motorController -> motorController.setSpeed(speed));
    }

    public void setVoltage(double voltage) {
        Arrays.stream(controllers).forEach(motorController -> motorController.setVoltage(voltage));
    }

    public void stopMotors() {
        Arrays.stream(controllers).forEach(MotorController::stopMotor);
    }

    public boolean allMotorsConnected() {
        return Arrays.stream(controllers).allMatch(MotorController::isConnected);
    }

    public void update() {
        Arrays.stream(controllers).forEach(MotorController::update);
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

    public EncoderGroup getEncoderGroup() {
        return encoders;
    }

    public double getAverageTemperatureCelsius() {
        return Arrays.stream(controllers)
                .mapToDouble(MotorController::getTemperatureCelsius)
                .average()
                .orElse(0.0);
    }

    @Override
    public ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level) {
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        ShuffleboardLayout summary = layout.getLayout("Summary", BuiltInLayouts.kList);
        summary.addBoolean("All Connected", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::allMotorsCachedConnected, period));
        summary.addDouble("Average Temp (C)", ca.frc6390.athena.core.RobotSendableSystem.rateLimit(this::getAverageCachedTemperatureCelsius, period));
        if (encoders != null && level.equals(SendableLevel.DEBUG)) {
            encoders.shuffleboard(layout.getLayout("Encoders", BuiltInLayouts.kList), level);
        }
        Arrays.stream(controllers).forEach(motorController ->
                motorController.shuffleboard(layout.getLayout(motorController.getName(), BuiltInLayouts.kList), level));
        return layout;
    }

    public boolean allMotorsCachedConnected() {
        return Arrays.stream(controllers).allMatch(MotorController::isCachedConnected);
    }

    private double getAverageCachedTemperatureCelsius() {
        return Arrays.stream(controllers)
                .mapToDouble(MotorController::getCachedTemperatureCelsius)
                .average()
                .orElse(0.0);
    }
}
