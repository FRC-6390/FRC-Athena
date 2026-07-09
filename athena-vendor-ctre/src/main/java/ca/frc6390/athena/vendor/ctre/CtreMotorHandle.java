package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * CTRE TalonFX-family motor handle backed by Phoenix 6.
 */
public final class CtreMotorHandle implements MotorHandle {
    private final MotorDevice device;
    private final CtreMotorOptions options;
    private final TalonController controller;
    private boolean activated;
    private boolean inputsFresh;
    private double positionRotations;
    private double velocityRotationsPerSecond;

    /**
     * Creates a CTRE motor handle using a real Phoenix 6 TalonFX-family controller.
     *
     * @param device motor declaration
     * @param options CTRE options
     */
    public CtreMotorHandle(MotorDevice device, CtreMotorOptions options) {
        this(device, options, new PhoenixTalonController(device));
    }

    CtreMotorHandle(MotorDevice device, CtreMotorOptions options, TalonController controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.options = options == null ? new CtreMotorOptions() : options;
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public MotorDevice device() {
        return device;
    }

    /**
     * Returns CTRE-specific options.
     *
     * @return options
     */
    public CtreMotorOptions options() {
        return options;
    }

    @Override
    public void activate() {
        if (!activated) {
            controller.setNeutralMode(device.neutralMode() == MotorNeutralMode.BRAKE);
            activated = true;
        }
    }

    @Override
    public void refreshInputs() {
        positionRotations = controller.positionRotations();
        velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
        inputsFresh = true;
    }

    @Override
    public void setPercentOutput(double percent) {
        controller.setPercent(clamp(percent));
    }

    @Override
    public void setVoltage(double volts) {
        controller.setVoltage(finiteOrZero(volts));
    }

    @Override
    public void setPositionTargetRotations(double rotations) {
        controller.setPositionTarget(finiteOrZero(rotations));
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        controller.setVelocityTarget(finiteOrZero(rotationsPerSecond));
    }

    @Override
    public void stop() {
        controller.stop();
    }

    @Override
    public double integratedPositionRotations() {
        ensureInputsFresh();
        return positionRotations;
    }

    @Override
    public double integratedVelocityRotationsPerSecond() {
        ensureInputsFresh();
        return velocityRotationsPerSecond;
    }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
    }

    private static double clamp(double value) {
        double finite = finiteOrZero(value);
        return Math.max(-1.0, Math.min(1.0, finite));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    interface TalonController {
        void setPercent(double percent);

        void setVoltage(double volts);

        void setPositionTarget(double rotations);

        void setVelocityTarget(double rotationsPerSecond);

        void stop();

        void setNeutralMode(boolean brake);

        double positionRotations();

        double velocityRotationsPerSecond();
    }

    private static final class PhoenixTalonController implements TalonController {
        private final TalonFX talon;

        private PhoenixTalonController(MotorDevice device) {
            talon = new TalonFX(device.id(), new CANBus(device.canbus()));
        }

        @Override
        public void setPercent(double percent) {
            talon.set(percent);
        }

        @Override
        public void setVoltage(double volts) {
            talon.setVoltage(volts);
        }

        @Override
        public void setPositionTarget(double rotations) {
            talon.setControl(new PositionVoltage(rotations));
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            talon.setControl(new VelocityVoltage(rotationsPerSecond));
        }

        @Override
        public void stop() {
            talon.stopMotor();
        }

        @Override
        public void setNeutralMode(boolean brake) {
            talon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }

        @Override
        public double positionRotations() {
            return talon.getPosition().refresh().getValue().in(Units.Rotations);
        }

        @Override
        public double velocityRotationsPerSecond() {
            return talon.getVelocity().refresh().getValue().in(Units.RotationsPerSecond);
        }
    }
}
