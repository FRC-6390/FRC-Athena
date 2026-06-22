package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.hardware.spec.NeutralMode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * CTRE TalonFX-family motor device backed by Phoenix 6.
 */
public final class CtreMotorDevice implements MotorDevice {
    private final MotorSpec spec;
    private final CtreMotorOptions options;
    private final TalonController controller;

    /**
     * Creates a CTRE motor device using a real Phoenix 6 TalonFX-family controller.
     *
     * @param spec normalized motor spec
     * @param options CTRE options
     */
    public CtreMotorDevice(MotorSpec spec, CtreMotorOptions options) {
        this(spec, options, new PhoenixTalonController(spec));
    }

    CtreMotorDevice(MotorSpec spec, CtreMotorOptions options, TalonController controller) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.options = options == null ? new CtreMotorOptions() : options;
        this.controller = Objects.requireNonNull(controller, "controller");
        this.controller.setNeutralMode(spec.neutralMode() == NeutralMode.BRAKE);
    }

    @Override
    public MotorSpec spec() {
        return spec;
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
        return controller.positionRotations();
    }

    @Override
    public double integratedVelocityRotationsPerSecond() {
        return controller.velocityRotationsPerSecond();
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

        private PhoenixTalonController(MotorSpec spec) {
            talon = new TalonFX(spec.id(), new CANBus(spec.canbus()));
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
