package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.hardware.backend.MotorDevice;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.hardware.spec.NeutralMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.Objects;

/**
 * REV Spark motor device backed by REVLib.
 */
public final class RevMotorDevice implements MotorDevice {
    private final MotorSpec spec;
    private final RevMotorOptions options;
    private final SparkController controller;

    /**
     * Creates a REV motor device using real REVLib Spark controllers.
     *
     * @param spec normalized motor spec
     * @param options REV options
     */
    public RevMotorDevice(MotorSpec spec, RevMotorOptions options) {
        this(spec, options, createController(spec));
    }

    RevMotorDevice(MotorSpec spec, RevMotorOptions options, SparkController controller) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.options = options == null ? new RevMotorOptions() : options;
        this.controller = Objects.requireNonNull(controller, "controller");
        this.controller.configure(spec, this.options);
    }

    @Override
    public MotorSpec spec() {
        return spec;
    }

    /**
     * Returns REV-specific options.
     *
     * @return options
     */
    public RevMotorOptions options() {
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

    @Override
    public double absolutePositionRotations() {
        return controller.absolutePositionRotations();
    }

    @Override
    public double absoluteVelocityRotationsPerSecond() {
        return controller.absoluteVelocityRotationsPerSecond();
    }

    private static SparkController createController(MotorSpec spec) {
        boolean brushless = spec.kind() == AthenaMotor.SPARK_MAX_BRUSHLESS
                || spec.kind() == AthenaMotor.SPARK_FLEX_BRUSHLESS
                || spec.kind().key().endsWith("brushless");
        MotorType motorType = brushless ? MotorType.kBrushless : MotorType.kBrushed;
        SparkBase spark = spec.kind() == AthenaMotor.SPARK_FLEX_BRUSHLESS
                || spec.kind() == AthenaMotor.SPARK_FLEX_BRUSHED
                || spec.kind().key().startsWith("rev:spark-flex")
                        ? new SparkFlex(spec.id(), motorType)
                        : new SparkMax(spec.id(), motorType);
        return new RevSparkController(spark, spec.kind().key().startsWith("rev:spark-flex"));
    }

    private static double clamp(double value) {
        double finite = finiteOrZero(value);
        return Math.max(-1.0, Math.min(1.0, finite));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    interface SparkController {
        void configure(MotorSpec spec, RevMotorOptions options);

        void setPercent(double percent);

        void setVoltage(double volts);

        void setPositionTarget(double rotations);

        void setVelocityTarget(double rotationsPerSecond);

        void stop();

        double positionRotations();

        double velocityRotationsPerSecond();

        double absolutePositionRotations();

        double absoluteVelocityRotationsPerSecond();
    }

    private static final class RevSparkController implements SparkController {
        private final SparkBase spark;
        private final boolean flex;

        private RevSparkController(SparkBase spark, boolean flex) {
            this.spark = spark;
            this.flex = flex;
        }

        @Override
        public void configure(MotorSpec spec, RevMotorOptions options) {
            SparkBaseConfig config = flex ? new SparkFlexConfig() : new SparkMaxConfig();
            config.idleMode(spec.neutralMode() == NeutralMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
            int currentLimit = options.smartCurrentLimitAmps() > 0
                    ? options.smartCurrentLimitAmps()
                    : spec.currentLimitAmps();
            if (currentLimit > 0) {
                config.smartCurrentLimit(currentLimit);
            }
            if (options.openLoopRampSeconds() > 0.0) {
                config.openLoopRampRate(options.openLoopRampSeconds());
            }
            if (options.closedLoopRampSeconds() > 0.0) {
                config.closedLoopRampRate(options.closedLoopRampSeconds());
            }
            spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        @Override
        public void setPercent(double percent) {
            spark.set(percent);
        }

        @Override
        public void setVoltage(double volts) {
            spark.setVoltage(volts);
        }

        @Override
        public void setPositionTarget(double rotations) {
            spark.getClosedLoopController().setSetpoint(rotations, ControlType.kPosition);
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            spark.getClosedLoopController().setSetpoint(rotationsPerSecond * 60.0, ControlType.kVelocity);
        }

        @Override
        public void stop() {
            spark.stopMotor();
        }

        @Override
        public double positionRotations() {
            return encoder().getPosition();
        }

        @Override
        public double velocityRotationsPerSecond() {
            return encoder().getVelocity() / 60.0;
        }

        @Override
        public double absolutePositionRotations() {
            return absoluteEncoder().getPosition();
        }

        @Override
        public double absoluteVelocityRotationsPerSecond() {
            return absoluteEncoder().getVelocity() / 60.0;
        }

        private RelativeEncoder encoder() {
            return spark.getEncoder();
        }

        private AbsoluteEncoder absoluteEncoder() {
            return spark.getAbsoluteEncoder();
        }
    }
}
