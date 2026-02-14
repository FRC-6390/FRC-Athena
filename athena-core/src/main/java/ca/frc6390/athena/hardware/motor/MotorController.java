package ca.frc6390.athena.hardware.motor;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.Consumer;

/**
 * Vendor-agnostic motor controller interface for the new vendordep system.
 */
public interface MotorController extends RobotSendableSystem.RobotSendableDevice {
    // Identity
    int getId();

    String getCanbus();

    MotorControllerType getType();

    // Outputs
    /**
     */
    void setSpeed(double percent);

    /**
     */
    void setVoltage(double volts);

    /**
     */
    void setCurrentLimit(double amps);

    /**
     */
    void setPosition(double rotations);

    /**
     */
    void setVelocity(double rotationsPerSecond);

    /**
     */
    void setNeutralMode(MotorNeutralMode mode);

    /**
     */
    void setPid(PIDController pid);

    boolean isConnected();

    double getTemperatureCelsius();

    default boolean isConnected(boolean poll) {
        if (poll) {
            update();
        }
        return isConnected();
    }

    default double getTemperatureCelsius(boolean poll) {
        if (poll) {
            update();
        }
        return getTemperatureCelsius();
    }

    Encoder getEncoder();

    // Expanded API to match legacy usage
    default boolean isInverted() { return false; }

    /**
     */
    default void setInverted(boolean inverted) {}

    default double getCurrentLimit() { return 0.0; }

    default MotorNeutralMode getNeutralMode() { return MotorNeutralMode.Coast; }

    default String getName() { return getCanbus() + "\\" + getId() + "\\" + getType().getKey(); }

    /**
     */
    default void stopMotor() { setSpeed(0); }

    default void update() {}

    default MotorControllerConfig getConfig() { return null; }

    /**
     * Sectioned runtime API for output interactions on an already-built motor controller.
     */
    default MotorController output(Consumer<OutputSection> section) {
        if (section != null) {
            section.accept(new OutputSection(this));
        }
        return this;
    }

    /**
     * Convenience accessor for non-lambda output chaining.
     */
    default OutputSection output() {
        return new OutputSection(this);
    }

    /**
     * Sectioned config API for hardware/tuning interactions.
     */
    default MotorController config(Consumer<HardwareSection> section) {
        if (section != null) {
            section.accept(new HardwareSection(this));
        }
        return this;
    }

    /**
     * Convenience accessor for non-lambda config chaining.
     */
    default HardwareSection config() {
        return new HardwareSection(this);
    }

    /**
     * Structured read API: {@code motor.telemetry().temperatureCelsius()}.
     */
    default TelemetryView telemetry() {
        return new TelemetryView(this, false);
    }

    default IdentityView identity() {
        return new IdentityView(this);
    }

    final class OutputSection extends MotorSectionSupport.OutputSectionBase<OutputSection> {
        private final MotorController owner;

        OutputSection(MotorController owner) {
            this.owner = owner;
        }

        @Override
        protected OutputSection self() {
            return this;
        }

        @Override
        protected void applySpeed(double percent) {
            owner.setSpeed(percent);
        }

        @Override
        protected void applyVoltage(double volts) {
            owner.setVoltage(volts);
        }

        @Override
        protected void applyPosition(double rotations) {
            owner.setPosition(rotations);
        }

        @Override
        protected void applyVelocity(double rotationsPerSecond) {
            owner.setVelocity(rotationsPerSecond);
        }

        @Override
        protected void applyStop() {
            owner.stopMotor();
        }
    }

    final class HardwareSection extends MotorSectionSupport.ConfigSectionBase<HardwareSection> {
        private final MotorController owner;

        HardwareSection(MotorController owner) {
            this.owner = owner;
        }

        @Override
        protected HardwareSection self() {
            return this;
        }

        @Override
        protected void applyCurrentLimit(double amps) {
            owner.setCurrentLimit(amps);
        }

        @Override
        protected void applyNeutralMode(MotorNeutralMode mode) {
            owner.setNeutralMode(mode);
        }

        @Override
        protected void applyPid(PIDController pid) {
            owner.setPid(pid);
        }

        public HardwareSection inverted(boolean inverted) {
            owner.setInverted(inverted);
            return this;
        }
    }

    final class TelemetryView {
        private final MotorController owner;
        private final boolean poll;

        TelemetryView(MotorController owner, boolean poll) {
            this.owner = owner;
            this.poll = poll;
        }

        public TelemetryView poll() {
            return new TelemetryView(owner, true);
        }

        public boolean connected() {
            return owner.isConnected(poll);
        }

        public double temperatureCelsius() {
            return owner.getTemperatureCelsius(poll);
        }

        public boolean inverted() {
            return owner.isInverted();
        }

        public double currentLimit() {
            return owner.getCurrentLimit();
        }

        public MotorNeutralMode neutralMode() {
            return owner.getNeutralMode();
        }
    }

    final class IdentityView {
        private final MotorController owner;

        IdentityView(MotorController owner) {
            this.owner = owner;
        }

        public int canId() {
            return owner.getId();
        }

        public String canbus() {
            return owner.getCanbus();
        }

        public MotorControllerType type() {
            return owner.getType();
        }

        public String typeKey() {
            MotorControllerType type = owner.getType();
            return type != null ? type.getKey() : "unknown";
        }

        public String name() {
            return owner.getName();
        }
    }

    @Override
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        MotorControllerType type = getType();
        node.putDouble("canId", getId());
        node.putString("canbus", getCanbus());
        node.putString("type", type != null ? type.getKey() : "unknown");
        node.putBoolean("connected", isConnected());
        node.putDouble("tempC", getTemperatureCelsius());
        node.putString("neutralMode", getNeutralMode().name());

        if (nt.enabled(RobotNetworkTables.Flag.HW_MOTOR_TUNING_WIDGETS)) {
            node.putDouble("currentLimitA", getCurrentLimit());
            node.putBoolean("inverted", isInverted());
            node.putBoolean("brakeMode", getNeutralMode() == MotorNeutralMode.Brake);

            Encoder encoder = getEncoder();
            if (encoder != null) {
                encoder.networkTables(node.child("Encoder"));
            }
        }

        return node;
    }
}
