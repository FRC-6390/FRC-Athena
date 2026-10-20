package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import java.util.Objects;

/**
 * CTRE Pigeon2 IMU handle backed by Phoenix 6.
 */
public final class CtrePigeon2Handle implements ImuHandle, AutoCloseable {
    private final ImuDevice device;
    private final Pigeon2Controller controller;
    private boolean inputsFresh;
    private double yawDegrees;
    private double pitchDegrees;
    private double rollDegrees;
    private double yawRateDegreesPerSecond;
    private double pitchRateDegreesPerSecond;
    private double rollRateDegreesPerSecond;
    private double linearAccelerationXG;
    private double linearAccelerationYG;
    private double linearAccelerationZG;
    private volatile double lastUpdateSeconds = Double.NaN;

    /**
     * Creates a CTRE Pigeon2 handle using a real Phoenix 6 Pigeon2.
     *
     * @param device IMU declaration
     */
    public CtrePigeon2Handle(ImuDevice device) {
        this(device, new PhoenixPigeon2Controller(device));
    }

    CtrePigeon2Handle(ImuDevice device, Pigeon2Controller controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public ImuDevice device() {
        return device;
    }

    @Override
    public double yawDegrees() {
        ensureInputsFresh();
        return yawDegrees;
    }

    @Override
    public void refreshInputs() {
        refreshInputs(CtreStatusBatch.UNSHARED);
    }

    @Override
    public void refreshInputs(long refreshPass) {
        // The connection check reads the frame timestamps of this batch, so refresh first.
        controller.refreshSignals(refreshPass);
        if (!controller.isConnected()) {
            throw new IllegalStateException("CTRE IMU is disconnected: " + device.defaultName());
        }
        yawDegrees = controller.yawDegrees();
        pitchDegrees = controller.pitchDegrees();
        rollDegrees = controller.rollDegrees();
        yawRateDegreesPerSecond = controller.yawRateDegreesPerSecond();
        pitchRateDegreesPerSecond = controller.pitchRateDegreesPerSecond();
        rollRateDegreesPerSecond = controller.rollRateDegreesPerSecond();
        linearAccelerationXG = controller.linearAccelerationXG();
        linearAccelerationYG = controller.linearAccelerationYG();
        linearAccelerationZG = controller.linearAccelerationZG();
        lastUpdateSeconds = System.nanoTime() * 1.0e-9;
        inputsFresh = true;
    }

    /**
     * Returns pitch in degrees.
     *
     * @return pitch degrees
     */
    @Override
    public double pitchDegrees() {
        ensureInputsFresh();
        return pitchDegrees;
    }

    /**
     * Returns roll in degrees.
     *
     * @return roll degrees
     */
    @Override
    public double rollDegrees() {
        ensureInputsFresh();
        return rollDegrees;
    }

    @Override
    public double yawRateDegreesPerSecond() {
        ensureInputsFresh();
        return yawRateDegreesPerSecond;
    }

    @Override
    public double pitchRateDegreesPerSecond() {
        ensureInputsFresh();
        return pitchRateDegreesPerSecond;
    }

    @Override
    public double rollRateDegreesPerSecond() {
        ensureInputsFresh();
        return rollRateDegreesPerSecond;
    }

    @Override
    public double linearAccelerationXG() {
        ensureInputsFresh();
        return linearAccelerationXG;
    }

    @Override
    public double linearAccelerationYG() {
        ensureInputsFresh();
        return linearAccelerationYG;
    }

    @Override
    public double linearAccelerationZG() {
        ensureInputsFresh();
        return linearAccelerationZG;
    }

    @Override public boolean isConnected() { return controller.isConnected(); }
    @Override public boolean isCalibrating() { return controller.isCalibrating(); }
    @Override public double lastUpdateSeconds() { return lastUpdateSeconds; }

    @Override
    public void setYawDegrees(double yawDegrees) {
        if (!controller.setYawDegrees(yawDegrees)) {
            throw new IllegalStateException("Failed to set yaw for " + device.defaultName());
        }
        inputsFresh = false;
    }

    @Override
    public void zeroYaw() {
        controller.setYawDegrees(0.0);
        inputsFresh = false;
    }

    @Override
    public void reset() {
        controller.reset();
        inputsFresh = false;
    }

    @Override
    public void close() {
        controller.close();
    }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
    }

    interface Pigeon2Controller extends AutoCloseable {
        default boolean isConnected() { return true; }

        /** Refreshes all cached status signals; the batch refreshes the bus unless this pass already did. */
        default void refreshSignals() { }

        default void refreshSignals(long refreshPass) { refreshSignals(); }

        double yawDegrees();

        double pitchDegrees();

        double rollDegrees();

        double yawRateDegreesPerSecond();

        default double pitchRateDegreesPerSecond() { return 0.0; }

        default double rollRateDegreesPerSecond() { return 0.0; }

        double linearAccelerationXG();

        double linearAccelerationYG();

        double linearAccelerationZG();

        default boolean isCalibrating() { return false; }

        boolean setYawDegrees(double yawDegrees);

        void reset();

        @Override
        void close();
    }

    private static final class PhoenixPigeon2Controller implements Pigeon2Controller {
        private final Pigeon2 pigeon;
        private final StatusSignal<Angle> yaw;
        private final StatusSignal<Angle> pitch;
        private final StatusSignal<Angle> roll;
        private final StatusSignal<AngularVelocity> yawRate;
        private final StatusSignal<AngularVelocity> pitchRate;
        private final StatusSignal<AngularVelocity> rollRate;
        private final StatusSignal<LinearAcceleration> accelerationX;
        private final StatusSignal<LinearAcceleration> accelerationY;
        private final StatusSignal<LinearAcceleration> accelerationZ;
        private final StatusSignal<Double> gravityX;
        private final StatusSignal<Double> gravityY;
        private final StatusSignal<Double> gravityZ;
        private final CtreStatusBatch batch;
        private final CtreStatusBatch.Member member;
        private final SignalFreshness freshness;

        private PhoenixPigeon2Controller(ImuDevice device) {
            pigeon = new Pigeon2(device.id(), new CANBus(device.canbus()));
            yaw = pigeon.getYaw();
            pitch = pigeon.getPitch();
            roll = pigeon.getRoll();
            yawRate = pigeon.getAngularVelocityZDevice();
            pitchRate = pigeon.getAngularVelocityYDevice();
            rollRate = pigeon.getAngularVelocityXDevice();
            accelerationX = pigeon.getAccelerationX();
            accelerationY = pigeon.getAccelerationY();
            accelerationZ = pigeon.getAccelerationZ();
            gravityX = pigeon.getGravityVectorX();
            gravityY = pigeon.getGravityVectorY();
            gravityZ = pigeon.getGravityVectorZ();
            batch = CtreStatusBatch.forBus(device.canbus());
            member = batch.join(
                yaw, pitch, roll, yawRate, pitchRate, rollRate,
                accelerationX, accelerationY, accelerationZ, gravityX, gravityY, gravityZ);
            freshness = new SignalFreshness(yaw);
        }

        @Override public boolean isConnected() { return freshness.connected(); }

        @Override
        public void refreshSignals() {
            refreshSignals(CtreStatusBatch.UNSHARED);
        }

        @Override
        public void refreshSignals(long refreshPass) {
            batch.refresh(refreshPass);
        }

        // Phoenix reports these signals in degrees, degrees/second and g, so the raw doubles need no conversion.
        @Override
        public double yawDegrees() {
            return yaw.getValueAsDouble();
        }

        @Override
        public double pitchDegrees() {
            return pitch.getValueAsDouble();
        }

        @Override
        public double rollDegrees() {
            return roll.getValueAsDouble();
        }

        @Override
        public double yawRateDegreesPerSecond() {
            return yawRate.getValueAsDouble();
        }

        @Override
        public double pitchRateDegreesPerSecond() {
            return pitchRate.getValueAsDouble();
        }

        @Override
        public double rollRateDegreesPerSecond() {
            return rollRate.getValueAsDouble();
        }

        @Override
        public double linearAccelerationXG() {
            return accelerationX.getValueAsDouble() - gravityX.getValueAsDouble();
        }

        @Override
        public double linearAccelerationYG() {
            return accelerationY.getValueAsDouble() - gravityY.getValueAsDouble();
        }

        @Override
        public double linearAccelerationZG() {
            return accelerationZ.getValueAsDouble() - gravityZ.getValueAsDouble();
        }

        @Override
        public boolean setYawDegrees(double yawDegrees) {
            return pigeon.setYaw(yawDegrees).isOK();
        }

        @Override
        public void reset() {
            pigeon.reset();
        }

        @Override
        public void close() {
            batch.leave(member);
            pigeon.close();
        }
    }
}
