package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

final class MechanismControlLoopScalingTest {

    @Test
    void periodicSimpleFeedforwardUsesVelocitySetpointNotPositionSetpoint() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c.ff("ff", ff -> ff.simple(0.0, 1.0, 0.0)).periodic("ff"));

        MechanismConfig.ControlLoopBinding<Mechanism> binding = cfg.controlLoops().get(0);
        TestMechanism mechanism = new TestMechanism(null);
        mechanism.setpoint(180.0);
        mechanism.control().nudge(5.0);

        TestControlContext context = new TestControlContext(mechanism);
        double output = binding.loop().calculate(context);

        assertEquals(0.0, context.lastFeedforwardVelocity(), 1e-9);
        assertEquals(0.0, output, 1e-9);
    }

    @Test
    void periodicPidUsesConfiguredVelocitySource() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c
                .pid("pid", p -> p.kp(1.0).source(MechanismConfig.InputSource.velocity))
                .periodic("pid"));

        MechanismConfig.ControlLoopBinding<Mechanism> binding = cfg.controlLoops().get(0);
        TestMechanism mechanism = new TestMechanism(null);
        mechanism.positionValue(42.0);
        mechanism.velocityValue(6.5);
        mechanism.setpoint(9.0);
        mechanism.control().nudge(1.0);

        TestControlContext context = new TestControlContext(mechanism);
        double output = binding.loop().calculate(context);

        assertEquals(6.5, context.lastPidMeasurement(), 1e-9);
        assertEquals(10.0, context.lastPidSetpoint(), 1e-9);
        assertEquals(6.5, output, 1e-9);
    }

    @Test
    void periodicPidUsesConfiguredInputSource() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c
                .pid("pid", p -> p.kp(1.0).source(MechanismConfig.InputSource.input("pidMeas")))
                .periodic("pid"));

        MechanismConfig.ControlLoopBinding<Mechanism> binding = cfg.controlLoops().get(0);
        TestMechanism mechanism = new TestMechanism(null);
        mechanism.positionValue(42.0);
        mechanism.velocityValue(6.5);
        mechanism.setpoint(9.0);
        mechanism.control().nudge(1.0);

        TestControlContext context = new TestControlContext(mechanism);
        context.setDoubleInput("pidMeas", 2.75);
        double output = binding.loop().calculate(context);

        assertEquals(2.75, context.lastPidMeasurement(), 1e-9);
        assertEquals(10.0, context.lastPidSetpoint(), 1e-9);
        assertEquals(2.75, output, 1e-9);
    }

    @Test
    void periodicFeedforwardUsesConfiguredInputSource() {
        MechanismConfig<Mechanism> cfg = MechanismConfig.generic();
        cfg.control(c -> c
                .ff("ff", ff -> ff.simple(0.0, 1.0, 0.0).source(MechanismConfig.InputSource.input("ffVel")))
                .periodic("ff"));

        MechanismConfig.ControlLoopBinding<Mechanism> binding = cfg.controlLoops().get(0);
        TestMechanism mechanism = new TestMechanism(null);
        TestControlContext context = new TestControlContext(mechanism);
        context.setDoubleInput("ffVel", 3.25);

        double output = binding.loop().calculate(context);

        assertEquals(3.25, context.lastFeedforwardVelocity(), 1e-9);
        assertEquals(3.25, output, 1e-9);
    }

    @Test
    void encoderUnitConversionsHonorGearRatioAndOffsets() {
        FakeEncoder encoder = new FakeEncoder(
                360.0,   // conversion
                100.0,   // gear ratio
                0.2,     // raw offset
                180.0);  // conversion offset
        TestMechanism mechanism = new TestMechanism(encoder);
        TestControlContext context = new TestControlContext(mechanism);

        double rotations = context.positionToRotations(
                540.0,
                MechanismControlContext.PositionUnit.ENCODER_UNITS);
        double rps = context.velocityToRotationsPerSecond(
                720.0,
                MechanismControlContext.VelocityUnit.ENCODER_UNITS_PER_SEC);

        assertEquals(0.21, rotations, 1e-9);
        assertEquals(0.02, rps, 1e-9);
    }

    private static final class TestMechanism extends Mechanism {
        private double position;
        private double velocity;

        private TestMechanism(Encoder encoder) {
            super(
                    new MotorControllerGroup(),
                    encoder,
                    false,
                    false,
                    new GenericLimitSwitch[0],
                    false,
                    0.02);
        }

        private void positionValue(double value) {
            this.position = value;
        }

        private void velocityValue(double value) {
            this.velocity = value;
        }

        @Override
        public double position() {
            return position;
        }

        @Override
        public double velocity() {
            return velocity;
        }
    }

    private static final class TestControlContext implements MechanismControlContext<Mechanism> {
        private final Mechanism mechanism;
        private final Map<String, Double> doubleInputs;
        private double lastFeedforwardVelocity;
        private double lastPidMeasurement;
        private double lastPidSetpoint;

        private TestControlContext(Mechanism mechanism) {
            this.mechanism = mechanism;
            this.doubleInputs = new HashMap<>();
            this.lastFeedforwardVelocity = Double.NaN;
            this.lastPidMeasurement = Double.NaN;
            this.lastPidSetpoint = Double.NaN;
        }

        private double lastFeedforwardVelocity() {
            return lastFeedforwardVelocity;
        }

        private double lastPidMeasurement() {
            return lastPidMeasurement;
        }

        private double lastPidSetpoint() {
            return lastPidSetpoint;
        }

        private void setDoubleInput(String key, double value) {
            doubleInputs.put(key, value);
        }

        @Override
        public Mechanism mechanism() {
            return mechanism;
        }

        @Override
        public double setpoint() {
            return mechanism.setpoint();
        }

        @Override
        public Enum<?> state() {
            return null;
        }

        @Override
        public boolean input(String key) {
            return false;
        }

        @Override
        public double doubleInput(String key) {
            return doubleInputs.getOrDefault(key, 0.0);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return () -> doubleInputs.getOrDefault(key, 0.0);
        }

        @Override
        public int intVal(String key) {
            return 0;
        }

        @Override
        public IntSupplier intValSupplier(String key) {
            return () -> 0;
        }

        @Override
        public String stringVal(String key) {
            return "";
        }

        @Override
        public Supplier<String> stringValSupplier(String key) {
            return () -> "";
        }

        @Override
        public Pose2d pose2dVal(String key) {
            return Pose2d.kZero;
        }

        @Override
        public Supplier<Pose2d> pose2dValSupplier(String key) {
            return () -> Pose2d.kZero;
        }

        @Override
        public Pose3d pose3dVal(String key) {
            return Pose3d.kZero;
        }

        @Override
        public Supplier<Pose3d> pose3dValSupplier(String key) {
            return () -> Pose3d.kZero;
        }

        @Override
        public <V> V objectInput(String key, Class<V> type) {
            return null;
        }

        @Override
        public <V> Supplier<V> objectInputSupplier(String key, Class<V> type) {
            return () -> null;
        }

        @Override
        public PIDController pid(String name) {
            return new PIDController(0.0, 0.0, 0.0);
        }

        @Override
        public MechanismConfig.BangBangProfile bangBang(String name) {
            return null;
        }

        @Override
        public SimpleMotorFeedforward feedforward(String name) {
            return new SimpleMotorFeedforward(0.0, 0.0, 0.0);
        }

        @Override
        public double feedforwardOut(String name, double velocity) {
            lastFeedforwardVelocity = velocity;
            return velocity;
        }

        @Override
        public double pidOut(String name, double measurement, double setpoint) {
            lastPidMeasurement = measurement;
            lastPidSetpoint = setpoint;
            return measurement;
        }
    }

    private static final class FakeEncoder implements Encoder {
        private final double conversion;
        private final double gearRatio;
        private final double offset;
        private final double conversionOffset;

        private FakeEncoder(double conversion, double gearRatio, double offset, double conversionOffset) {
            this.conversion = conversion;
            this.gearRatio = gearRatio;
            this.offset = offset;
            this.conversionOffset = conversionOffset;
        }

        @Override
        public double getPosition() {
            return 0.0;
        }

        @Override
        public double getVelocity() {
            return 0.0;
        }

        @Override
        public void setPosition(double position) {
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public void setConversion(double conversion) {
        }

        @Override
        public void setOffset(double offset) {
        }

        @Override
        public double getConversion() {
            return conversion;
        }

        @Override
        public double getConversionOffset() {
            return conversionOffset;
        }

        @Override
        public double getOffset() {
            return offset;
        }

        @Override
        public double getGearRatio() {
            return gearRatio;
        }

        @Override
        public EncoderConfig getConfig() {
            return EncoderConfig.create(() -> "test", 1);
        }
    }
}
