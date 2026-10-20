package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.effort.Effort;
import ca.frc6390.athena.mechanism.effort.EffortLimit;
import ca.frc6390.athena.mechanism.effort.LoadResponse;
import ca.frc6390.athena.mechanism.model.FrictionTable;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import edu.wpi.first.units.Units;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/**
 * Closed-loop runs against a DC-motor plant (B5/B6): the predicted motor current must stay within the
 * declared effort, including when the arm is held against a hard stop.
 */
class EffortPlantTest {
    private static final double DT = 0.02;
    private static final int SUBSTEPS = 10;
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 31);
    /** One scope for every rig, so each new rig replaces the previous motor binding. */
    private static final RuntimeScope SCOPE = new RuntimeScope("effort-plant");
    private static final MotorConstants K = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();

    /** One-motor DC plant on the motor shaft: J dw/dt = Kt I - tauExt - b w, with a hard stop at {@code stopMin}. */
    static final class Plant {
        double theta;
        double omega;
        double volts;
        double current;
        double tauExt;
        double stopMin = Double.NaN;
        final double inertia = 0.002;
        final double damping = 0.02;

        void step(double dt) {
            double h = dt / SUBSTEPS;
            for (int i = 0; i < SUBSTEPS; i++) {
                current = (volts - omega / K.raw().speedConstantRadPerSecPerVolt()) / K.resistance().in(Units.Ohms);
                double torque = K.raw().torqueConstantNmPerAmp() * current;
                omega += (torque - tauExt - damping * omega) / inertia * h;
                theta += omega * h / (2.0 * Math.PI);
                if (!Double.isNaN(stopMin) && theta < stopMin) {
                    theta = stopMin;
                    if (omega < 0.0) {
                        omega = 0.0;
                    }
                }
            }
            current = (volts - omega / K.raw().speedConstantRadPerSecPerVolt()) / K.resistance().in(Units.Ohms);
        }
    }

    static final class PlantMotor implements MotorHandle {
        private final MotorDevice device;
        private final Plant plant;

        PlantMotor(MotorDevice device, Plant plant) {
            this.device = device;
            this.plant = plant;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void setVoltage(double volts) {
            plant.volts = volts;
        }

        @Override
        public void setPercentOutput(double percent) {
            plant.volts = percent * 12.0;
        }

        @Override
        public MotorControlCapabilities controlCapabilities() {
            return MotorControlCapabilities.OPEN_LOOP_ONLY;
        }

        @Override
        public double integratedPositionRotations() {
            return plant.theta;
        }

        @Override
        public double integratedVelocityRotationsPerSecond() {
            return plant.omega / (2.0 * Math.PI);
        }

        @Override
        public double appliedVoltage() {
            return plant.volts;
        }

        @Override
        public double statorCurrentAmps() {
            return plant.current;
        }

        @Override
        public double supplyCurrentAmps() {
            return plant.current;
        }
    }

    static final class PlantEncoder implements EncoderHandle {
        private final EncoderDevice device;
        private final Plant plant;

        PlantEncoder(EncoderDevice device, Plant plant) {
            this.device = device;
            this.plant = plant;
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public double positionRotations() {
            return plant.theta;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return plant.omega / (2.0 * Math.PI);
        }
    }

    static final class PlantContext implements ActionContext {
        private final Map<MotorDevice, MotorHandle> motors = new HashMap<>();
        private final Map<EncoderDevice, EncoderHandle> encoders = new HashMap<>();

        PlantContext(MotorDevice motor, EncoderDevice encoder, Plant plant) {
            motors.put(motor, new PlantMotor(motor, plant));
            encoders.put(encoder, new PlantEncoder(encoder, plant));
        }

        @Override
        public MotorHandle motor(MotorDevice ref) {
            return motors.get(ref);
        }

        @Override
        public EncoderHandle encoder(EncoderDevice ref) {
            return encoders.get(ref);
        }
    }

    /** Mechanism that owns one arm binding; the runtime discovers the binding by field. */
    static final class Arm implements Mechanism {
        final ControlBinding<?> angle;

        Arm(ControlBinding<?> angle) {
            this.angle = angle;
        }
    }

    /** Feedforward that matches the plant's back-EMF, so the model's load and blocked checks are exact. */
    static MechanismModel model() {
        return new MechanismModel(MotionKind.ROTARY, 0.0, 0.0, 2.0 * Math.PI / K.raw().speedConstantRadPerSecPerVolt(), 0.0, 0.0,
                0.0, FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, K.raw().torqueConstantNmPerAmp(), K.resistance().in(Units.Ohms), 1,
                List.of(), 1.0, Map.of(), 0.0, 0);
    }

    private static final class Rig {
        final MotorDevice motor = MOTOR;
        final EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 31).measures(Units.Rotations);
        final Plant plant;
        final MechanismRuntime runtime;
        final ControlBinding<?> angle;
        double time;

        Rig(Plant plant, ControlBinding<?> angle, Action action) {
            this.plant = plant;
            this.angle = angle;
            Arm arm = new Arm(angle);
            // Devices read by Actions and checks outside the robot runtime must be bound to the plant.
            motor.bindRuntime(SCOPE, new PlantMotor(motor, plant));
            runtime = MechanismRuntime.of(arm, new PlantContext(motor, encoder, plant));
            runtime.set(action);
        }

        /** Whether the binding's effort runtime is yielding, as of the last cycle. */
        boolean runtimeYielded() {
            for (ControlLoop loop : angle.loops()) {
                if (loop instanceof EffortLimit limit) {
                    return limit.yielded();
                }
            }
            return false;
        }

        /** Runs one control cycle and integrates the plant with the voltage it commanded. */
        void cycle() {
            time += DT;
            runtime.periodic(new MechanismContext(time, 0.0, DT, true, false, false), EventContext.empty());
            plant.step(DT);
        }
    }

    @Test
    void currentLimitHoldsWhileTheArmIsStalledAgainstAHardStop() {
        Plant plant = new Plant();
        ControlBinding<?> angle = Controls.position(MOTOR, Units.Rotations)
                .feedback(plantPosition(plant), plantVelocity(plant))
                .pid(10.0, 0.0, 0.0)
                .effort(Effort.max(Units.Amps.of(5.0)));
        Rig rig = rigFor(plant, angle, angle.position(-0.5));
        rig.plant.stopMin = 0.0;
        double peak = 0.0;
        for (int i = 0; i < 150; i++) {
            rig.cycle();
            peak = Math.max(peak, Math.abs(rig.plant.current));
        }
        assertTrue(peak <= 5.0 + 1e-6, "peak current " + peak);
        assertTrue(peak > 4.9, "the limit should be reached, peak " + peak);
        assertEquals(0.0, rig.plant.theta, 1e-9, "the stop holds the arm");
    }

    @Test
    void withoutAnEffortLimitTheSameStallDrawsFarMoreCurrent() {
        Plant plant = new Plant();
        ControlBinding<?> angle = Controls.position(MOTOR, Units.Rotations)
                .feedback(plantPosition(plant), plantVelocity(plant))
                .pid(10.0, 0.0, 0.0);
        Rig rig = rigFor(plant, angle, angle.position(-0.5));
        rig.plant.stopMin = 0.0;
        double peak = 0.0;
        for (int i = 0; i < 150; i++) {
            rig.cycle();
            peak = Math.max(peak, Math.abs(rig.plant.current));
        }
        assertTrue(peak > 50.0, "unlimited stall current " + peak);
    }

    @Test
    void currentLimitHoldsDuringAcceleration() {
        Plant plant = new Plant();
        ControlBinding<?> angle = Controls.position(MOTOR, Units.Rotations)
                .feedback(plantPosition(plant), plantVelocity(plant))
                .pid(10.0, 0.0, 0.0)
                .effort(Effort.max(Units.Amps.of(5.0)));
        Rig rig = rigFor(plant, angle, angle.position(3.0));
        double peak = 0.0;
        double peakSpeed = 0.0;
        for (int i = 0; i < 100; i++) {
            rig.cycle();
            peak = Math.max(peak, Math.abs(rig.plant.current));
            peakSpeed = Math.max(peakSpeed, rig.plant.omega);
        }
        assertTrue(peak <= 5.0 + 1e-6, "accelerating current " + peak);
        // Viscous damping caps the speed at Kt I / b (about 4.8 rad/s here), so the arm reaches that.
        assertTrue(peakSpeed > 4.0, "the arm should accelerate, peak speed " + peakSpeed);
    }

    @Test
    void torqueCeilingTranslatesToTheSameCurrentLimitAsAmps() {
        double torque = K.raw().torqueConstantNmPerAmp() * 5.0;
        Plant plant = new Plant();
        ControlBinding<?> angle = Controls.position(MOTOR, Units.Rotations)
                .feedback(plantPosition(plant), plantVelocity(plant))
                .pid(10.0, 0.0, 0.0)
                .effort(Effort.max(Units.NewtonMeters.of(torque)).gearing(1.0));
        Rig rig = rigFor(plant, angle, angle.position(-0.5));
        rig.plant.stopMin = 0.0;
        double peak = 0.0;
        for (int i = 0; i < 150; i++) {
            rig.cycle();
            peak = Math.max(peak, Math.abs(rig.plant.current));
        }
        assertTrue(peak <= 5.0 + 1e-6, "torque-limited current " + peak);
    }

    /**
     * Stow against a hard stop, then hold gently: the composition
     * {@code position(x).effort(e).until(blocked()).then(position(x).effort(smaller))} runs against a real
     * plant. A stall under an effort ceiling draws I R volts, so the blocked threshold is set below that.
     */
    @Test
    void stowAgainstAHardStopThenHoldGently() {
        Plant plant = new Plant();
        ControlBinding<?> angle = Controls.position(MOTOR, Units.Rotations)
                .feedback(plantPosition(plant), plantVelocity(plant))
                .pid(10.0, 0.0, 0.0)
                .model(model());
        double stow = -0.2;
        Action action = angle.position(stow).effort(Units.Amps.of(8.0))
                .until(angle.blocked().threshold(0.1).forTime(Units.Seconds.of(0.0)))
                .then(angle.position(stow).effort(Units.Amps.of(1.0)));
        Rig rig = rigFor(plant, angle, action);
        rig.plant.theta = 0.4;
        rig.plant.stopMin = 0.0;

        // Phase 1: the arm travels to the stop and presses with the stow effort (8 A) until blocked() fires.
        // The clamp uses the speed measured at the start of a cycle. The impact with the stop removes the
        // back-EMF inside the cycle, so the first cycles after contact can exceed the limit. Peaks are taken
        // after the impact (the arm arrives within about 0.6 s).
        double peakStow = 0.0;
        for (int i = 0; i < 200; i++) {
            rig.cycle();
            if (i >= 60) {
                peakStow = Math.max(peakStow, Math.abs(rig.plant.current));
            }
        }
        assertTrue(peakStow <= 8.0 + 1e-6, "stow current " + peakStow);
        assertEquals(0.0, rig.plant.theta, 0.01, "the arm is at the hard stop");

        // Phase 2: blocked() has fired, so the gentle hold (1 A) replaces the stow. Only the last cycles are checked
        // so the transition itself is allowed a few cycles.
        double peakHold = 0.0;
        for (int i = 0; i < 150; i++) {
            rig.cycle();
            if (i >= 50) {
                peakHold = Math.max(peakHold, Math.abs(rig.plant.current));
            }
        }
        assertTrue(peakHold <= 1.0 + 1e-6, "holding current " + peakHold);
        assertTrue(rig.plant.theta > -1e-9 && rig.plant.theta < 0.05, "held at the stop, theta " + rig.plant.theta);
    }

    /**
     * YIELD against a scripted external torque: the arm holds with a small demand, a push larger than the
     * ceiling makes it yield (zero current, so it follows the push), and once the push is released the
     * ceiling ramps back and the arm returns toward its target.
     */
    @Test
    void yieldFollowsAScriptedPushAndReturnsWhenReleased() {
        Plant plant = new Plant();
        ControlBinding<?> angle = Controls.position(MOTOR, Units.Rotations)
                .feedback(plantPosition(plant), plantVelocity(plant))
                .pid(0.5, 0.0, 0.0)
                .model(model())
                .effort(Effort.max(Units.NewtonMeters.of(0.2)).gearing(1.0)
                        .onLoad(LoadResponse.YIELD.recoverAfter(Units.Seconds.of(0.1))));
        Rig rig = rigFor(plant, angle, angle.position(0.2));
        for (int i = 0; i < 75; i++) {
            rig.cycle();
        }
        assertEquals(0.2, plant.theta, 0.02, "holds the target before the push");

        plant.tauExt = 0.5;  // opposes the motor torque, so it pushes the arm below its target
        boolean yieldedDuringPush = false;
        double peakWhileYielded = 0.0;
        for (int i = 0; i < 50; i++) {
            rig.cycle();
            if (rig.runtimeYielded()) {
                yieldedDuringPush = true;
                peakWhileYielded = Math.max(peakWhileYielded, Math.abs(plant.current));
            }
        }
        assertTrue(yieldedDuringPush, "the push above the ceiling must yield");
        // Holding the push would take 0.5 N m, about 26 A. The zero-current voltage is computed from the speed at the
        // start of each cycle, and the push changes the speed within a cycle, so a few amps leak through.
        assertTrue(peakWhileYielded < 2.6, "a yielded arm draws little current, peak " + peakWhileYielded);
        assertTrue(plant.theta < 0.0, "the arm follows the push, theta " + plant.theta);

        plant.tauExt = 0.0;
        for (int i = 0; i < 400; i++) {
            rig.cycle();
        }
        assertTrue(plant.theta > -0.2, "the arm returns after the push ends, theta " + plant.theta);
    }

    private static Rig rigFor(Plant plant, ControlBinding<?> angle, Action action) {
        return new Rig(plant, angle, action);
    }

    private static PositionSignal plantPosition(Plant plant) {
        return () -> plant.theta;
    }

    private static VelocitySignal plantVelocity(Plant plant) {
        return () -> plant.omega / (2.0 * Math.PI);
    }
}
