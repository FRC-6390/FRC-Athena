package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.TorqueVelocityGains;
import ca.frc6390.athena.mechanism.effort.Effort;
import ca.frc6390.athena.mechanism.effort.EffortLimit;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

/**
 * Torque-based velocity control against a DC flywheel plant. The plant uses the Kraken X60 constants, so the
 * controller's model and the plant agree exactly. Gains are derived from the plant (see {@link #tuned}) for the
 * same closed-loop bandwidth, so the torque loop and the voltage PID+FF are compared at equivalent tuning.
 */
class TorqueVelocityTest {
    private static final double DT = 0.02;
    private static final int SUBSTEPS = 10;
    private static final MotorConstants K = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
    private static final double TWO_PI = 2.0 * Math.PI;
    /** Flywheel inertia on the motor shaft, kg m^2. */
    private static final double INERTIA = 0.01;
    /** Viscous damping, N m per (rad/s). */
    private static final double DAMPING = 0.0002;
    /** Target speed in rotations per second. About 6 V of back-EMF headroom remains at 12 V. */
    private static final double TARGET = 50.0;
    /** Closed-loop bandwidth both tunings are built for, rad/s. */
    private static final double BANDWIDTH = 20.0;
    /** Load torque of the disturbance test, N m (about 31 A through one Kraken). */
    private static final double LOAD = 0.6;
    private static final double LOAD_START = 1.0;
    private static final double LOAD_END = 1.5;
    /**
     * Load-estimate time constant of the observer under test, seconds. About half a control period: the estimate
     * then follows the load within one cycle, which is the latency limit of a 20 ms loop.
     */
    private static final double OBSERVER_TAU = 0.01;
    /** Speed band for the time-outside metric, rps (0.5% of the target). */
    private static final double BAND = 0.25;
    private static final RuntimeScope SCOPE = new RuntimeScope("torque-velocity");
    private static final AtomicInteger IDS = new AtomicInteger(400);

    /** Flywheel driven by identical motors on one shaft: {@code J w' = N Kt I - tau_load - b w}, per motor current I. */
    static final class Flywheel {
        final int motors;
        double omega;
        double volts;
        double current;
        double loadTorque;

        Flywheel(int motors) {
            this.motors = motors;
        }

        double rps() {
            return omega / TWO_PI;
        }

        void step(double dt) {
            double h = dt / SUBSTEPS;
            for (int s = 0; s < SUBSTEPS; s++) {
                current = (volts - omega / K.raw().speedConstantRadPerSecPerVolt()) / K.resistance().in(Units.Ohms);
                double torque = motors * K.raw().torqueConstantNmPerAmp() * current - loadTorque - DAMPING * omega;
                omega += torque / INERTIA * h;
            }
            current = (volts - omega / K.raw().speedConstantRadPerSecPerVolt()) / K.resistance().in(Units.Ohms);
        }
    }

    /** Motor handle that drives the shared flywheel. */
    static final class FlywheelMotor implements MotorHandle {
        private final MotorDevice device;
        private final Flywheel plant;

        FlywheelMotor(MotorDevice device, Flywheel plant) {
            this.device = device;
            this.plant = plant;
        }

        @Override public MotorDevice device() { return device; }
        @Override public MotorControlCapabilities controlCapabilities() {
            return MotorControlCapabilities.OPEN_LOOP_ONLY;
        }
        @Override public void setVoltage(double volts) { plant.volts = volts; }
        @Override public void setPercentOutput(double percent) { plant.volts = percent * 12.0; }
        @Override public double appliedVoltage() { return plant.volts; }
        @Override public double statorCurrentAmps() { return plant.current; }
        @Override public double supplyCurrentAmps() { return plant.current; }
        @Override public double integratedVelocityRotationsPerSecond() { return plant.rps(); }
        @Override public OptionalDouble temperatureCelsius() { return OptionalDouble.empty(); }
    }

    /** Mechanism that owns one velocity binding; the runtime discovers the binding by field. */
    static final class Shooter implements Mechanism {
        final ControlBinding<?> velocity;

        Shooter(ControlBinding<?> velocity) {
            this.velocity = velocity;
        }
    }

    /** A rig: the plant, its motors, one velocity binding and its runtime. */
    static final class Rig implements ActionContext {
        final Flywheel plant;
        final List<MotorDevice> motors = new ArrayList<>();
        final Map<MotorDevice, MotorHandle> handles = new HashMap<>();
        final ControlBinding<?> control;
        final MechanismRuntime runtime;
        double time;

        Rig(Flywheel plant, int motorCount, java.util.function.Function<List<MotorDevice>, ControlBinding<?>> build) {
            this.plant = plant;
            for (int index = 0; index < motorCount; index++) {
                motors.add(MotorDevice.of(MotorKinds.KRAKEN_X60, IDS.incrementAndGet()));
            }
            for (MotorDevice device : motors) {
                MotorHandle handle = new FlywheelMotor(device, plant);
                handles.put(device, handle);
                device.bindRuntime(SCOPE, handle);
            }
            control = build.apply(motors);
            runtime = MechanismRuntime.of(new Shooter(control), this);
            runtime.set(control.velocity(TARGET));
        }

        @Override public MotorHandle motor(MotorDevice ref) { return handles.get(ref); }
        @Override public EncoderHandle encoder(ca.frc6390.athena.hardware.device.EncoderDevice ref) { return null; }

        /** Runs the controller for one cycle; the plant is not stepped, so the commanded voltage can be read. */
        double command() {
            time += DT;
            runtime.periodic(new MechanismContext(time, 0.0, DT, true, false, false), EventContext.empty());
            return plant.volts;
        }

        /** Runs one controller cycle, then integrates the plant with the voltage it commanded. */
        void cycle() {
            command();
            plant.step(DT);
        }

        TorqueVelocityGains torque() {
            for (ControlLoop loop : control.loops()) {
                if (loop instanceof TorqueVelocityGains gains) {
                    return gains;
                }
            }
            throw new AssertionError("no torque velocity loop on the binding");
        }

        double telemetry(String name) {
            return torque().telemetry().get(name).number();
        }
    }

    /** Position and velocity feedback read straight from the plant. */
    private static PositionSignal position(Flywheel plant) {
        return () -> plant.omega / TWO_PI;
    }

    private static VelocitySignal velocity(Flywheel plant) {
        return () -> plant.rps();
    }

    /**
     * Gains for the torque loop and the voltage loop at the same bandwidth. For a motor with inertia {@code J} and
     * torque constant {@code Kt}, a proportional gain {@code kP} in amps per rps gives a closed-loop rate of
     * {@code Kt kP / (2 pi J)}; the voltage gain is the same loop with the resistance folded in.
     */
    private static final class Tuning {
        final double kpAmps = 2.0 * Math.PI * INERTIA * BANDWIDTH / K.raw().torqueConstantNmPerAmp();
        final double kiAmps = kpAmps * BANDWIDTH / 4.0;
        final double kaAmps = 2.0 * Math.PI * INERTIA / K.raw().torqueConstantNmPerAmp();
        final double kpVolts = kpAmps * K.resistance().in(Units.Ohms);
        final double kiVolts = kiAmps * K.resistance().in(Units.Ohms);
        final double kaVolts = kaAmps * K.resistance().in(Units.Ohms);
        final double kvVolts = TWO_PI / K.raw().speedConstantRadPerSecPerVolt();
    }

    private static final Tuning TUNING = new Tuning();

    private static ControlBinding<?> voltageControl(Flywheel plant, List<MotorDevice> motors) {
        return Controls.velocity(motors.get(0), motors.subList(1, motors.size()).toArray(new MotorDevice[0]))
                .feedback(position(plant), velocity(plant))
                .pid(TUNING.kpVolts, TUNING.kiVolts, 0.0)
                .feedforward(FeedforwardGains.of(0.0, TUNING.kvVolts, TUNING.kaVolts, 0.0));
    }

    private static ControlBinding<?> torqueControl(Flywheel plant, List<MotorDevice> motors, double observerTau) {
        return Controls.velocity(motors.get(0), motors.subList(1, motors.size()).toArray(new MotorDevice[0]))
                .feedback(position(plant), velocity(plant))
                .torqueVelocity(TorqueVelocityGains.of(TUNING.kpAmps, TUNING.kiAmps, 0.0)
                        .inertia(TUNING.kaAmps)
                        .loadEstimate(observerTau));
    }

    @Test
    void voltageIsResistiveDropPlusBackEmfOfTheMeasuredSpeed() {
        Flywheel plant = new Flywheel(1);
        Rig rig = new Rig(plant, 1, motors -> Controls.velocity(motors.get(0))
                .feedback(position(plant), velocity(plant))
                .torqueVelocity(TorqueVelocityGains.of(1.0, 0.0, 0.0)));
        rig.runtime.set(rig.control.velocity(20.0));
        int checked = 0;
        for (int cycle = 0; cycle < 60; cycle++) {
            double omegaBefore = plant.omega;
            double volts = rig.command();
            double amps = rig.telemetry("CurrentCommandAmps");
            if (Math.abs(volts) < 12.0 - 1e-9) {
                // The command is I R + omega / Kv with the speed measured at the start of the cycle.
                assertEquals(amps * K.resistance().in(Units.Ohms) + omegaBefore / K.raw().speedConstantRadPerSecPerVolt(), volts, 1e-9,
                        "cycle " + cycle);
                assertEquals(volts, rig.telemetry("VoltageCommand"), 1e-12);
                checked++;
            }
            plant.step(DT);
        }
        assertTrue(checked > 40, "most cycles are unclamped, checked " + checked);
    }

    @Test
    void steadyStateHoldsTheTargetWithoutLoad() {
        Flywheel plant = new Flywheel(1);
        Rig rig = new Rig(plant, 1, motors -> torqueControl(plant, motors, 0.0));
        for (int cycle = 0; cycle < 150; cycle++) {
            rig.cycle();
        }
        assertEquals(TARGET, plant.rps(), 0.05, "zero error at steady state");
        assertEquals(0.0, rig.telemetry("LoadEstimateAmps"), 1e-9, "no estimate when it is off");
    }

    /**
     * Speed dip after a load step, for the voltage PID+FF, the torque loop, and the torque loop with the
     * load estimate. The dip is the largest shortfall below the target in the window; the time outside the band
     * is how long the speed stays more than 1% from the target after the load arrives.
     */
    @Test
    void loadEstimateShrinksAndShortensTheSpeedDipAfterALoadStep() {
        Dip voltage = dipAfterLoadStep(Variant.VOLTAGE);
        Dip torque = dipAfterLoadStep(Variant.TORQUE);
        Dip observed = dipAfterLoadStep(Variant.TORQUE_WITH_LOAD_ESTIMATE);
        System.out.printf("Load step %.2f N m from %.1f s to %.1f s, target %.0f rps, band %.2f rps%n",
                LOAD, LOAD_START, LOAD_END, TARGET, BAND);
        System.out.printf("  voltage PID+FF : dip %.3f rps, time outside band %.3f s, release overshoot %.3f rps%n",
                voltage.dip, voltage.outsideBand, voltage.releaseOvershoot);
        System.out.printf("  torque loop    : dip %.3f rps, time outside band %.3f s, release overshoot %.3f rps%n",
                torque.dip, torque.outsideBand, torque.releaseOvershoot);
        System.out.printf("  torque + load  : dip %.3f rps, time outside band %.3f s, release overshoot %.3f rps%n",
                observed.dip, observed.outsideBand, observed.releaseOvershoot);
        assertTrue(observed.dip < voltage.dip * 0.7,
                "the load estimate should cut the dip by 30% or more: " + observed.dip + " vs " + voltage.dip);
        assertTrue(observed.outsideBand < voltage.outsideBand,
                "and shorten the time outside the band: " + observed.outsideBand + " vs " + voltage.outsideBand);
        assertTrue(observed.releaseOvershoot < 0.02 * TARGET,
                "the release overshoot stays under 2% of the target: " + observed.releaseOvershoot);
    }

    private record Dip(double dip, double outsideBand, double releaseOvershoot) {
    }

    private enum Variant { VOLTAGE, TORQUE, TORQUE_WITH_LOAD_ESTIMATE }

    /** Runs a load step on one flywheel with the given control variant. */
    private static Dip dipAfterLoadStep(Variant variant) {
        Flywheel plant = new Flywheel(1);
        Rig rig = new Rig(plant, 1, motors -> switch (variant) {
            case VOLTAGE -> voltageControl(plant, motors);
            case TORQUE -> torqueControl(plant, motors, 0.0);
            case TORQUE_WITH_LOAD_ESTIMATE -> torqueControl(plant, motors, OBSERVER_TAU);
        });
        for (int cycle = 0; cycle < 100; cycle++) {
            rig.cycle();
        }
        double worst = 0.0;
        double outside = 0.0;
        double overshoot = 0.0;
        int steps = (int) Math.round(3.0 / DT);
        for (int cycle = 0; cycle < steps; cycle++) {
            double t = cycle * DT;
            plant.loadTorque = t >= LOAD_START && t < LOAD_END ? LOAD : 0.0;
            rig.cycle();
            double error = TARGET - plant.rps();
            if (t >= LOAD_START) {
                worst = Math.max(worst, error);
                if (Math.abs(error) > BAND) {
                    outside += DT;
                }
            }
            if (t >= LOAD_END) {
                overshoot = Math.max(overshoot, -error);
            }
        }
        return new Dip(worst, outside, overshoot);
    }

    @Test
    void loadEstimateTracksTheLoadCurrent() {
        Flywheel plant = new Flywheel(1);
        Rig rig = new Rig(plant, 1, motors -> torqueControl(plant, motors, OBSERVER_TAU));
        for (int cycle = 0; cycle < 100; cycle++) {
            rig.cycle();
        }
        plant.loadTorque = LOAD;
        for (int cycle = 0; cycle < 60; cycle++) {
            rig.cycle();
        }
        // Load current from the plant: the load torque plus the damping torque at the speed, over Kt.
        double expected = (LOAD + DAMPING * plant.omega) / K.raw().torqueConstantNmPerAmp();
        double estimate = rig.telemetry("LoadEstimateAmps");
        assertEquals(expected, estimate, 0.25 * expected, "load estimate " + estimate + " A, expected " + expected);
    }

    @Test
    void antiWindupRecoversAfterASaturatedStep() {
        Flywheel plant = new Flywheel(1);
        Rig rig = new Rig(plant, 1, motors -> torqueControl(plant, motors, 0.0));
        rig.runtime.set(rig.control.velocity(150.0));
        double largestVoltage = 0.0;
        for (int cycle = 0; cycle < 75; cycle++) {
            rig.cycle();
            largestVoltage = Math.max(largestVoltage, Math.abs(plant.volts));
        }
        assertEquals(12.0, largestVoltage, 1e-9, "the voltage saturates at 12 V");
        assertTrue(plant.rps() < 110.0, "the flywheel cannot reach 150 rps, reached " + plant.rps());

        rig.runtime.set(rig.control.velocity(TARGET));
        double lowest = Double.POSITIVE_INFINITY;
        boolean crossed = false;
        for (int cycle = 0; cycle < 250; cycle++) {
            rig.cycle();
            assertTrue(Math.abs(plant.volts) <= 12.0 + 1e-9, "voltage within the bus, cycle " + cycle);
            if (plant.rps() <= TARGET * 1.01) {
                crossed = true;
            }
            if (crossed) {
                lowest = Math.min(lowest, plant.rps());
            }
        }
        // Without anti-windup the integral would hold tens of thousands of amps and the speed would undershoot far more.
        assertTrue(crossed, "the speed comes down to the target");
        assertTrue(lowest > TARGET * 0.9, "undershoot after the saturated step, lowest " + lowest);
        assertEquals(TARGET, plant.rps(), 0.5, "settles back on the target");
    }

    @Test
    void effortCeilingBoundsTheCurrentOfTheTorqueLoop() {
        Flywheel plant = new Flywheel(1);
        Rig rig = new Rig(plant, 1, motors -> torqueControl(plant, motors, 0.0)
                .effort(Effort.max(Units.Amps.of(5.0))));
        assertTrue(rig.control.loops().stream().anyMatch(loop -> loop instanceof EffortLimit),
                "the effort ceiling is a loop on the binding");
        double peak = 0.0;
        for (int cycle = 0; cycle < 150; cycle++) {
            // The current is largest at the start of a cycle, while the speed is still the cycle's first value.
            rig.command();
            peak = Math.max(peak, Math.abs(startOfCycleCurrent(plant)));
            plant.step(DT);
        }
        assertTrue(peak <= 5.0 + 1e-6, "peak current " + peak);
        assertTrue(peak > 4.9, "the ceiling is reached while accelerating, peak " + peak);
    }

    /** Current the commanded voltage drives at the speed the cycle started from. */
    private static double startOfCycleCurrent(Flywheel plant) {
        return (plant.volts - plant.omega / K.raw().speedConstantRadPerSecPerVolt()) / K.resistance().in(Units.Ohms);
    }

    @Test
    void powerCapBoundsTheCurrentOfTheTorqueLoop() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, IDS.incrementAndGet())
                .power(PowerProfile.of().sustained(Units.Amps.of(5.0)));
        Flywheel plant = new Flywheel(1);
        Map<MotorDevice, MotorHandle> handles = Map.of(motor, new FlywheelMotor(motor, plant));
        ActionContext hardware = new ActionContext() {
            @Override public MotorHandle motor(MotorDevice ref) { return handles.get(ref); }
        };
        motor.bindRuntime(SCOPE, handles.get(motor));
        ControlBinding<?> control = Controls.velocity(motor)
                .feedback(position(plant), velocity(plant))
                .torqueVelocity(TorqueVelocityGains.of(TUNING.kpAmps, TUNING.kiAmps, 0.0));
        PowerRuntime power = new PowerRuntime();
        power.policy(PowerPolicy.defaults().estimateBatteryResistance(false));
        power.busVoltage(() -> 12.5);
        OutputApplier applier = OutputApplier.using(hardware);
        applier.electrical(power);
        Output target = Outputs.velocity(TARGET);
        double peak = 0.0;
        for (int cycle = 0; cycle < 100; cycle++) {
            double now = cycle * DT;
            power.cycle(List.of(motor), hardware, new RuntimeOverrides(), now);
            applier.apply(new ResolvedOutput(OutputRequest.of(control, target), target));
            if (cycle > 0) {
                peak = Math.max(peak, Math.abs(startOfCycleCurrent(plant)));
            }
            plant.step(DT);
        }
        assertEquals(5.0, power.capAmps(motor), 1e-9, "the sustained cap");
        assertTrue(peak <= 5.0 + 1e-6, "power-capped current " + peak);
        assertTrue(peak > 4.5, "the cap is reached while accelerating, peak " + peak);
    }

    @Test
    void motorKindWithoutConstantsIsRejectedAtDeclaration() {
        MotorDevice minion = MotorDevice.of(MotorKinds.MINION, IDS.incrementAndGet());
        Flywheel plant = new Flywheel(1);
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class,
                () -> Controls.velocity(minion)
                        .feedback(position(plant), velocity(plant))
                        .torqueVelocity(TorqueVelocityGains.of(1.0, 0.0, 0.0)));
        assertTrue(error.getMessage().contains("motor constants"), error.getMessage());
    }

    @Test
    void torqueVelocityNeedsAVelocityBinding() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, IDS.incrementAndGet());
        Flywheel plant = new Flywheel(1);
        assertThrows(IllegalStateException.class, () -> Controls.position(motor)
                .feedback(position(plant), velocity(plant))
                .torqueVelocity(TorqueVelocityGains.of(1.0, 0.0, 0.0)));
    }

    @Test
    void voltageLoopsCannotBeStackedOnTorqueVelocity() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, IDS.incrementAndGet());
        Flywheel plant = new Flywheel(1);
        ControlBinding<?> torque = Controls.velocity(motor)
                .feedback(position(plant), velocity(plant))
                .torqueVelocity(TorqueVelocityGains.of(1.0, 0.0, 0.0));
        assertThrows(IllegalStateException.class, () -> torque.pid(1.0, 0.0, 0.0));
        assertThrows(IllegalStateException.class, () -> torque.ff(0.0, 0.1, 0.0));
    }

    @Test
    void declaringTorqueVelocityReplacesEarlierVoltageLoops() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, IDS.incrementAndGet());
        Flywheel plant = new Flywheel(1);
        ControlBinding<?> binding = Controls.velocity(motor)
                .feedback(position(plant), velocity(plant))
                .pid(1.0, 0.0, 0.0)
                .ff(0.0, 0.1, 0.0)
                .torqueVelocity(TorqueVelocityGains.of(1.0, 0.0, 0.0));
        assertEquals(1, binding.loops().size(), "only the torque loop remains: " + binding.loops());
        assertTrue(binding.loops().get(0) instanceof TorqueVelocityGains);
    }

    @Test
    void followerCountSplitsTheCurrentAcrossTheMotors() {
        Flywheel single = new Flywheel(1);
        Rig one = new Rig(single, 1, motors -> Controls.velocity(motors.get(0))
                .feedback(position(single), velocity(single))
                .torqueVelocity(TorqueVelocityGains.of(1.0, 0.0, 0.0)));
        Flywheel pair = new Flywheel(2);
        Rig two = new Rig(pair, 2, motors -> Controls.velocity(motors.get(0), motors.get(1))
                .feedback(position(pair), velocity(pair))
                .torqueVelocity(TorqueVelocityGains.of(1.0, 0.0, 0.0)));
        one.runtime.set(one.control.velocity(20.0));
        two.runtime.set(two.control.velocity(20.0));
        // At rest the back-EMF is zero, so the voltage is I R per motor: the pair draws half the current each.
        double voltageOne = one.command();
        double voltageTwo = two.command();
        assertEquals(one.telemetry("CurrentCommandAmps"), two.telemetry("CurrentCommandAmps"), 1e-9,
                "the same total current command");
        assertEquals(voltageOne / 2.0, voltageTwo, 1e-9, "each of two motors carries half the current");
    }

    @Test
    void gainChangeIsBumpless() {
        Flywheel plant = new Flywheel(1);
        Rig rig = new Rig(plant, 1, motors -> torqueControl(plant, motors, 0.0));
        for (int cycle = 0; cycle < 80; cycle++) {
            rig.cycle();
        }
        double before = plant.volts;
        assertTrue(rig.torque().telemetry().get("p").tryWrite(TUNING.kpAmps * 2.0).accepted(), "p is live-tunable");
        rig.command();
        assertEquals(before, plant.volts, 1e-6, "the output does not step when the gain changes");
    }
}
