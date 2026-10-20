package ca.frc6390.athena.mechanism.effort;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.model.FrictionTable;
import ca.frc6390.athena.mechanism.model.MechanismModel;
import edu.wpi.first.units.Units;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

/** Effort state and voltage clamp (B5/B6) driven with scripted inputs. */
class EffortRuntimeTest {
    private static final double DT = 0.02;
    private static final MotorDevice KRAKEN = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final MotorDevice NEO = MotorDevice.of(MotorKinds.NEO, 2);
    private static final MotorConstants KRAKEN_K = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
    private static final MotorConstants NEO_K = MotorConstants.forKind(MotorKinds.NEO).orElseThrow();

    /** Motor handle with a fixed speed; the voltage is only recorded. */
    static final class FakeMotor implements MotorHandle {
        private final MotorDevice device;
        double rotationsPerSecond;
        double supplyAmps = Double.NaN;

        FakeMotor(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public double integratedVelocityRotationsPerSecond() {
            return rotationsPerSecond;
        }

        @Override
        public double supplyCurrentAmps() {
            return supplyAmps;
        }
    }

    static final class FakeContext implements ActionContext {
        private final Map<MotorDevice, FakeMotor> motors = new java.util.HashMap<>();

        FakeMotor motor(MotorDevice device, double rotationsPerSecond) {
            FakeMotor handle = new FakeMotor(device);
            handle.rotationsPerSecond = rotationsPerSecond;
            motors.put(device, handle);
            return handle;
        }

        @Override
        public MotorHandle motor(MotorDevice ref) {
            return motors.get(ref);
        }
    }

    /** Predicted current for a voltage at a speed: the quantity the clamp bounds. */
    static double currentAt(double volts, double rotationsPerSecond, MotorConstants k) {
        double omega = rotationsPerSecond * 2.0 * Math.PI;
        return (volts - omega / k.raw().speedConstantRadPerSecPerVolt()) / k.resistance().in(Units.Ohms);
    }

    static MechanismModel model(MotorConstants k) {
        // Feedforward matches back-EMF exactly (kV = 2 pi / Kv); no friction, gravity or inertia terms.
        return new MechanismModel(MotionKind.ROTARY, 0.0, 0.0, 2.0 * Math.PI / k.raw().speedConstantRadPerSecPerVolt(), 0.0, 0.0,
                0.0, FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, k.raw().torqueConstantNmPerAmp(), k.resistance().in(Units.Ohms), 1,
                List.of(), 1.0, Map.of(), 0.0, 0);
    }

    private static EncoderDevice encoder(int id) {
        return EncoderDevice.of(EncoderKinds.CANCODER, id).measures(Units.Rotations);
    }

    /** Runtime of the binding's effort loop. */
    private static EffortRuntime runtime(ControlBinding<?> binding) {
        for (var loop : binding.loops()) {
            if (loop instanceof EffortLimit limit) {
                return (EffortRuntime) limit.bind(new ControlLoopBinding(binding, ActionContext.empty()));
            }
        }
        throw new AssertionError("no effort loop");
    }

    @Test
    void stallClampLimitsVoltageToCurrentTimesResistance() {
        EffortRuntime runtime = EffortRuntime.unlimited(List.of(KRAKEN));
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 0.0);
        runtime.advance(5.0, DT, 12.0, 0.0, 0.0);
        double clamped = runtime.clampVolts(context, 12.0);
        assertEquals(5.0 * KRAKEN_K.resistance().in(Units.Ohms), clamped, 1e-12);
        assertEquals(-5.0 * KRAKEN_K.resistance().in(Units.Ohms), runtime.clampVolts(context, -12.0), 1e-12);
        // Small commands are left alone.
        assertEquals(0.05, runtime.clampVolts(context, 0.05), 0.0);
    }

    @Test
    void movingClampBoundsTheCurrentAroundTheBackEmf() {
        EffortRuntime runtime = EffortRuntime.unlimited(List.of(KRAKEN));
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 40.0);
        runtime.advance(10.0, DT, 12.0, 0.0, 40.0);
        double clamped = runtime.clampVolts(context, 12.0);
        assertEquals(10.0, currentAt(clamped, 40.0, KRAKEN_K), 1e-9);
        double low = runtime.clampVolts(context, -12.0);
        assertEquals(-10.0, currentAt(low, 40.0, KRAKEN_K), 1e-9);
    }

    @Test
    void predictedCurrentNeverExceedsTheLimitForAnyCommandOrSpeed() {
        Random random = new Random(7);
        for (int trial = 0; trial < 2000; trial++) {
            double limit = 0.5 + random.nextDouble() * 60.0;
            double speed = (random.nextDouble() - 0.5) * 200.0;
            double command = (random.nextDouble() - 0.5) * 24.0;
            EffortRuntime runtime = EffortRuntime.unlimited(List.of(KRAKEN));
            FakeContext context = new FakeContext();
            context.motor(KRAKEN, speed);
            runtime.advance(limit, DT, command, 0.0, speed);
            double volts = runtime.clampVolts(context, command);
            assertTrue(Math.abs(currentAt(volts, speed, KRAKEN_K)) <= limit + 1e-9,
                    "limit " + limit + " speed " + speed + " command " + command);
            // A command already inside the window is not changed.
            if (Math.abs(currentAt(command, speed, KRAKEN_K)) <= limit) {
                assertEquals(command, volts, 0.0);
            }
        }
    }

    @Test
    void twoMotorsShareOneVoltageSoEachStaysWithinItsOwnLimit() {
        EffortRuntime runtime = EffortRuntime.unlimited(List.of(KRAKEN, NEO));
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 30.0);
        context.motor(NEO, 30.0);
        runtime.advance(8.0, DT, 12.0, 0.0, 30.0);
        double volts = runtime.clampVolts(context, 12.0);
        assertTrue(Math.abs(currentAt(volts, 30.0, KRAKEN_K)) <= 8.0 + 1e-9);
        assertTrue(Math.abs(currentAt(volts, 30.0, NEO_K)) <= 8.0 + 1e-9);
    }

    @Test
    void motorsWithoutConstantsAreNotClampedAndKeepTheHardwareLimit() {
        MotorDevice minion = MotorDevice.of(MotorKinds.MINION, 3);
        EffortRuntime runtime = EffortRuntime.unlimited(List.of(minion));
        FakeContext context = new FakeContext();
        context.motor(minion, 0.0);
        runtime.advance(5.0, DT, 12.0, 0.0, 0.0);
        assertEquals(12.0, runtime.clampVolts(context, 12.0), 0.0);
    }

    @Test
    void ceilingFromTheBindingIsEnforcedWithoutAnyActionLimit() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations).feedback(encoder(4))
                .model(model(KRAKEN_K))
                .effort(Effort.max(Units.NewtonMeters.of(0.1)).gearing(1.0));
        EffortRuntime runtime = runtime(arm);
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 0.0);
        runtime.advance(Double.POSITIVE_INFINITY, DT, 12.0, 0.0, 0.0);
        double expectedAmps = 0.1 / KRAKEN_K.raw().torqueConstantNmPerAmp();
        assertEquals(expectedAmps * KRAKEN_K.resistance().in(Units.Ohms), runtime.clampVolts(context, 12.0), 1e-9);
    }

    @Test
    void yieldDropsAuthorityWhileDemandExceedsCeilingThenRampsBack() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations).feedback(encoder(5))
                .model(model(KRAKEN_K))
                .effort(Effort.max(Units.NewtonMeters.of(0.2)).gearing(1.0)
                        .onLoad(LoadResponse.YIELD.recoverAfter(Units.Seconds.of(0.1))));
        EffortRuntime runtime = runtime(arm);
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 0.0);

        // Demand below the ceiling: normal operation.
        runtime.advance(Double.POSITIVE_INFINITY, DT, 0.1, 0.0, 0.0);
        assertFalse(runtime.yielded());
        assertEquals(0.1, runtime.clampVolts(context, 0.1), 1e-12);

        // Demand far above the ceiling: yields, so the command falls to zero current.
        runtime.advance(Double.POSITIVE_INFINITY, DT, 1.0, 0.0, 0.0);
        assertTrue(runtime.yielded());
        assertEquals(0.0, currentAt(runtime.clampVolts(context, 1.0), 0.0, KRAKEN_K), 1e-9);

        // The applied current stays zero, so the load clears after the recovery time (5 cycles of 0.02 s).
        for (int i = 0; i < 4; i++) {
            runtime.advance(Double.POSITIVE_INFINITY, DT, 1.0, 0.0, 0.0);
            runtime.clampVolts(context, 1.0);
            assertTrue(runtime.yielded(), "still yielded at cycle " + i);
        }
        runtime.advance(Double.POSITIVE_INFINITY, DT, 0.0, 0.0, 0.0);
        runtime.clampVolts(context, 0.0);
        // Cleared for 0.1 s: the ceiling ramps back in over the same time, starting from zero.
        assertFalse(runtime.yielded());
        runtime.advance(Double.POSITIVE_INFINITY, DT, 0.1, 0.0, 0.0);
        double early = Math.abs(currentAt(runtime.clampVolts(context, 12.0), 0.0, KRAKEN_K));
        assertTrue(early < 0.2 / KRAKEN_K.raw().torqueConstantNmPerAmp(), "ramp starts below the ceiling: " + early);
        for (int i = 0; i < 6; i++) {
            runtime.advance(Double.POSITIVE_INFINITY, DT, 0.1, 0.0, 0.0);
        }
        double full = Math.abs(currentAt(runtime.clampVolts(context, 12.0), 0.0, KRAKEN_K));
        assertEquals(0.2 / KRAKEN_K.raw().torqueConstantNmPerAmp(), full, 1e-6);
    }

    @Test
    void holdingDemandBelowCeilingNeverYields() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations).feedback(encoder(6))
                .model(model(KRAKEN_K))
                .effort(Effort.max(Units.NewtonMeters.of(0.2)).gearing(1.0)
                        .onLoad(LoadResponse.YIELD.recoverAfter(Units.Seconds.of(0.1))));
        EffortRuntime runtime = runtime(arm);
        for (int i = 0; i < 100; i++) {
            runtime.advance(Double.POSITIVE_INFINITY, DT, 0.2, 0.0, 0.0);
            assertFalse(runtime.yielded());
        }
    }

    @Test
    void pushThroughAllowsTheBurstForItsDurationThenFallsBackToTheCeiling() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations).feedback(encoder(7))
                .model(model(KRAKEN_K))
                .effort(Effort.max(Units.NewtonMeters.of(0.2)).gearing(1.0)
                        .onLoad(LoadResponse.pushThrough(Units.NewtonMeters.of(0.4), Units.Seconds.of(0.1))));
        EffortRuntime runtime = runtime(arm);
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 0.0);
        double burstVolts = 0.4 / KRAKEN_K.raw().torqueConstantNmPerAmp() * KRAKEN_K.resistance().in(Units.Ohms);
        double ceilingVolts = 0.2 / KRAKEN_K.raw().torqueConstantNmPerAmp() * KRAKEN_K.resistance().in(Units.Ohms);
        for (int i = 1; i <= 5; i++) {
            runtime.advance(Double.POSITIVE_INFINITY, DT, 12.0, 0.0, 0.0);
            assertEquals(burstVolts, runtime.clampVolts(context, 12.0), 1e-9, "burst cycle " + i);
        }
        runtime.advance(Double.POSITIVE_INFINITY, DT, 12.0, 0.0, 0.0);
        assertEquals(ceilingVolts, runtime.clampVolts(context, 12.0), 1e-9);
        assertFalse(runtime.yielded());
    }

    @Test
    void slowScalesTheSpeedOnlyWhileLoaded() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations).feedback(encoder(8))
                .model(model(KRAKEN_K))
                .effort(Effort.max(Units.NewtonMeters.of(0.2)).gearing(1.0)
                        .onLoad(LoadResponse.slow(0.5)));
        EffortRuntime runtime = runtime(arm);
        runtime.advance(Double.POSITIVE_INFINITY, DT, 0.1, 0.0, 0.0);
        assertEquals(1.0, runtime.speedScale(), 0.0);
        runtime.advance(Double.POSITIVE_INFINITY, DT, 1.0, 0.0, 0.0);
        assertEquals(0.5, runtime.speedScale(), 0.0);
        runtime.advance(Double.POSITIVE_INFINITY, DT, 0.1, 0.0, 0.0);
        assertEquals(1.0, runtime.speedScale(), 0.0);
    }

    @Test
    void withoutAModelResponsesAreIgnoredAndOnlyTheCeilingApplies() {
        ControlBinding<?> arm = Controls.position(KRAKEN, Units.Rotations).feedback(encoder(9))
                .effort(Effort.max(Units.NewtonMeters.of(0.2)).gearing(1.0)
                        .onLoad(LoadResponse.YIELD.recoverAfter(Units.Seconds.of(0.1))));
        EffortRuntime runtime = runtime(arm);
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 0.0);
        runtime.advance(Double.POSITIVE_INFINITY, DT, 1.0, 0.0, 0.0);
        assertFalse(runtime.yielded());
        assertEquals(0.2 / KRAKEN_K.raw().torqueConstantNmPerAmp() * KRAKEN_K.resistance().in(Units.Ohms),
                runtime.clampVolts(context, 12.0), 1e-9);
    }

    @Test
    void powerCapAndActionEffortCombineAsTheSmallerLimit() {
        EffortRuntime runtime = EffortRuntime.unlimited(List.of(KRAKEN));
        FakeContext context = new FakeContext();
        context.motor(KRAKEN, 0.0);
        // Action effort 8 A, power cap 5 A: the cap binds.
        runtime.advance(8.0, new double[] {5.0}, DT, 12.0, 0.0, 0.0);
        assertEquals(5.0 * KRAKEN_K.resistance().in(Units.Ohms), runtime.clampVolts(context, 12.0), 1e-12);
        // Action effort 8 A, power cap 30 A: the Action binds.
        runtime.advance(8.0, new double[] {30.0}, DT, 12.0, 0.0, 0.0);
        assertEquals(8.0 * KRAKEN_K.resistance().in(Units.Ohms), runtime.clampVolts(context, 12.0), 1e-12);
    }

    @Test
    void motorWithoutConstantsIsScaledByItsMeasuredSupplyCurrent() {
        MotorDevice minion = MotorDevice.of(MotorKinds.MINION, 3);
        EffortRuntime runtime = EffortRuntime.unlimited(List.of(minion));
        FakeContext context = new FakeContext();
        context.motor(minion, 0.0).supplyAmps = 40.0;
        // Cap 20 A against 40 A measured: the command is halved.
        runtime.advance(Double.POSITIVE_INFINITY, new double[] {20.0}, DT, 12.0, 0.0, 0.0);
        assertEquals(6.0, runtime.clampVolts(context, 12.0), 1e-9);
        // Below the cap the command passes unchanged.
        context.motor(minion, 0.0).supplyAmps = 10.0;
        assertEquals(12.0, runtime.clampVolts(context, 12.0), 1e-9);
    }
}
