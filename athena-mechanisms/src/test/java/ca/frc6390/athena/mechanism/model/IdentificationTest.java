package ca.frc6390.athena.mechanism.model;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.model.Identification.FeedforwardFit;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

class IdentificationTest {
    private static final double TWO_PI = 2.0 * Math.PI;

    @Test
    void rotaryFeedforwardRecoversGainsAndGravityPhase() {
        double kS = 0.30;
        double kV = 1.20;
        double kA = 0.05;
        double kG = 0.80;
        double offset = 0.40;
        Random random = new Random(11);
        List<CharacterizationSample> samples = new ArrayList<>();
        for (int i = 0; i < 2000; i++) {
            double t = i * 0.02;
            double position = 0.25 + 0.2 * Math.sin(0.31 * t) + 0.1 * Math.sin(1.7 * t);
            double velocity = 0.2 * 0.31 * Math.cos(0.31 * t) + 0.1 * 1.7 * Math.cos(1.7 * t);
            double accel = -0.2 * 0.31 * 0.31 * Math.sin(0.31 * t) - 0.1 * 1.7 * 1.7 * Math.sin(1.7 * t);
            double volts = kS * Math.signum(velocity) + kV * velocity + kA * accel
                    + kG * Math.cos(TWO_PI * position + offset) + 0.01 * random.nextGaussian();
            samples.add(sample(SysIdState.DYNAMIC_FORWARD, t, volts, position, velocity, accel));
        }

        FeedforwardFit fit = Identification.fitFeedforward(samples, MotionKind.ROTARY, 0.02);

        assertEquals(kS, fit.kineticStaticVolts(), 0.02);
        assertEquals(kV, fit.kV(), 0.02);
        assertEquals(kA, fit.kA(), 0.02);
        assertEquals(kG, fit.kG(), 0.02);
        assertEquals(offset, fit.gravityOffsetRadians(), 0.03);
        assertTrue(fit.rSquared() > 0.99);
        assertTrue(fit.standardErrors().get("kV") < 0.01);
    }

    @Test
    void linearFeedforwardRecoversConstantGravity() {
        Random random = new Random(4);
        List<CharacterizationSample> samples = new ArrayList<>();
        for (int i = 0; i < 1500; i++) {
            double t = i * 0.02;
            double position = 0.5 + 0.3 * Math.sin(0.31 * t) + 0.1 * Math.sin(1.7 * t);
            double velocity = 0.3 * 0.31 * Math.cos(0.31 * t) + 0.1 * 1.7 * Math.cos(1.7 * t);
            double accel = -0.3 * 0.31 * 0.31 * Math.sin(0.31 * t) - 0.1 * 1.7 * 1.7 * Math.sin(1.7 * t);
            double volts = 0.40 * Math.signum(velocity) + 2.0 * velocity + 0.2 * accel + 0.6
                    + 0.01 * random.nextGaussian();
            samples.add(sample(SysIdState.QUASISTATIC_FORWARD, t, volts, position, velocity, accel));
        }

        FeedforwardFit fit = Identification.fitFeedforward(samples, MotionKind.LINEAR, 0.02);

        assertEquals(0.40, fit.kineticStaticVolts(), 0.02);
        assertEquals(2.0, fit.kV(), 0.02);
        assertEquals(0.2, fit.kA(), 0.02);
        assertEquals(0.6, fit.kG(), 0.02);
        assertEquals(0.0, fit.gravityOffsetRadians(), 1.0e-12);
    }

    @Test
    void staticFrictionIsTheBreakawayVoltageMinusGravity() {
        FeedforwardFit model = new FeedforwardFit(0.4, 2.0, 0.0, 0.3, 0.0, Map.of(), 1.0, 1.0, 10);
        List<CharacterizationSample> samples = new ArrayList<>();
        double position = 0.0;
        for (int i = 0; i < 400; i++) {
            double t = i * 0.02;
            double volts = 0.5 * t;   // quasistatic ramp in volts per second
            double velocity = volts > 0.45 + 0.3 ? (volts - 0.45 - 0.3) / 2.0 : 0.0;
            position += velocity * 0.02;
            samples.add(sample(SysIdState.QUASISTATIC_FORWARD, t, volts, position, velocity, 0.0));
        }

        double staticVolts = Identification.staticFriction(samples, MotionKind.LINEAR, model, 0.01)
                .orElseThrow();

        assertEquals(0.45, staticVolts, 0.03);
    }

    @Test
    void backlashIsTheDistanceBetweenTheReversalAndTheReverseEngagement() {
        List<CharacterizationSample> samples = new ArrayList<>();
        double t = 0.0;
        for (int i = 0; i <= 100; i++) {
            samples.add(sample(SysIdState.QUASISTATIC_FORWARD, t, 1.0, i * 0.01, 1.0, 0.0));
            t += 0.02;
        }
        for (int i = 0; i < 5; i++) {
            samples.add(sample(SysIdState.QUASISTATIC_REVERSE, t, -0.1, 1.0, 0.0, 0.0));
            t += 0.02;
        }
        for (int i = 0; i < 20; i++) {
            samples.add(sample(SysIdState.QUASISTATIC_REVERSE, t, -1.0, 0.95 - i * 0.01, -1.0, 0.0));
            t += 0.02;
        }

        assertEquals(0.05, Identification.backlash(samples, 0.02).orElseThrow(), 1.0e-9);
    }

    @Test
    void latencyIsTheTimeFromStepOnsetToFirstMotion() {
        List<CharacterizationSample> samples = new ArrayList<>();
        for (int i = 0; i < 50; i++) {
            double t = i * 0.02;
            double velocity = t < 0.1 - 1.0e-9 ? 0.0 : 0.5;
            samples.add(sample(SysIdState.DYNAMIC_FORWARD, t, 4.0, 0.0, velocity, 0.0));
        }

        assertEquals(0.1, Identification.latency(samples, 0.05).orElseThrow(), 1.0e-9);
        assertFalse(Identification.latency(List.of(), 0.05).isPresent());
    }

    @Test
    void frictionTableCapturesPositionDependentFriction() {
        FeedforwardFit fit = new FeedforwardFit(0.5, 2.0, 0.0, 0.0, 0.0, Map.of(), 1.0, 1.0, 10);
        Random random = new Random(9);
        List<CharacterizationSample> samples = new ArrayList<>();
        for (int i = 0; i < 1000; i++) {
            double position = random.nextDouble();
            double velocity = (i % 2 == 0 ? 1.0 : -1.0) * (0.2 + random.nextDouble());
            double friction = 0.1 + 0.2 * position;
            double volts = Math.signum(velocity) * (0.5 + friction) + 2.0 * velocity;
            samples.add(sample(SysIdState.QUASISTATIC_FORWARD, i * 0.02, volts, position, velocity, 0.0));
        }

        FrictionTable table = Identification.frictionTable(samples, MotionKind.LINEAR, fit, 0.02, 8);

        assertFalse(table.isEmpty());
        assertTrue(table.at(0.9) - table.at(0.1) > 0.12, "table should rise with position");
        assertEquals(table.at(0.5), table.at(0.5 + 1.0e-12), 1.0e-9);
    }

    @Test
    void payloadIsTheLoadedGravityAmplitudeMinusTheBaseline() {
        FeedforwardFit baseline = new FeedforwardFit(0.5, 2.0, 0.2, 0.0, 0.0, Map.of(), 1.0, 1.0, 10);
        Random random = new Random(2);
        List<CharacterizationSample> holds = new ArrayList<>();
        for (int i = 0; i < 200; i++) {
            double volts = 0.5 + 0.3 + 0.01 * random.nextGaussian();
            holds.add(sample(SysIdState.QUASISTATIC_FORWARD, i * 0.02, volts, 0.0, 0.0, 0.0));
        }

        Identification.PayloadEstimate estimate = Identification.payload(holds, MotionKind.LINEAR, baseline, 0.5);

        assertEquals(0.3, estimate.deltaKG(), 0.01);
        assertEquals(200, estimate.samples());
    }

    @Test
    void fittingRequiresMoreSamplesThanParameters() {
        List<CharacterizationSample> few = List.of(
                sample(SysIdState.DYNAMIC_FORWARD, 0.0, 1.0, 0.0, 1.0, 0.0),
                sample(SysIdState.DYNAMIC_FORWARD, 0.02, 2.0, 0.1, -1.0, 0.0));
        assertThrows(IllegalArgumentException.class,
                () -> Identification.fitFeedforward(few, MotionKind.ROTARY, 0.02));
    }

    private static CharacterizationSample sample(SysIdState state, double t, double volts, double position,
            double velocity, double accel) {
        return new CharacterizationSample(state, t, volts, position, velocity, accel, Double.NaN);
    }
}
