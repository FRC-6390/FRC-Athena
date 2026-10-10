package ca.frc6390.athena.mechanism.model;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.model.ExcitationGate.ExcitationCriteria;
import java.util.Random;
import org.junit.jupiter.api.Test;

class ExcitationGateTest {
    private static final ExcitationCriteria CRITERIA = new ExcitationCriteria(60, 30, 0.05, 0.05, 1.0e4);

    @Test
    void rejectsConstantVelocityAndAcceptsAVaryingSweep() {
        ExcitationGate gate = new ExcitationGate(CRITERIA);
        ExcitationGate.Decision last = null;
        for (int i = 0; i < 120; i++) {
            double velocity = 0.5;
            double accel = 0.0;
            last = gate.offer(Identification.features(
                    ca.frc6390.athena.hardware.units.MotionKind.LINEAR, i * 0.01, velocity, accel, 0.02), velocity, accel);
        }
        assertFalse(last.accepted());
        assertTrue(last.reason().contains("velocity variance"), last.reason());

        Random random = new Random(1);
        for (int i = 0; i < 120; i++) {
            double velocity = Math.sin(i * 0.3) + 0.05 * random.nextGaussian();
            double accel = Math.cos(i * 0.3) * 0.3;
            last = gate.offer(Identification.features(
                    ca.frc6390.athena.hardware.units.MotionKind.LINEAR, i * 0.01, velocity, accel, 0.02), velocity, accel);
        }
        assertTrue(last.accepted(), last.reason());
    }

    @Test
    void rejectsUntilTheSampleCountIsReached() {
        ExcitationGate gate = new ExcitationGate(CRITERIA);
        ExcitationGate.Decision decision = null;
        for (int i = 0; i < 50; i++) {
            double velocity = Math.sin(i * 0.4);
            decision = gate.offer(new double[] {1.0, velocity, velocity * 0.1, 1.0}, velocity, velocity * 0.1);
        }
        assertFalse(decision.accepted());
        assertTrue(decision.reason().contains("need 60"), decision.reason());
        gate.reset();
        assertTrue(gate.offered() == 0L);
    }

    @Test
    void rejectsLowAccelerationVariance() {
        ExcitationGate gate = new ExcitationGate(CRITERIA);
        ExcitationGate.Decision decision = null;
        for (int i = 0; i < 120; i++) {
            double velocity = Math.sin(i * 0.3);
            decision = gate.offer(new double[] {1.0, velocity, 0.0, 1.0}, velocity, 0.0);
        }
        assertFalse(decision.accepted());
        assertTrue(decision.reason().contains("acceleration variance"), decision.reason());
    }
}
