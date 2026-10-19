package ca.frc6390.athena.mechanism.model;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.model.DriftMonitor.DriftReport;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

class MechanismModelTest {
    private static MechanismModel model(MotionKind kind, double kA) {
        MotorConstants constants = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
        return new MechanismModel(kind, 0.5, 0.4, 2.0, kA, 0.3, 0.1, FrictionTable.EMPTY, Double.NaN,
                Double.NaN, 0.0, constants.torqueConstantNmPerAmp(), constants.resistance().in(Units.Ohms), 2,
                List.of("ctre/kraken-x60", "ctre/kraken-x60"), 10.0, Map.of("kV", 0.01), 0.99, 500);
    }

    @Test
    void feedforwardUsesStaticFrictionAtRestAndKineticWhenMoving() {
        MechanismModel model = new MechanismModel(MotionKind.LINEAR, 0.5, 0.4, 2.0, 0.0, 0.0, 0.0,
                FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, 0.0, 0.0, 0, List.of(), 1.0,
                Map.of(), 0.0, 0);

        assertEquals(0.5, model.feedforwardVolts(0.0, 0.0, 0.0, 1.0), 1.0e-12);
        assertEquals(-0.5, model.feedforwardVolts(0.0, 0.0, 0.0, -1.0), 1.0e-12);
        assertEquals(0.0, model.feedforwardVolts(0.0, 0.0, 0.0, 0.0), 1.0e-12);
        assertEquals(2.4, model.feedforwardVolts(0.0, 1.0, 0.0, 0.0), 1.0e-12);
        assertEquals(-2.4, model.feedforwardVolts(0.0, -1.0, 0.0, 0.0), 1.0e-12);
    }

    @Test
    void rotaryGravityFollowsTheModelPhase() {
        MechanismModel model = new MechanismModel(MotionKind.ROTARY, 0.0, 0.0, 0.0, 0.0, 0.8, 0.4,
                FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, 0.0, 0.0, 0, List.of(), 1.0,
                Map.of(), 0.0, 0);

        assertEquals(0.8 * Math.cos(2.0 * Math.PI * 0.2 + 0.4), model.gravityVolts(0.2), 1.0e-12);
    }

    @Test
    void inertiaFollowsFromKaAndMotorConstantsForRotaryMechanisms() {
        MechanismModel model = model(MotionKind.ROTARY, 0.5);
        MotorConstants kraken = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();

        double expected = 0.5 / (2.0 * Math.PI) * 2 * 10.0 * kraken.torqueConstantNmPerAmp() / kraken.resistance().in(Units.Ohms);
        assertEquals(expected, model.inertia(), 1.0e-12);
        assertEquals(expected, model.momentOfInertia().orElseThrow().in(Units.KilogramSquareMeters), 1.0e-12);
        assertFalse(model(MotionKind.LINEAR, 0.5).momentOfInertia().isPresent());
        assertFalse(MechanismModel.empty(MotionKind.ROTARY).momentOfInertia().isPresent());
    }

    @Test
    void simulationTakesTheIdentifiedInertia() {
        MechanismModel model = model(MotionKind.ROTARY, 0.5);
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

        SimModel armed = SimModel.arm(motor).model(model);

        assertEquals(model.inertia(), armed.momentOfInertia().orElseThrow().in(Units.KilogramSquareMeters), 1.0e-12);
        assertFalse(SimModel.arm(motor).model(model(MotionKind.LINEAR, 0.5)).momentOfInertia().isPresent());
    }

    @Test
    void driftAlertsOnlyOutsideTheToleranceBand() {
        MechanismModel baseline = model(MotionKind.ROTARY, 0.5);
        MechanismModel small = new MechanismModel(MotionKind.ROTARY, 0.5, 0.4, 2.2, 0.5, 0.3, 0.1,
                FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, 0.0, 0.0, 0, List.of(), 1.0,
                Map.of(), 0.0, 0);
        MechanismModel large = new MechanismModel(MotionKind.ROTARY, 0.5, 0.4, 3.0, 0.5, 0.3, 0.1,
                FrictionTable.EMPTY, Double.NaN, Double.NaN, 0.0, 0.0, 0.0, 0, List.of(), 1.0,
                Map.of(), 0.0, 0);

        assertFalse(DriftMonitor.evaluate(baseline, small, 0.25).alert());
        DriftReport report = DriftMonitor.evaluate(baseline, large, 0.25);
        assertTrue(report.alert());
        assertTrue(report.message().contains("kV"), report.message());
        assertFalse(report.message().contains("kA"));
    }

    @Test
    void historyKeepsOnlyTheNewestSnapshots() {
        ModelHistory history = new ModelHistory(3);
        MechanismModel model = MechanismModel.empty(MotionKind.LINEAR);
        for (int i = 0; i < 5; i++) {
            history.add(new ModelHistory.Snapshot(i, "match", model));
        }

        List<ModelHistory.Snapshot> kept = history.snapshots();
        assertEquals(3, kept.size());
        assertEquals(2.0, kept.get(0).timestampSeconds(), 1.0e-12);
        assertEquals(4.0, history.latest().orElseThrow().timestampSeconds(), 1.0e-12);
    }

    @Test
    void storageRoundTripsTheModelValue() {
        InMemoryModelStorage storage = new InMemoryModelStorage();
        MechanismModel model = model(MotionKind.ROTARY, 0.5);

        storage.save("arm", model);

        assertEquals(model, storage.load("arm").orElseThrow());
        assertFalse(storage.load("missing").isPresent());
    }

    @Test
    void frictionTableSortsAndInterpolates() {
        FrictionTable table = new FrictionTable(List.of(1.0, 0.0), List.of(0.4, 0.0));

        assertEquals(0.0, table.at(-5.0), 1.0e-12);
        assertEquals(0.2, table.at(0.5), 1.0e-12);
        assertEquals(0.4, table.at(9.0), 1.0e-12);
        assertEquals(0.0, FrictionTable.EMPTY.at(0.3), 1.0e-12);
    }
}
