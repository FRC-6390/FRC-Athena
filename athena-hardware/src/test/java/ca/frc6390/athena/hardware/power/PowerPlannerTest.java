package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.power.PowerPlanner.Bus;
import ca.frc6390.athena.hardware.power.PowerPlanner.Channel;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class PowerPlannerTest {
    private static final double DT = 0.02;
    private static final double INF = Double.POSITIVE_INFINITY;

    private static PowerPolicy policy(double floor) {
        return PowerPolicy.defaults().floor(Units.Volts.of(floor));
    }

    private static Bus healthy() {
        return new Bus(12.5, 12.5, 0.02);
    }

    @Test
    void noShedWhenPredictedVoltageAboveFloor() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        Channel a = new Channel();
        a.inputs(null, 100.0, INF);
        planner.plan(healthy(), 0.0, DT, List.of(a));
        assertFalse(planner.shedding());
        assertTrue(Double.isInfinite(a.capAmps()));
    }

    @Test
    void sagShedsLowestShareFirstWhenFloorBreached() {
        // Voc 12.5, R 0.02, floor 7 -> budget 275 A. Demand 400 A total -> predicted 4.5 V, below floor.
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        PowerProfile low = PowerProfile.of().share(1.0);
        PowerProfile high = PowerProfile.of().share(3.0);
        Channel a = new Channel();
        Channel b = new Channel();
        a.inputs(low, 200.0, INF);
        b.inputs(high, 200.0, INF);
        Bus bus = new Bus(4.5, 12.5, 0.02);
        planner.plan(bus, 0.0, DT, List.of(a, b));
        assertTrue(planner.shedding());
        // Water fill: the share-3 motor's 200 A demand is below its 3x level, so it is served in full
        // and the share-1 motor gets the remaining 75 A.
        assertEquals(75.0, a.capAmps(), 1e-6);
        assertEquals(200.0, b.capAmps(), 1e-6);
    }

    @Test
    void sheddingHasHysteresisBeforeRelease() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        Channel a = new Channel();
        Bus bus = new Bus(12.5, 12.5, 0.02);
        // Predicted 12.5 - 0.02 * 300 = 6.5 V: shedding.
        a.inputs(null, 300.0, INF);
        planner.plan(bus, 0.0, DT, List.of(a));
        assertTrue(planner.shedding());
        // Predicted 12.5 - 0.02 * 265 = 7.2 V is above the floor but inside the 0.25 V band: still shedding.
        a.inputs(null, 265.0, INF);
        planner.plan(bus, DT, DT, List.of(a));
        assertTrue(planner.shedding());
        // Predicted 12.5 - 0.02 * 260 = 7.3 V: above floor + 0.25, released.
        a.inputs(null, 260.0, INF);
        planner.plan(bus, 2 * DT, DT, List.of(a));
        assertFalse(planner.shedding());
    }

    @Test
    void sustainedCapLimitsDemand() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        PowerProfile p = PowerProfile.of().sustained(Units.Amps.of(60));
        Channel a = new Channel();
        a.inputs(p, 100.0, INF);
        planner.plan(healthy(), 0.0, DT, List.of(a));
        assertEquals(60.0, a.capAmps(), 1e-9);
    }

    @Test
    void burstAllowsHigherCapUntilBudgetSpent() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        PowerProfile p = PowerProfile.of().sustained(Units.Amps.of(60))
                .burst(Units.Amps.of(90), Units.Seconds.of(0.1));
        Channel a = new Channel();
        double now = 0.0;
        // 0.1 s of burst at 20 ms per cycle is five cycles at 90 A, then sustained.
        for (int i = 0; i < 5; i++) {
            a.inputs(p, 100.0, INF);
            planner.plan(healthy(), now, DT, List.of(a));
            assertEquals(90.0, a.capAmps(), 1e-9, "cycle " + i);
            now += DT;
        }
        a.inputs(p, 100.0, INF);
        planner.plan(healthy(), now, DT, List.of(a));
        assertEquals(60.0, a.capAmps(), 1e-9);
    }

    @Test
    void burstBudgetRefillsWhileDrawingSustained() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        PowerProfile p = PowerProfile.of().sustained(Units.Amps.of(60))
                .burst(Units.Amps.of(90), Units.Seconds.of(0.1));
        Channel a = new Channel();
        double now = 0.0;
        for (int i = 0; i < 5; i++) {
            a.inputs(p, 100.0, INF);
            planner.plan(healthy(), now += DT, DT, List.of(a));
        }
        // Rest for 0.2 s: refill is half real time, so 0.1 s is restored.
        for (int i = 0; i < 10; i++) {
            a.inputs(p, 10.0, INF);
            planner.plan(healthy(), now += DT, DT, List.of(a));
        }
        assertEquals(0.1, a.burstBucketSeconds(), 1e-9);
    }

    @Test
    void rampLimitsIncreasesButNotDecreases() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        // 10 A per 0.1 s = 100 A/s. One 20 ms cycle allows 2 A of rise.
        PowerProfile p = PowerProfile.of().rampLimit(Units.Amps.of(10), Units.Seconds.of(0.1));
        Channel a = new Channel();
        a.inputs(p, 0.0, 10.0);
        planner.plan(healthy(), 0.0, DT, List.of(a));
        assertEquals(10.0, a.capAmps(), 1e-9);
        a.inputs(p, 100.0, 80.0);
        planner.plan(healthy(), DT, DT, List.of(a));
        assertEquals(12.0, a.capAmps(), 1e-9);
        planner.plan(healthy(), 2 * DT, DT, List.of(a));
        assertEquals(14.0, a.capAmps(), 1e-9);
        a.inputs(p, 0.0, 5.0);
        planner.plan(healthy(), 3 * DT, DT, List.of(a));
        assertEquals(5.0, a.capAmps(), 1e-9);
    }

    @Test
    void sagProfileReducesCapWhileBusIsLow() {
        PowerPlanner planner = new PowerPlanner(policy(6.0));
        PowerProfile p = PowerProfile.of().onSag(Sag.shed(0.5).below(Units.Volts.of(9)));
        Channel a = new Channel();
        a.inputs(p, 40.0, INF);
        planner.plan(new Bus(8.5, 12.5, 0.02), 0.0, DT, List.of(a));
        assertEquals(20.0, a.capAmps(), 1e-9);
        a.inputs(p, 40.0, INF);
        planner.plan(new Bus(10.0, 12.5, 0.02), DT, DT, List.of(a));
        assertTrue(Double.isInfinite(a.capAmps()));
    }

    @Test
    void staggerHoldsSecondLargeStartUntilSpacingPasses() {
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults().staggerStarts(Units.Seconds.of(0.2)));
        Channel a = new Channel();
        Channel b = new Channel();
        a.inputs(null, 80.0, INF);
        b.inputs(null, 80.0, INF);
        planner.plan(healthy(), 0.0, DT, List.of(a, b));
        assertTrue(Double.isInfinite(a.capAmps()), "first start admitted");
        assertEquals(PowerPlanner.LARGE_CURRENT_AMPS, b.capAmps(), 1e-9, "second start held");
        // Still held inside the window.
        planner.plan(healthy(), 0.1, DT, List.of(a, b));
        assertEquals(PowerPlanner.LARGE_CURRENT_AMPS, b.capAmps(), 1e-9);
        // Admitted once 0.2 s has passed since the first start.
        planner.plan(healthy(), 0.25, DT, List.of(a, b));
        assertTrue(Double.isInfinite(b.capAmps()));
    }

    @Test
    void staggerDisabledAdmitsEveryStart() {
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults());
        Channel a = new Channel();
        Channel b = new Channel();
        a.inputs(null, 80.0, INF);
        b.inputs(null, 80.0, INF);
        planner.plan(healthy(), 0.0, DT, List.of(a, b));
        assertTrue(Double.isInfinite(b.capAmps()));
    }

    @Test
    void declaredCapAppliesWhenNoBudgetOrProfile() {
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults());
        Channel a = new Channel();
        a.inputs(null, 10.0, 45.0);
        planner.plan(healthy(), 0.0, DT, List.of(a));
        assertEquals(45.0, a.capAmps(), 1e-9);
    }
}
