package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;

class PowerPlannerElectricalTest {
    private static final double INF = Double.POSITIVE_INFINITY;

    @Test
    void electricalSolveMatchesBoundedCallbackAcrossRandomFollowerFamiliesAndCaps() {
        Random random = new Random(63902026L);
        for (int trial = 0; trial < 1200; trial++) {
            List<PowerPlanner.Channel> electrical = new ArrayList<>();
            List<PowerPlanner.Channel> generic = new ArrayList<>();
            int groupCount = 1 + random.nextInt(4);
            for (int group = 0; group < groupCount; group++) {
                PowerProfile profile = PowerProfile.of().share(0.1 + 4.0 * random.nextDouble());
                int members = 1 + random.nextInt(5);
                for (int member = 0; member < members; member++) {
                    double resistance = 0.01 + 0.3 * random.nextDouble();
                    double zeroVolts = -15.0 + 30.0 * random.nextDouble();
                    double slopeVolts = -25.0 + 50.0 * random.nextDouble();
                    double emf = -15.0 + 30.0 * random.nextDouble();
                    double initialCurrent = (zeroVolts - emf) / resistance;
                    double currentSlope = slopeVolts / resistance;
                    double cap = random.nextDouble() < 0.45 ? INF : 1.0 + 300.0 * random.nextDouble();
                    var model = new PowerPlanner.ElectricalDemandModel().inputs(initialCurrent,
                            currentSlope, zeroVolts, slopeVolts, 12.0);
                    PowerPlanner.Channel fast = channel(profile, cap, model);
                    PowerPlanner.Channel reference = channel(profile, cap,
                            new PowerPlanner.DemandModel(initialCurrent, currentSlope, model::positiveSupplyAt));
                    if (random.nextDouble() < 0.15) {
                        fast.accountedLoad(false);
                        reference.accountedLoad(false);
                    }
                    electrical.add(fast);
                    generic.add(reference);
                }
            }
            double budget = 1.0 + 400.0 * random.nextDouble();
            var bus = new PowerPlanner.Bus(12.0, 12.0, 5.0 / budget);
            var policy = PowerPolicy.defaults().floor(Units.Volts.of(7.0));
            var fastPlanner = new PowerPlanner(policy);
            var referencePlanner = new PowerPlanner(policy);
            fastPlanner.plan(bus, 0.0, 0.02, electrical);
            referencePlanner.plan(bus, 0.0, 0.02, generic);
            assertEquals(referencePlanner.shedding(), fastPlanner.shedding(), "trial " + trial);
            for (int member = 0; member < electrical.size(); member++) {
                var fast = electrical.get(member);
                var reference = generic.get(member);
                String label = "trial " + trial + ", member " + member;
                assertEquals(reference.couplingFeasible(), fast.couplingFeasible(), label);
                if (fast.couplingFeasible()) {
                    assertEquals(reference.couplingScale(), fast.couplingScale(), 2e-8, label);
                    assertEquals(reference.capAmps(), fast.capAmps(), 2e-5, label);
                }
            }
        }
    }

    @Test
    void nonmonotoneFollowerCostFindsRightHandRootAcrossRegenerationBreakpoints() {
        // V=12*scale, I=120*scale for the root; the follower's signed native offset
        // has V=12*scale-6, I=120*scale-30. Positive supply clips off its regen band.
        Object family = new Object();
        var rootModel = new PowerPlanner.ElectricalDemandModel().inputs(0.0, 120.0, 0.0, 12.0, 12.0);
        var followerModel = new PowerPlanner.ElectricalDemandModel().inputs(-30.0, 120.0, -6.0, 12.0, 12.0);
        var root = channel(null, INF, rootModel);
        var follower = channel(null, INF, followerModel);
        root.couplingGroup(family);
        follower.couplingGroup(family);
        var planner = new PowerPlanner(PowerPolicy.defaults().floor(Units.Volts.of(7.0)));
        planner.plan(new PowerPlanner.Bus(12.0, 12.0, 5.0 / 12.0), 0.0, 0.02, List.of(root, follower));
        assertTrue(root.couplingFeasible());
        assertEquals(Math.sqrt(0.1), root.couplingScale(), 1e-12);
        assertEquals(root.couplingScale(), follower.couplingScale());
        assertEquals(12.0, rootModel.positiveSupplyAt(root.couplingScale())
                + followerModel.positiveSupplyAt(follower.couplingScale()), 1e-10);
    }

    @Test
    void minimumAboveAllocationRejectsCoupledFamilyInsteadOfGrantingZeroScale() {
        Object family = new Object();
        var first = channel(null, INF, new PowerPlanner.ElectricalDemandModel()
                .inputs(0.0, 100.0, 0.0, 10.0, 10.0));
        var second = channel(null, INF, new PowerPlanner.ElectricalDemandModel()
                .inputs(-100.0, 100.0, -10.0, 10.0, 10.0));
        first.couplingGroup(family);
        second.couplingGroup(family);
        var planner = new PowerPlanner(PowerPolicy.defaults().floor(Units.Volts.of(7.0)));
        planner.plan(new PowerPlanner.Bus(12.0, 12.0, 5.0 / 40.0), 0.0, 0.02, List.of(first, second));
        assertFalse(first.couplingFeasible(), "100*x^2+100*(1-x)^2 has a 50 A minimum");
        assertFalse(second.couplingFeasible());
    }

    @Test
    void reusedModelUsesCurrentCycleCoefficientsAndAliasesNeverAddBatteryCost() {
        var model = new PowerPlanner.ElectricalDemandModel().inputs(0.0, 100.0, 0.0, 12.0, 12.0);
        var root = channel(null, INF, model);
        Object family = new Object();
        root.couplingGroup(family);
        var alias = channel(null, 30.0, model);
        alias.couplingGroup(family);
        alias.accountedLoad(false);
        var planner = new PowerPlanner(PowerPolicy.defaults().floor(Units.Volts.of(7.0)));
        var bus = new PowerPlanner.Bus(12.0, 12.0, 0.1);
        planner.plan(bus, 0.0, 0.02, List.of(root, alias));
        assertEquals(0.3, root.couplingScale(), 1e-12);
        model.inputs(0.0, 200.0, 0.0, 12.0, 12.0);
        root.inputs(null, 200.0, INF, 200.0, value -> value, 1.0, model);
        alias.inputs(null, 200.0, 160.0, 200.0, value -> value, 1.0, model);
        planner.plan(bus, 0.02, 0.02, List.of(root, alias));
        assertEquals(0.5, root.couplingScale(), 1e-12, "50 A / (200 A*x^2), alias is not another load");
    }

    @Test
    void hugeOffsetFullRequestAtZeroSupplyNeedsNoArtificialReduction() {
        var model = new PowerPlanner.ElectricalDemandModel().inputs(-1e150, 1e150, -1e150, 1e150, 1.0);
        var motor = channel(null, INF, model);
        var planner = new PowerPlanner(PowerPolicy.defaults().floor(Units.Volts.of(7.0)));
        planner.plan(new PowerPlanner.Bus(12.0, 12.0, 0.1), 0.0, 0.02, List.of(motor));
        assertTrue(motor.couplingFeasible());
        assertEquals(1.0, motor.couplingScale(), "zero demand at full scale needs no solve");
    }

    @Test
    void hugeAffineOffsetAndNarrowFeasibleRangeRemainWithinActualBudget() {
        double offset = 1e12, slope = offset + 1.0, budget = 0.01;
        var model = new PowerPlanner.ElectricalDemandModel().inputs(-offset, slope, -offset, slope, 12.0);
        var motor = channel(null, INF, model);
        var planner = new PowerPlanner(PowerPolicy.defaults().floor(Units.Volts.of(7.0)));
        planner.plan(new PowerPlanner.Bus(12.0, 12.0, 5.0 / budget), 0.0, 0.02, List.of(motor));
        assertTrue(motor.couplingFeasible());
        double expected = (offset + Math.sqrt(12.0 * budget)) / slope;
        assertEquals(expected, motor.couplingScale(), 2.0 * Math.ulp(expected));
        assertTrue(model.positiveSupplyAt(motor.couplingScale()) <= budget,
                "rounded analytic roots must satisfy the factored electrical demand");
    }

    @Test
    void zeroBudgetFindsRightEdgeOfExtremelyNarrowRegenerationPlateau() {
        double offset = Math.scalb(1.0, 40), slope = 2.0 * offset;
        var model = new PowerPlanner.ElectricalDemandModel().inputs(-offset + 0.25, slope,
                -offset, slope, 12.0);
        var motor = channel(null, INF, model);
        var planner = new PowerPlanner(PowerPolicy.defaults().floor(Units.Volts.of(7.0)));
        planner.plan(new PowerPlanner.Bus(6.0, 6.0, 0.1), 0.0, 0.02, List.of(motor));
        assertTrue(motor.couplingFeasible());
        assertEquals(0.5, motor.couplingScale(), 0.0);
        assertEquals(0.0, model.positiveSupplyAt(motor.couplingScale()));
    }

    @Test
    void overflowedElectricalDemandFailsClosedWithoutNaN() {
        var model = new PowerPlanner.ElectricalDemandModel().inputs(1e200, 1e200, 1e200, 1e200, 1.0);
        var motor = channel(null, INF, model);
        var planner = new PowerPlanner(PowerPolicy.defaults().floor(Units.Volts.of(7.0)));
        planner.plan(new PowerPlanner.Bus(12.0, 12.0, 0.1), 0.0, 0.02, List.of(motor));
        assertFalse(motor.couplingFeasible());
        assertEquals(0.0, motor.couplingScale());
    }

    private static PowerPlanner.Channel channel(PowerProfile profile, double cap, PowerPlanner.ForwardDemand model) {
        var channel = new PowerPlanner.Channel();
        channel.inputs(profile, Math.abs(model.currentAt(1.0)), cap, model.positiveSupplyAt(1.0),
                value -> value, 1.0, model);
        return channel;
    }
}
