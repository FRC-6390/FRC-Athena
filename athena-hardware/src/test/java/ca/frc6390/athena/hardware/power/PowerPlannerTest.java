package ca.frc6390.athena.hardware.power;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.power.PowerPlanner.Bus;
import ca.frc6390.athena.hardware.power.PowerPlanner.Channel;
import edu.wpi.first.units.Units;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
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
    void sagResponseGrowsAcrossItsBandBelowTheThreshold() {
        PowerProfile p = PowerProfile.of().onSag(Sag.shed(0.5).below(Units.Volts.of(9)));
        double[][] cases = {{9.5, Double.POSITIVE_INFINITY}, {8.5, 40.0 * (1.0 - 0.5 * (0.5 / 0.75))}, {8.25, 20.0}, {7.0, 20.0}};
        for (double[] c : cases) {
            PowerPlanner planner = new PowerPlanner(policy(6.0));
            Channel a = new Channel();
            a.inputs(p, 40.0, INF);
            planner.plan(new Bus(c[0], 12.5, 0.02), 0.0, DT, List.of(a));
            assertEquals(c[1], a.capAmps(), 1e-9, "bus " + c[0] + " V");
        }
    }

    @Test
    void sagCutsSlewDownAndRecoverWithoutSnapping() {
        PowerPlanner planner = new PowerPlanner(policy(6.0));
        PowerProfile p = PowerProfile.of().onSag(Sag.shed(0.5).below(Units.Volts.of(9)));
        Channel a = new Channel();
        double now = 0.0;
        double previous = Double.NaN;
        for (int i = 0; i < 60; i++, now += DT) {
            double bus = i < 5 ? 10.0 : i < 30 ? 7.5 : 10.0;
            a.inputs(p, 40.0, INF);
            planner.plan(new Bus(bus, 12.5, 0.02), now, DT, List.of(a));
            double cap = a.capAmps();
            if (Double.isFinite(previous) && Double.isFinite(cap)) {
                double change = cap - previous;
                assertTrue(change >= -PowerPlanner.SOFT_CUT_AMPS_PER_SECOND * DT - 1e-9, "cut too fast at " + i);
                assertTrue(change <= PowerPlanner.RECOVERY_AMPS_PER_SECOND * DT + 1e-9, "recovered too fast at " + i);
            }
            previous = cap;
        }
        assertTrue(Double.isInfinite(a.capAmps()), "uncapped again once the bus recovered");
    }

    @Test
    void atTheFloorASagCutIsImmediate() {
        PowerPlanner planner = new PowerPlanner(policy(7.5));
        PowerProfile p = PowerProfile.of().sustained(Units.Amps.of(60)).onSag(Sag.shed(0.5).below(Units.Volts.of(9)));
        Channel a = new Channel();
        a.inputs(p, 60.0, INF);
        planner.plan(new Bus(10.0, 12.5, 0.02), 0.0, DT, List.of(a));
        assertEquals(60.0, a.capAmps(), 1e-9);
        // The filter needs time to follow, but the floor allows the cut it asks for at once.
        for (int i = 1; i <= 10; i++) {
            a.inputs(p, 60.0, INF);
            planner.plan(new Bus(7.4, 12.5, 0.02), i * DT, DT, List.of(a));
        }
        assertEquals(30.0, a.capAmps(), 1e-9);
    }

    /**
     * A load that pulls the bus below its sag threshold. An on/off sag response flips every cycle (cut, the bus
     * recovers above the threshold, release, sag again); the proportional, slewed response settles.
     */
    @Test
    void sagResponseSettlesInsteadOfOscillatingAgainstTheBattery() {
        PowerPlanner planner = new PowerPlanner(policy(6.0));
        PowerProfile p = PowerProfile.of().onSag(Sag.shed(0.5).below(Units.Volts.of(9)));
        Channel a = new Channel();
        double ohms = 0.03;
        double demand = 130.0;
        double drawn = 0.0;
        int reversals = 0;
        double lastChange = 0.0;
        double previous = Double.NaN;
        for (int i = 0; i < 150; i++) {
            double bus = 12.5 - ohms * drawn;
            a.inputs(p, demand, INF);
            planner.plan(new Bus(bus, 12.5, ohms), i * DT, DT, List.of(a));
            drawn = Math.min(demand, a.capAmps());
            if (i >= 50 && Double.isFinite(previous)) {
                double change = drawn - previous;
                if (Math.abs(change) > 0.5 && Math.signum(change) != Math.signum(lastChange) && lastChange != 0.0) {
                    reversals++;
                }
                if (Math.abs(change) > 0.5) lastChange = change;
            }
            previous = drawn;
        }
        assertTrue(reversals <= 2, "current reversed " + reversals + " times after settling");
        assertTrue(drawn < demand && drawn > demand * 0.5, "settled at a partial shed: " + drawn);
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

    @Test
    void separateSupplyDemandAvoidsFalseSheddingAtLowDuty() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        Channel a = new Channel();
        a.inputs(null, 300.0, INF, 25.0, supply -> supply * 12.0, 1.0);
        planner.plan(healthy(), 0.0, DT, List.of(a));
        assertFalse(planner.shedding());
        assertEquals(25.0, a.supplyDemandAmps());
        assertTrue(Double.isInfinite(a.capAmps()));
    }

    @Test
    void positiveSupplyAllocationsAreConvertedWithoutClampingIdleOrRegenerativeChannels() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        Channel large = new Channel();
        Channel small = new Channel();
        Channel idle = new Channel();
        large.inputs(null, 600.0, INF, 300.0, supply -> supply * 2.0, 1.0);
        small.inputs(null, 10.0, INF, 5.0, supply -> supply * 2.0, 1.0);
        idle.inputs(null, 100.0, INF, 0.0, supply -> { throw new AssertionError("idle conversion"); }, 1.0);
        planner.plan(healthy(), 0.0, DT, List.of(large, small, idle));
        assertTrue(planner.shedding());
        assertEquals(540.0, large.capAmps(), 1e-9);
        assertTrue(large.supplyLimited());
        assertEquals(10.0, small.capAmps(), "positive loads keep their allocation while shedding");
        assertTrue(Double.isInfinite(idle.capAmps()));
    }

    @Test
    void reachingTheAllocatedCurrentDoesNotDropTheCapBeforeHysteresisReleases() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        Channel a = new Channel();
        a.inputs(null, 300.0, INF, 300.0, supply -> supply, 1.0);
        planner.plan(new Bus(6.5, 12.5, 0.02), 0.0, DT, List.of(a));
        assertEquals(275.0, a.capAmps());
        a.inputs(null, 275.0, INF, 275.0, supply -> supply, 1.0);
        planner.plan(new Bus(7.0, 12.5, 0.02), DT, DT, List.of(a));
        assertTrue(planner.shedding());
        assertEquals(275.0, a.capAmps(), "do not release the cap and alternate between high draw and shedding");
        a.inputs(null, 260.0, INF, 260.0, supply -> supply, 1.0);
        planner.plan(new Bus(7.3, 12.5, 0.02), 2 * DT, DT, List.of(a));
        assertFalse(planner.shedding());
        assertEquals(INF, a.capAmps());
    }

    @Test
    void zeroAllocationAndZeroThermalFractionAreActiveWindingLimits() {
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        Channel a = new Channel();
        a.inputs(null, 30.0, INF, 20.0, supply -> supply, 1.0);
        planner.plan(new Bus(6.0, 6.0, 0.02), 0.0, DT, List.of(a));
        assertEquals(0.0, a.capAmps());
        assertTrue(a.supplyLimited());
        a.inputs(PowerProfile.of().sustained(Units.Amps.of(60)), 30.0, INF,
                0.0, supply -> supply, 0.0);
        planner.plan(healthy(), DT, DT, List.of(a));
        assertEquals(0.0, a.capAmps());
    }

    private static Channel modeled(PowerProfile profile, double signedCurrent, double cap,
            DoubleUnaryOperator supplyAtScale) {
        Channel channel = new Channel();
        channel.inputs(profile, Math.abs(signedCurrent), cap, supplyAtScale.applyAsDouble(1.0),
                supply -> supply, 1.0, new PowerPlanner.DemandModel(0.0, signedCurrent, supplyAtScale));
        return channel;
    }

    @Test
    void sharedProfilePreservesUnequalSignedTorqueRequestsWhenOneMemberIsLimited() {
        PowerProfile profile = PowerProfile.of();
        Channel first = modeled(profile, 40.0, 20.0, scale -> 40.0 * scale * scale);
        Channel second = modeled(profile, -30.0, INF, scale -> 30.0 * scale * scale);
        Channel third = modeled(profile, 10.0, INF, scale -> 10.0 * scale * scale);
        new PowerPlanner(policy(7.0)).plan(healthy(), 0.0, DT, List.of(first, second, third));
        assertEquals(0.5, first.couplingScale());
        assertEquals(0.5, second.couplingScale());
        assertEquals(0.5, third.couplingScale());
        assertEquals(20.0, first.capAmps());
        assertEquals(15.0, second.capAmps());
        assertEquals(5.0, third.capAmps());
        assertTrue(first.couplingFeasible());
    }

    @Test
    void equalSettingsAndNullProfilesRemainIndependent() {
        PowerProfile firstProfile = PowerProfile.of().share(2.0);
        PowerProfile equalSettings = PowerProfile.of().share(2.0);
        Channel limited = modeled(firstProfile, 10.0, 2.0, scale -> 10.0 * scale);
        Channel independent = modeled(equalSettings, 10.0, INF, scale -> 10.0 * scale);
        Channel nullLimited = modeled(null, 10.0, 3.0, scale -> 10.0 * scale);
        Channel nullIndependent = modeled(null, 10.0, INF, scale -> 10.0 * scale);
        new PowerPlanner(policy(7.0)).plan(healthy(), 0.0, DT,
                List.of(limited, independent, nullLimited, nullIndependent));
        assertEquals(0.2, limited.couplingScale());
        assertEquals(1.0, independent.couplingScale());
        assertEquals(INF, independent.capAmps());
        assertEquals(0.3, nullLimited.couplingScale());
        assertEquals(1.0, nullIndependent.couplingScale());
        assertEquals(INF, nullIndependent.capAmps());
    }

    @Test
    void reusingOneCopyCreatesASeparateCouplingGroup() {
        PowerProfile original = PowerProfile.of();
        PowerProfile copy = original.copy();
        Channel originalMotor = modeled(original, 10.0, INF, scale -> 10.0 * scale);
        Channel copiedFirst = modeled(copy, 10.0, 4.0, scale -> 10.0 * scale);
        Channel copiedSecond = modeled(copy, -5.0, INF, scale -> 5.0 * scale);
        new PowerPlanner(policy(7.0)).plan(healthy(), 0.0, DT,
                List.of(originalMotor, copiedFirst, copiedSecond));
        assertEquals(1.0, originalMotor.couplingScale());
        assertEquals(0.4, copiedFirst.couplingScale());
        assertEquals(0.4, copiedSecond.couplingScale());
        assertEquals(2.0, copiedSecond.capAmps());
    }

    @Test
    void sharedProfilesAdmitAllMembersTogetherDespiteInterleavedChannelOrder() {
        PowerProfile group = PowerProfile.of();
        Channel first = new Channel();
        Channel last = new Channel();
        Channel other = new Channel();
        first.inputs(group, 80.0, INF);
        last.inputs(group, 80.0, INF);
        other.inputs(group.copy(), 80.0, INF);
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults().staggerStarts(Units.Seconds.of(0.2)));
        planner.plan(healthy(), 0.0, DT, List.of(first, other, last));
        assertEquals(INF, first.capAmps());
        assertEquals(INF, last.capAmps());
        assertEquals(20.0, other.capAmps());
        planner.plan(healthy(), 0.1, DT, List.of(first, other, last));
        assertEquals(20.0, other.capAmps());
        planner.plan(healthy(), 0.21, DT, List.of(first, other, last));
        assertEquals(INF, other.capAmps());
    }

    @Test
    void copiedStartupMembershipIsIndependentUnlessExplicitlyRejoined() {
        PowerProfile drive = PowerProfile.of().startTogether("swerve");
        PowerProfile steer = drive.copy().startTogether("swerve");
        PowerProfile independent = drive.copy();
        Channel first = new Channel();
        Channel steering = new Channel();
        Channel other = new Channel();
        first.inputs(drive, 80.0, INF);
        steering.inputs(steer, 80.0, INF);
        other.inputs(independent, 80.0, INF);
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults().staggerStarts(Units.Seconds.of(0.2)));
        planner.plan(healthy(), 0.0, DT, List.of(first, other, steering));
        assertEquals(INF, first.capAmps());
        assertEquals(INF, steering.capAmps());
        assertEquals(20.0, other.capAmps());
    }

    @Test
    void startupGroupsJoinWholeCommandFamiliesTransitively() {
        Object physicalFamily = new Object();
        Channel bridgeA = new Channel();
        Channel bridgeB = new Channel();
        Channel externalA = new Channel();
        Channel externalB = new Channel();
        Channel unrelated = new Channel();
        bridgeA.inputs(PowerProfile.of().startTogether("a"), 80.0, INF);
        bridgeB.inputs(PowerProfile.of().startTogether("b"), 80.0, INF);
        externalA.inputs(PowerProfile.of().startTogether("a"), 80.0, INF);
        externalB.inputs(PowerProfile.of().startTogether("b"), 80.0, INF);
        unrelated.inputs(null, 80.0, INF);
        bridgeA.couplingGroup(physicalFamily);
        bridgeB.couplingGroup(physicalFamily);
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults().staggerStarts(Units.Seconds.of(0.2)));
        planner.plan(new Bus(24.0, 24.0, 0.02), 0.0, DT,
                List.of(externalA, unrelated, externalB, bridgeA, bridgeB));
        for (Channel member : List.of(bridgeA, bridgeB, externalA, externalB)) {
            assertEquals(INF, member.capAmps());
        }
        assertEquals(20.0, unrelated.capAmps());
    }

    @Test
    void groupedWeightsSumPhysicalMemberShares() {
        PowerProfile drive = PowerProfile.of().share(4.0);
        Channel first = modeled(drive, 6.0, INF, scale -> 6.0 * scale);
        Channel second = modeled(drive, 4.0, INF, scale -> 4.0 * scale);
        Channel other = modeled(PowerProfile.of(), 10.0, INF, scale -> 10.0 * scale);
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        planner.plan(new Bus(1.0, 16.0, 1.0), 0.0, DT, List.of(first, second, other));
        assertTrue(planner.shedding());
        assertEquals(0.8, first.couplingScale(), 1e-12);
        assertEquals(0.8, second.couplingScale(), 1e-12);
        assertEquals(4.8, first.capAmps(), 1e-12);
        assertEquals(3.2, second.capAmps(), 1e-12);
        assertEquals(1.0, other.capAmps(), 1e-12);
    }

    @Test
    void commonFactorSolvesQuadraticSupplyWithoutScalingVoltages() {
        PowerProfile profile = PowerProfile.of();
        Channel first = modeled(profile, 4.0, INF, scale -> 16.0 * scale * scale);
        Channel second = modeled(profile, -3.0, INF, scale -> 9.0 * scale * scale);
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        planner.plan(new Bus(1.0, 16.0, 1.0), 0.0, DT, List.of(first, second));
        assertEquals(0.6, first.couplingScale(), 1e-12);
        assertEquals(0.6, second.couplingScale(), 1e-12);
        assertEquals(2.4, first.capAmps(), 1e-12);
        assertEquals(1.8, second.capAmps(), 1e-12);
        double allocated = 25.0 * first.couplingScale() * first.couplingScale();
        assertTrue(allocated <= 9.0 + 1e-12);
    }

    @Test
    void neutralMemberDoesNotAcquireAZeroCurrentClampFromPeersDerating() {
        PowerProfile profile = PowerProfile.of();
        Channel active = modeled(profile, 10.0, 5.0, scale -> 10.0 * scale);
        Channel neutral = modeled(profile, 0.0, INF, scale -> 0.0);
        new PowerPlanner(policy(7.0)).plan(healthy(), 0.0, DT, List.of(active, neutral));
        assertEquals(0.5, neutral.couplingScale());
        assertEquals(INF, neutral.capAmps());
    }

    private static List<Channel> offsetFollowerFamily(double rootCap) {
        PowerProfile profile = PowerProfile.of();
        Channel root = modeled(profile, 10.0, rootCap, scale -> 10.0 * scale * scale);
        Channel follower = new Channel();
        follower.inputs(profile, 2.0, 1.0, 4.0, supply -> supply, 1.0,
                new PowerPlanner.DemandModel(-8.0, 10.0,
                        scale -> 100.0 * (scale - 0.8) * (scale - 0.8)));
        return List.of(root, follower);
    }

    @Test
    void followerCurrentAtZeroScaleUsesItsFeasibleIntervalAndConvexSupplyMinimum() {
        List<Channel> family = offsetFollowerFamily(INF);
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        planner.plan(new Bus(1.0, 13.0, 1.0), 0.0, DT, family);
        double expected = (160.0 + Math.sqrt(80.0)) / 220.0;
        assertTrue(family.get(0).couplingFeasible());
        assertEquals(expected, family.get(0).couplingScale(), 1e-9);
        assertEquals(expected, family.get(1).couplingScale(), 1e-9);
        assertTrue(Math.abs(-8.0 + 10.0 * expected) <= 1.0);
        assertEquals(Math.abs(-8.0 + 10.0 * expected), family.get(1).capAmps(), 1e-9);
    }

    @Test
    void incompatibleFollowerAndRootCurrentLimitsAreReportedAsInfeasible() {
        List<Channel> family = offsetFollowerFamily(3.0);
        new PowerPlanner(policy(7.0)).plan(healthy(), 0.0, DT, family);
        assertFalse(family.get(0).couplingFeasible());
        assertFalse(family.get(1).couplingFeasible());
    }

    @Test
    void budgetBelowMinimumFeasibleFollowerPowerIsReportedAsInfeasible() {
        List<Channel> family = offsetFollowerFamily(INF);
        new PowerPlanner(policy(7.0)).plan(new Bus(1.0, 12.0, 1.0), 0.0, DT, family);
        assertFalse(family.get(0).couplingFeasible());
        assertFalse(family.get(1).couplingFeasible());
    }

    @Test
    void physicalAliasesKeepTheirLimitsWithoutDuplicatingSupplyDemandOrShare() {
        Object family = new Object();
        Channel owner = modeled(PowerProfile.of().share(4.0), 10.0, INF, scale -> 10.0 * scale);
        Channel alias = modeled(PowerProfile.of().share(4.0), 10.0, INF, scale -> 30.0 * scale);
        Channel unrelated = modeled(PowerProfile.of(), 10.0, INF, scale -> 10.0 * scale);
        owner.couplingGroup(family);
        alias.couplingGroup(family);
        alias.accountedLoad(false);
        new PowerPlanner(policy(7.0)).plan(new Bus(1.0, 17.0, 1.0), 0.0, DT,
                List.of(owner, alias, unrelated));
        assertEquals(0.8, owner.couplingScale(), 1e-12);
        assertEquals(0.8, alias.couplingScale(), 1e-12);
        assertEquals(0.2, unrelated.couplingScale(), 1e-12);
        assertEquals(0.0, alias.shareWeight());

        alias.inputs(PowerProfile.of(), 10.0, 3.0, 30.0, supply -> supply, 1.0,
                new PowerPlanner.DemandModel(0.0, 10.0, scale -> 30.0 * scale));
        new PowerPlanner(policy(7.0)).plan(healthy(), 0.0, DT, List.of(owner, alias));
        assertEquals(0.3, owner.couplingScale(), 1e-12);
        assertEquals(0.3, alias.couplingScale(), 1e-12);
    }

    @Test
    void zeroAccountingGroupRemainsFiniteAndDoesNotConsumeBatteryAllocation() {
        Channel alias = modeled(PowerProfile.of(), 10.0, INF, scale -> 100.0 * scale);
        alias.accountedLoad(false);
        Channel actual = modeled(PowerProfile.of(), 10.0, INF, scale -> 10.0 * scale);
        new PowerPlanner(policy(7.0)).plan(new Bus(1.0, 12.0, 1.0), 0.0, DT, List.of(alias, actual));
        assertEquals(1.0, alias.couplingScale());
        assertTrue(alias.couplingFeasible());
        assertEquals(0.5, actual.couplingScale(), 1e-12);
    }

    @Test
    void switchingToADetachedCopyDoesNotInheritOldBurstHistory() {
        PowerProfile profile = PowerProfile.of().sustained(Units.Amps.of(20))
                .burst(Units.Amps.of(40), Units.Seconds.of(0.02));
        Channel channel = new Channel();
        PowerPlanner planner = new PowerPlanner(policy(7.0));
        channel.inputs(profile, 50.0, INF);
        planner.plan(healthy(), 0.0, DT, List.of(channel));
        planner.plan(healthy(), DT, DT, List.of(channel));
        assertEquals(20.0, channel.capAmps());
        channel.inputs(profile.copy(), 50.0, INF);
        planner.plan(healthy(), 2 * DT, DT, List.of(channel));
        assertEquals(40.0, channel.capAmps());
    }

    @Test
    void disappearingAndReturningChannelMustBeAdmittedAgain() {
        Channel first = new Channel();
        Channel later = new Channel();
        first.inputs(null, 80.0, INF);
        later.inputs(null, 80.0, INF);
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults().staggerStarts(Units.Seconds.of(0.2)));
        planner.plan(healthy(), 0.0, DT, List.of(first));
        planner.plan(healthy(), 0.21, DT, List.of(later));
        planner.plan(healthy(), 0.22, DT, List.of(first, later));
        assertEquals(20.0, first.capAmps());
        assertEquals(INF, later.capAmps());
    }

    @Test
    void syntheticVoltageConstraintDoesNotConsumeOrReceiveAStartCurrentThreshold() {
        Channel virtualVoltage = new Channel();
        virtualVoltage.inputs(null, 24.0, 24.0, 0.0, value -> value, 1.0,
                new PowerPlanner.DemandModel(0.0, 24.0, ignored -> 0.0));
        virtualVoltage.accountedLoad(false);
        virtualVoltage.startupParticipant(false);
        Channel actual = new Channel();
        actual.inputs(null, 80.0, INF);
        PowerPlanner planner = new PowerPlanner(PowerPolicy.defaults().staggerStarts(Units.Seconds.of(0.2)));
        planner.plan(healthy(), 0.0, DT, List.of(virtualVoltage, actual));
        assertEquals(24.0, virtualVoltage.capAmps());
        assertEquals(INF, actual.capAmps());
    }
}
