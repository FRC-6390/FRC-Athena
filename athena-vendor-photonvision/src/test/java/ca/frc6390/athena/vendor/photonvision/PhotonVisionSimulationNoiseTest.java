package ca.frc6390.athena.vendor.photonvision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Milliseconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.runtime.VisionSimNoise;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Statistical checks of the realistic PhotonVision simulation model. Every run uses a fixed seed, so the measured
 * statistics are reproducible and the bands are fixed.
 */
class PhotonVisionSimulationNoiseTest {
    private static final PoseSnapshot TRUTH = new PoseSnapshot(0.0, 0.0, 0.0);
    private static final double DT = 1.0 / 60.0;
    private static final long SEED = 42L;

    @Test
    void multiTagErrorsHaveDistanceScaledZeroMeanNoise() {
        for (double distance : List.of(2.0, 4.0)) {
            List<Sample> samples = run(multiTagField(distance), noise(), 120.0);
            double dAvg = Math.hypot(distance, 0.25);
            double sigmaXy = noise().translationStdAt1m().in(Meters) * dAvg * dAvg;
            double sigmaHeading = noise().headingStdAt1m().in(Radians) * dAvg * dAvg;

            Stats x = Stats.of(samples, s -> s.pose().pose().xMeters());
            Stats y = Stats.of(samples, s -> s.pose().pose().yMeters());
            Stats heading = Stats.of(samples, s -> wrap(s.pose().pose().headingRadians()));

            assertMeanNearZero(x, "multi x at " + distance + " m");
            assertMeanNearZero(y, "multi y at " + distance + " m");
            assertMeanNearZero(heading, "multi heading at " + distance + " m");
            assertWithin(x.std(), sigmaXy, 0.08, "multi x std at " + distance + " m");
            assertWithin(y.std(), sigmaXy, 0.08, "multi y std at " + distance + " m");
            assertWithin(heading.std(), sigmaHeading, 0.08, "multi heading std at " + distance + " m");
            for (Sample sample : samples) {
                assertEquals(sigmaXy, sample.pose().stdDevs().raw().xMeters(), 1.0e-9);
                assertEquals(sigmaHeading, sample.pose().stdDevs().raw().headingRadians(), 1.0e-9);
                assertEquals(2, sample.pose().targetCount());
                assertEquals(dAvg, sample.pose().averageTargetDistanceMeters(), 1.0e-9);
                assertEquals(0.02, sample.pose().ambiguity(), 1.0e-9);
            }
        }
    }

    @Test
    void singleTagErrorsAreLargerAndGrowWithDistanceSquared() {
        Stats near = null;
        Stats far = null;
        for (double distance : List.of(2.0, 4.0)) {
            List<Sample> samples = run(singleTagField(distance), noise().ambiguityFlipProbability(0.0), 120.0);
            double sigmaXy = noise().singleTagMultiplier() * noise().translationStdAt1m().in(Meters) * distance * distance;
            double sigmaHeading = noise().headingStdAt1m().in(Radians) * noise().singleTagMultiplier()
                    * distance * distance;
            Stats x = Stats.of(samples, s -> s.pose().pose().xMeters());
            Stats y = Stats.of(samples, s -> s.pose().pose().yMeters());
            Stats heading = Stats.of(samples, s -> wrap(s.pose().pose().headingRadians()));

            assertMeanNearZero(x, "single x at " + distance + " m");
            assertMeanNearZero(heading, "single heading at " + distance + " m");
            assertWithin(x.std(), sigmaXy, 0.08, "single x std at " + distance + " m");
            assertWithin(y.std(), sigmaXy, 0.08, "single y std at " + distance + " m");
            assertWithin(heading.std(), sigmaHeading, 0.08, "single heading std at " + distance + " m");
            for (Sample sample : samples) {
                assertEquals(1, sample.pose().targetCount());
                assertEquals(0.05 + 0.02 * distance, sample.pose().ambiguity(), 1.0e-9);
            }
            if (distance == 2.0) {
                near = x;
            } else {
                far = x;
            }
        }
        // Doubling the distance quadruples the standard deviation (d^2 model): 4 +- 10 %.
        double ratio = far.std() / near.std();
        assertTrue(ratio > 3.6 && ratio < 4.4, () -> "distance ratio " + ratio);
    }

    @Test
    void singleTagFramesFlipToAMirroredPoseWithHighAmbiguity() {
        List<Sample> samples = run(singleTagField(2.0), noise().ambiguityFlipProbability(0.2), 120.0);
        long flipped = samples.stream().filter(s -> s.pose().ambiguity() > 0.5).count();
        double fraction = (double) flipped / samples.size();

        assertTrue(fraction > 0.17 && fraction < 0.23, () -> "flip fraction " + fraction);
        // The mirrored solution of a tag 2 m ahead is about 4 m behind the robot's true position reflection.
        for (Sample sample : samples) {
            double error = Math.hypot(sample.pose().pose().xMeters(), sample.pose().pose().yMeters());
            if (sample.pose().ambiguity() > 0.5) {
                assertTrue(error > 3.0, () -> "mirrored pose too close to truth: " + error);
            } else {
                assertTrue(error < 1.0, () -> "unflipped pose far from truth: " + error);
            }
        }
    }

    @Test
    void multiTagFramesNeverFlip() {
        List<Sample> samples = run(multiTagField(2.0), noise().ambiguityFlipProbability(1.0), 10.0);

        assertTrue(!samples.isEmpty());
        assertTrue(samples.stream().allMatch(s -> s.pose().ambiguity() < 0.5));
    }

    @Test
    void framesArriveAtTheConfiguredRateOverTwoSeconds() {
        VisionSimNoise thirtyFps = noise().fps(Hertz.of(30.0)).dropProbability(0.0);
        VisionSimNoise tenFps = noise().fps(Hertz.of(10.0)).dropProbability(0.0);

        assertEquals(60, distinctCaptures(run(singleTagField(2.0), thirtyFps, 2.5), 2.0));
        assertEquals(20, distinctCaptures(run(singleTagField(2.0), tenFps, 2.5), 2.0));
    }

    @Test
    void timestampsAreCaptureTimesAndLatencyIsReported() {
        VisionSimNoise fixedLatency = noise().fps(Hertz.of(30.0)).latency(Milliseconds.of(25.0)).jitter(Milliseconds.of(0.0)).dropProbability(0.0);
        List<Sample> samples = run(singleTagField(2.0), fixedLatency, 3.0);

        assertTrue(samples.size() > 60);
        for (Sample sample : samples) {
            PoseMeasurementSample pose = sample.pose();
            double frameIndex = pose.timestampSeconds() * 30.0;
            assertEquals(Math.round(frameIndex), frameIndex, 1.0e-6, "timestamp on the frame grid");
            assertEquals(0.025, pose.latencySeconds(), 1.0e-12);
            double age = sample.deliveredAtSeconds() - pose.timestampSeconds();
            assertTrue(age >= 0.025 - 1.0e-9 && age < 0.025 + DT + 1.0e-9, () -> "delivered after " + age + " s");
        }
    }

    @Test
    void latencyJitterIsZeroMeanAndNonNegative() {
        List<Sample> samples = run(singleTagField(2.0), noise().fps(Hertz.of(30.0)).dropProbability(0.0), 60.0);

        double mean = samples.stream().mapToDouble(s -> s.pose().latencySeconds()).average().orElse(Double.NaN);
        double min = samples.stream().mapToDouble(s -> s.pose().latencySeconds()).min().orElse(Double.NaN);
        double max = samples.stream().mapToDouble(s -> s.pose().latencySeconds()).max().orElse(Double.NaN);
        assertEquals(0.025, mean, 0.001, "mean latency");
        assertTrue(min >= 0.0 && max > min, () -> "latency range " + min + ".." + max);
    }

    @Test
    void dropProbabilityRemovesFramesAtTheConfiguredRate() {
        List<Sample> samples = run(singleTagField(2.0), noise().fps(Hertz.of(30.0)).dropProbability(0.25), 120.0);

        double delivered = samples.size() / (120.0 * 30.0);
        assertTrue(Math.abs(delivered - 0.75) < 0.03, () -> "delivered fraction " + delivered);
    }

    @Test
    void tagsBeyondMaxRangeAreNotSeen() {
        VisionSimNoise limited = noise().maxRange(Meters.of(3.0)).dropProbability(0.0);

        assertTrue(!run(singleTagField(2.0), limited, 5.0).isEmpty());
        assertEquals(0, run(singleTagField(5.0), limited, 5.0).size());
    }

    @Test
    void sameSeedReproducesTheSameMeasurements() {
        List<Sample> first = run(multiTagField(3.0), noise(), 10.0);
        List<Sample> second = run(multiTagField(3.0), noise(), 10.0);

        assertEquals(first.size(), second.size());
        for (int i = 0; i < first.size(); i++) {
            assertEquals(first.get(i).pose().pose().xMeters(), second.get(i).pose().pose().xMeters(), 0.0);
            assertEquals(first.get(i).pose().timestampSeconds(), second.get(i).pose().timestampSeconds(), 0.0);
        }
    }

    @Test
    void perfectNoiseReproducesExactPosesOnEveryUpdate() {
        CameraDevice camera = Cameras.photonVision("front").mount(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0));
        VisionSimulation simulation = new PhotonVisionSimulationProvider().create(
                List.of(camera), multiTagField(2.0).noise(VisionSimNoise.perfect()));
        CameraDevice bound = simulation.bind(camera);
        PoseSnapshot truth = new PoseSnapshot(0.3, -0.2, 0.4);

        for (int k = 1; k <= 50; k++) {
            double now = k * 0.02;
            simulation.update(truth, now);
            List<Measurement> poses = bound.pose().measurements();
            assertEquals(1, poses.size(), () -> "one frame per update at " + now);
            PoseMeasurementSample pose = (PoseMeasurementSample) poses.get(0);
            assertEquals(truth.xMeters(), pose.pose().xMeters(), 0.0);
            assertEquals(truth.yMeters(), pose.pose().yMeters(), 0.0);
            assertEquals(truth.headingRadians(), pose.pose().headingRadians(), 0.0);
            assertEquals(now, pose.timestampSeconds(), 0.0);
            assertEquals(0.0, pose.latencySeconds(), 0.0);
            assertEquals(0.0, pose.ambiguity(), 0.0);
            assertEquals(Double.POSITIVE_INFINITY, pose.stdDevs().raw().xMeters());
        }
    }

    @Test
    void configuredSingleTagStdDevsOverrideSimulatedStdDevs() {
        CameraDevice camera = Cameras.photonVision("front").mount(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0));
        VisionSimulation simulation = new PhotonVisionSimulationProvider().create(
                List.of(camera), singleTagField(2.0).noise(noise()));
        CameraDevice bound = simulation.bind(camera);

        List<Measurement> measurements = new ArrayList<>();
        for (int k = 0; k <= 120; k++) {
            simulation.update(TRUTH, k * DT);
            measurements.addAll(bound.pose().singleTagStdDevs(Meters.of(0.9), Meters.of(0.9), Radians.of(0.2)).measurements());
        }

        assertTrue(!measurements.isEmpty());
        for (Measurement measurement : measurements) {
            PoseMeasurementSample pose = (PoseMeasurementSample) measurement;
            assertEquals(0.9, pose.stdDevs().raw().xMeters(), 1.0e-9);
            assertEquals(0.2, pose.stdDevs().raw().headingRadians(), 1.0e-9);
        }
    }

    private static VisionSimNoise noise() {
        return VisionSimNoise.realistic().seed(SEED);
    }

    private static VisionSimulationField multiTagField(double distance) {
        return VisionSimulationField.of(
                VisionSimulationTarget.aprilTag(1, Meters.of(distance), Meters.of(0.25), Meters.of(1.0), Radians.of(Math.PI)),
                VisionSimulationTarget.aprilTag(2, Meters.of(distance), Meters.of(-0.25), Meters.of(1.0), Radians.of(Math.PI)));
    }

    private static VisionSimulationField singleTagField(double distance) {
        return VisionSimulationField.of(VisionSimulationTarget.aprilTag(1, Meters.of(distance), Meters.of(0.0), Meters.of(1.0), Radians.of(Math.PI)));
    }

    private static List<Sample> run(VisionSimulationField field, VisionSimNoise noise, double seconds) {
        CameraDevice camera = Cameras.photonVision("front").mount(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), Degrees.of(0.0), Degrees.of(0.0), Degrees.of(0.0));
        VisionSimulation simulation = new PhotonVisionSimulationProvider().create(
                List.of(camera), field.noise(noise));
        CameraDevice bound = simulation.bind(camera);
        List<Sample> samples = new ArrayList<>();
        int steps = (int) Math.round(seconds / DT);
        for (int k = 0; k <= steps; k++) {
            double now = k * DT;
            simulation.update(TRUTH, now);
            for (Measurement measurement : bound.pose().measurements()) {
                samples.add(new Sample((PoseMeasurementSample) measurement, now));
            }
        }
        return samples;
    }

    private static int distinctCaptures(List<Sample> samples, double beforeSeconds) {
        Set<Long> captures = new HashSet<>();
        for (Sample sample : samples) {
            if (sample.pose().timestampSeconds() < beforeSeconds) {
                captures.add(Math.round(sample.pose().timestampSeconds() * 1.0e6));
            }
        }
        return captures.size();
    }

    private static double wrap(double radians) {
        return Math.IEEEremainder(radians, 2.0 * Math.PI);
    }

    private static void assertMeanNearZero(Stats stats, String label) {
        double standardError = stats.std() / Math.sqrt(stats.count());
        assertTrue(Math.abs(stats.mean()) < 4.0 * standardError,
                () -> label + " mean " + stats.mean() + " exceeds 4 standard errors (" + standardError + ")");
    }

    private static void assertWithin(double measured, double expected, double relativeBand, String label) {
        assertTrue(Math.abs(measured - expected) <= relativeBand * expected,
                () -> label + ": measured " + measured + ", expected " + expected);
    }

    private record Sample(PoseMeasurementSample pose, double deliveredAtSeconds) {
    }

    private record Stats(double mean, double std, int count) {
        static Stats of(List<Sample> samples, java.util.function.ToDoubleFunction<Sample> value) {
            int n = samples.size();
            double mean = samples.stream().mapToDouble(value).average().orElse(Double.NaN);
            double variance = samples.stream()
                    .mapToDouble(s -> Math.pow(value.applyAsDouble(s) - mean, 2.0))
                    .sum() / (n - 1);
            return new Stats(mean, Math.sqrt(variance), n);
        }
    }
}
