package ca.frc6390.athena.vendor.photonvision;

import static edu.wpi.first.units.Units.Seconds;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.runtime.VisionSimNoise;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationProvider;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * PhotonVision-backed simulation provider for Athena camera declarations.
 *
 * <p>Each camera captures frames on its configured {@link VisionSimNoise} frame grid. A frame is the robot pose at
 * its capture time plus noise, is delivered after the modelled pipeline latency, and carries the capture time as its
 * timestamp. With {@link VisionSimNoise#perfect()} every robot update captures and delivers one exact frame with zero
 * latency.</p>
 */
public final class PhotonVisionSimulationProvider implements VisionSimulationProvider {
    @Override
    public boolean supports(List<CameraDevice> cameras) {
        return cameras != null && cameras.stream().anyMatch(Objects::nonNull);
    }

    @Override
    public VisionSimulation create(List<CameraDevice> cameras) {
        return create(cameras, VisionSimulationField.EMPTY);
    }

    @Override
    public VisionSimulation create(List<CameraDevice> cameras, VisionSimulationField field) {
        return new PhotonVisionSimulation(cameras, field);
    }

    static final class PhotonVisionSimulation implements VisionSimulation {
        private static final double DEFAULT_FOV_DEGREES = 70.0;
        private static final int DEFAULT_WIDTH = 640;
        private static final int DEFAULT_HEIGHT = 480;
        private static final double NATIVE_FPS_FALLBACK = 90.0;
        /** Robot loop period used for updates that do not carry a timestamp. */
        private static final double CLOCK_STEP_SECONDS = 0.02;
        /** Pose history kept for capture-time lookups. */
        private static final double HISTORY_SECONDS = 2.0;
        private static final double TIME_EPSILON_SECONDS = 1.0e-9;
        private static final long CHANNEL_SEED_STRIDE = 7919L;
        private static final double MULTI_TAG_AMBIGUITY = 0.02;
        private static final double SINGLE_TAG_AMBIGUITY = 0.05;
        private static final double SINGLE_TAG_AMBIGUITY_PER_METER = 0.02;
        /** Ambiguity reported for a mirrored single-tag pose. */
        private static final double MIRRORED_AMBIGUITY = 0.6;
        private static final VisionSimulationTarget DEFAULT_TARGET =
                VisionSimulationTarget.aprilTag(1, Meters.of(5.0), Meters.of(0.0), Meters.of(1.0), Radians.of(0.0));

        private final VisionSystemSim visionSystem = createVisionSystem();
        private final Map<String, PhotonCameraSim> cameras = new LinkedHashMap<>();
        private final Map<String, CameraChannel> channels = new LinkedHashMap<>();
        private final List<TimedPose> history = new ArrayList<>();
        private final VisionSimulationField field;
        private final VisionSimNoise noise;
        // Noise model values converted once from the typed noise declaration, so the per-frame path stays primitive.
        private final double framePeriod;
        private final double maxRangeMeters;
        private final double dropProbability;
        private final double singleTagMultiplier;
        private final double translationStdMeters;
        private final double headingStdRadians;
        private final double ambiguityFlipProbability;
        private final double latencyMeanMs;
        private final double jitterMs;
        private double nextClockSeconds = 0.0;

        PhotonVisionSimulation(List<CameraDevice> cameras, VisionSimulationField field) {
            VisionSimulationField safeField = field == null ? VisionSimulationField.EMPTY : field;
            this.field = safeField.targets().isEmpty()
                    ? VisionSimulationField.of(DEFAULT_TARGET).noise(safeField.noise())
                    : safeField;
            this.noise = this.field.noise();
            double fps = noise.fps().in(Hertz);
            this.framePeriod = Double.isInfinite(fps) ? 0.0 : 1.0 / fps;
            this.maxRangeMeters = noise.maxRange().in(Meters);
            this.dropProbability = noise.dropProbability();
            this.singleTagMultiplier = noise.singleTagMultiplier();
            this.translationStdMeters = noise.translationStdAt1m().in(Meters);
            this.headingStdRadians = noise.headingStdAt1m().in(Radians);
            this.ambiguityFlipProbability = noise.ambiguityFlipProbability();
            this.latencyMeanMs = noise.latency().in(Milliseconds);
            this.jitterMs = noise.jitter().in(Milliseconds);
            if (cameras != null) {
                cameras.stream()
                        .filter(Objects::nonNull)
                        .forEach(this::register);
            }
        }

        @Override
        public CameraDevice bind(CameraDevice camera) {
            if (camera == null || camera.hasBoundSignals()) {
                return camera;
            }
            String key = key(camera);
            Supplier<List<Measurement>> poses = () -> channelOutput(key, true);
            Supplier<List<Measurement>> targets = () -> channelOutput(key, false);
            return camera
                    .bindPose(poses)
                    .bindTargets(targets);
        }

        /**
         * Updates the simulated cameras using an internal clock that advances by one robot loop per call.
         * Prefer {@link #update(PoseSnapshot, double)} so frame timing follows the session clock.
         */
        @Override
        public void update(PoseSnapshot robotPose) {
            update(robotPose, nextClockSeconds);
            nextClockSeconds += CLOCK_STEP_SECONDS;
        }

        @Override
        public void update(PoseSnapshot robotPose, double timestampSeconds) {
            if (robotPose == null || !Double.isFinite(timestampSeconds)) {
                return;
            }
            if (visionSystem != null && Boolean.getBoolean("athena.photonvision.sim.native")) {
                visionSystem.update(new Pose2d(
                        robotPose.xMeters(),
                        robotPose.yMeters(),
                        Rotation2d.fromRadians(robotPose.headingRadians())));
            }
            record(timestampSeconds, robotPose);
            for (CameraChannel channel : channels.values()) {
                captureDue(channel, timestampSeconds);
                deliverDue(channel, timestampSeconds);
            }
        }

        Map<String, PhotonCameraSim> cameras() {
            return Map.copyOf(cameras);
        }

        private void register(CameraDevice camera) {
            if (visionSystem != null && Boolean.getBoolean("athena.photonvision.sim.native")) {
                PhotonCameraSim sim = new PhotonCameraSim(new PhotonCamera(camera.name()), defaultProperties(noise));
                visionSystem.addCamera(sim, transform(camera.mountPose()));
                cameras.put(key(camera), sim);
            }
            long seed = noise.seed() + CHANNEL_SEED_STRIDE * channels.size();
            channels.put(key(camera), new CameraChannel(camera, new Random(seed)));
        }

        private static String key(CameraDevice camera) {
            return camera.kind().key() + ":" + camera.name();
        }

        private List<Measurement> channelOutput(String key, boolean poses) {
            CameraChannel channel = channels.get(key);
            if (channel == null) {
                return List.of();
            }
            return poses ? channel.poses : channel.targets;
        }

        /** Stores the robot pose used for capture-time lookups. A backwards clock restarts the simulation. */
        private void record(double timestampSeconds, PoseSnapshot robotPose) {
            if (!history.isEmpty() && timestampSeconds < history.get(history.size() - 1).seconds()) {
                history.clear();
                channels.values().forEach(CameraChannel::restart);
            }
            if (!history.isEmpty() && timestampSeconds == history.get(history.size() - 1).seconds()) {
                history.set(history.size() - 1, new TimedPose(timestampSeconds, robotPose));
            } else {
                history.add(new TimedPose(timestampSeconds, robotPose));
            }
            double cutoff = timestampSeconds - HISTORY_SECONDS;
            while (history.size() > 1 && history.get(0).seconds() < cutoff) {
                history.remove(0);
            }
        }

        private void captureDue(CameraChannel channel, double now) {
            double framePeriod = framePeriodSeconds();
            if (framePeriod == 0.0) {
                capture(channel, now, now);
                return;
            }
            if (Double.isNaN(channel.nextCaptureSeconds)) {
                channel.nextCaptureSeconds = now;
            }
            while (channel.nextCaptureSeconds <= now + TIME_EPSILON_SECONDS) {
                capture(channel, channel.nextCaptureSeconds, now);
                channel.nextCaptureSeconds += framePeriod;
            }
        }

        private double framePeriodSeconds() {
            return framePeriod;
        }

        private void capture(CameraChannel channel, double captureSeconds, double now) {
            CameraDevice camera = channel.camera;
            PoseSnapshot truth = poseAt(captureSeconds);
            CameraMountPose mount = safeMount(camera.mountPose());
            List<VisibleTarget> visible = visibleTargets(truth, mount, field.targets(), maxRangeMeters);

            // Every random draw happens on every capture so the stream stays aligned across configurations.
            double dropRoll = channel.random.nextDouble();
            double flipRoll = channel.random.nextDouble();
            double noiseX = channel.random.nextGaussian();
            double noiseY = channel.random.nextGaussian();
            double noiseHeading = channel.random.nextGaussian();
            double latencyNoise = channel.random.nextGaussian();

            if (visible.isEmpty() || dropRoll < dropProbability) {
                return;
            }
            boolean modelled = !noise.isPerfect();
            int targetCount = visible.size();
            boolean singleTag = targetCount == 1;
            double averageDistance = visible.stream().mapToDouble(VisibleTarget::distanceMeters).average().orElse(0.0);
            double spread = averageDistance * averageDistance * (singleTag ? singleTagMultiplier : 1.0);
            double sigmaXy = translationStdMeters * spread;
            double sigmaHeading = headingStdRadians * spread;

            boolean mirrored = singleTag && flipRoll < ambiguityFlipProbability;
            PoseSnapshot base = mirrored ? mirror(truth, visible.get(0).target()) : truth;
            PoseSnapshot measured = new PoseSnapshot(
                    base.xMeters() + noiseX * sigmaXy,
                    base.yMeters() + noiseY * sigmaXy,
                    normalizeRadians(base.headingRadians() + noiseHeading * sigmaHeading));

            double ambiguity = 0.0;
            if (modelled) {
                ambiguity = mirrored
                        ? MIRRORED_AMBIGUITY
                        : singleTag
                                ? SINGLE_TAG_AMBIGUITY + SINGLE_TAG_AMBIGUITY_PER_METER * averageDistance
                                : MULTI_TAG_AMBIGUITY;
            }
            double latencySeconds = Math.max(0.0, latencyMeanMs + jitterMs * latencyNoise) / 1000.0;

            PoseMeasurementSample pose = Measurements.pose(measured)
                    .timing(Seconds.of(captureSeconds), Seconds.of(latencySeconds))
                    .visionMetadata(ambiguity, targetCount, averageDistance)
                    .source(camera);
            if (modelled) {
                pose = pose.stdDevs(MeasurementStdDevs.of(Meters.of(sigmaXy), Meters.of(sigmaXy), Radians.of(sigmaHeading)));
            }
            List<Measurement> targets = visible.stream()
                    .map(VisibleTarget::measurement)
                    .map(target -> (Measurement) Measurements.target(
                            target.fiducialId(),
                            Degrees.of(target.yawDegrees()),
                            Degrees.of(target.pitchDegrees()),
                            Meters.of(target.distanceMeters()),
                            target.confidence()))
                    .toList();
            channel.pending.add(new PendingFrame(captureSeconds + latencySeconds, channel.sequence++, pose, targets));
        }

        private void deliverDue(CameraChannel channel, double now) {
            List<Measurement> poses = new ArrayList<>();
            List<Measurement> targets = new ArrayList<>();
            while (!channel.pending.isEmpty()
                    && channel.pending.peek().deliverySeconds() <= now + TIME_EPSILON_SECONDS) {
                PendingFrame frame = channel.pending.poll();
                poses.add(frame.pose());
                targets.addAll(frame.targets());
            }
            channel.poses = List.copyOf(poses);
            channel.targets = List.copyOf(targets);
        }

        /** Robot pose at a time inside the history, interpolated linearly. */
        private PoseSnapshot poseAt(double seconds) {
            TimedPose first = history.get(0);
            TimedPose last = history.get(history.size() - 1);
            if (seconds <= first.seconds()) {
                return first.pose();
            }
            if (seconds >= last.seconds()) {
                return last.pose();
            }
            for (int i = 1; i < history.size(); i++) {
                TimedPose next = history.get(i);
                if (seconds <= next.seconds()) {
                    TimedPose previous = history.get(i - 1);
                    double span = next.seconds() - previous.seconds();
                    double fraction = span <= 0.0 ? 0.0 : (seconds - previous.seconds()) / span;
                    return interpolate(previous.pose(), next.pose(), fraction);
                }
            }
            return last.pose();
        }

        private static PoseSnapshot interpolate(PoseSnapshot a, PoseSnapshot b, double fraction) {
            double heading = a.headingRadians() + fraction * normalizeRadians(b.headingRadians() - a.headingRadians());
            return new PoseSnapshot(
                    a.xMeters() + fraction * (b.xMeters() - a.xMeters()),
                    a.yMeters() + fraction * (b.yMeters() - a.yMeters()),
                    normalizeRadians(heading));
        }

        /** Reflects the pose across the plane of the tag, the mirrored solution of a single-tag estimate. */
        private static PoseSnapshot mirror(PoseSnapshot pose, VisionSimulationTarget tag) {
            double normalX = Math.cos(tag.pose().headingRadians());
            double normalY = Math.sin(tag.pose().headingRadians());
            double side = (pose.xMeters() - tag.pose().xMeters()) * normalX
                    + (pose.yMeters() - tag.pose().yMeters()) * normalY;
            double x = pose.xMeters() - 2.0 * side * normalX;
            double y = pose.yMeters() - 2.0 * side * normalY;
            double directionX = Math.cos(pose.headingRadians());
            double directionY = Math.sin(pose.headingRadians());
            double alongNormal = directionX * normalX + directionY * normalY;
            directionX -= 2.0 * alongNormal * normalX;
            directionY -= 2.0 * alongNormal * normalY;
            return new PoseSnapshot(x, y, Math.atan2(directionY, directionX));
        }

        static List<Measurement> simulatedTargets(PoseSnapshot robotPose, CameraDevice camera) {
            return simulatedTargets(robotPose, camera, VisionSimulationField.of(DEFAULT_TARGET));
        }

        static List<Measurement> simulatedTargets(
                PoseSnapshot robotPose,
                CameraDevice camera,
                VisionSimulationField field) {
            VisionSimulationField safeField = field == null ? VisionSimulationField.EMPTY : field;
            return visibleTargets(robotPose, safeMount(camera.mountPose()), safeField.targets(),
                    Double.POSITIVE_INFINITY).stream()
                    .map(VisibleTarget::measurement)
                    .map(target -> (Measurement) Measurements.target(
                            target.fiducialId(),
                            Degrees.of(target.yawDegrees()),
                            Degrees.of(target.pitchDegrees()),
                            Meters.of(target.distanceMeters()),
                            target.confidence()))
                    .toList();
        }

        private static List<VisibleTarget> visibleTargets(
                PoseSnapshot robotPose,
                CameraMountPose mount,
                List<VisionSimulationTarget> targets,
                double maxRangeMeters) {
            return targets.stream()
                    .map(target -> visibleTarget(robotPose, mount, target, maxRangeMeters))
                    .filter(Objects::nonNull)
                    .toList();
        }

        private static VisibleTarget visibleTarget(
                PoseSnapshot robotPose,
                CameraMountPose mount,
                VisionSimulationTarget target,
                double maxRangeMeters) {
            double dx = target.pose().xMeters() - robotPose.xMeters();
            double dy = target.pose().yMeters() - robotPose.yMeters();
            double distance = Math.hypot(dx, dy);
            if (distance <= 1.0e-9 || distance > maxRangeMeters) {
                return null;
            }
            double targetAngle = Math.atan2(dy, dx);
            double cameraYaw = robotPose.headingRadians() + Math.toRadians(mount.yaw().in(Degrees));
            double yawDegrees = Math.toDegrees(normalizeRadians(targetAngle - cameraYaw));
            if (Math.abs(yawDegrees) > DEFAULT_FOV_DEGREES * 0.5) {
                return null;
            }
            double pitchDegrees = Math.toDegrees(Math.atan2(target.height().in(Meters) - mount.z().in(Meters), distance))
                    - mount.pitch().in(Degrees);
            PhotonVisionTarget measurement = PhotonVisionTarget.aprilTag(
                    target.id(),
                    yawDegrees,
                    pitchDegrees,
                    distance,
                    Math.max(0.0, 1.0 - target.confidence()));
            return new VisibleTarget(target, measurement);
        }

        private static CameraMountPose safeMount(CameraMountPose mount) {
            return mount == null || !mount.isFinite() ? CameraMountPose.identity() : mount;
        }

        private static double normalizeRadians(double radians) {
            double value = radians;
            while (value > Math.PI) {
                value -= Math.PI * 2.0;
            }
            while (value < -Math.PI) {
                value += Math.PI * 2.0;
            }
            return value;
        }

        private static SimCameraProperties defaultProperties(VisionSimNoise noise) {
            double fps = Double.isInfinite(noise.fps().in(Hertz)) ? NATIVE_FPS_FALLBACK : noise.fps().in(Hertz);
            SimCameraProperties properties = new SimCameraProperties();
            properties.setCalibration(DEFAULT_WIDTH, DEFAULT_HEIGHT, Rotation2d.fromDegrees(DEFAULT_FOV_DEGREES))
                    .setFPS(fps)
                    .setAvgLatencyMs(noise.latency().in(Milliseconds))
                    .setLatencyStdDevMs(noise.jitter().in(Milliseconds));
            return properties;
        }

        private static Transform3d transform(CameraMountPose pose) {
            CameraMountPose safePose = pose == null || !pose.isFinite() ? CameraMountPose.identity() : pose;
            return new Transform3d(
                    new Translation3d(safePose.x().in(Meters), safePose.y().in(Meters), safePose.z().in(Meters)),
                    new Rotation3d(
                            safePose.roll().in(Radians),
                            safePose.pitch().in(Radians),
                            safePose.yaw().in(Radians)));
        }

        private static VisionSystemSim createVisionSystem() {
            if (!Boolean.getBoolean("athena.photonvision.sim.native")) {
                return null;
            }
            try {
                return new VisionSystemSim("AthenaVisionSim");
            } catch (RuntimeException | LinkageError error) {
                return null;
            }
        }

        private record TimedPose(double seconds, PoseSnapshot pose) {
        }

        private record VisibleTarget(VisionSimulationTarget target, PhotonVisionTarget measurement) {
            double distanceMeters() {
                return measurement.distanceMeters();
            }
        }

        private record PendingFrame(
                double deliverySeconds,
                long sequence,
                PoseMeasurementSample pose,
                List<Measurement> targets) {
        }

        /** Per-camera frame schedule, random stream and delivered output. */
        private static final class CameraChannel {
            private static final Comparator<PendingFrame> DELIVERY_ORDER =
                    Comparator.comparingDouble(PendingFrame::deliverySeconds)
                            .thenComparingLong(PendingFrame::sequence);

            private final CameraDevice camera;
            private final Random random;
            private final PriorityQueue<PendingFrame> pending = new PriorityQueue<>(DELIVERY_ORDER);
            private double nextCaptureSeconds = Double.NaN;
            private long sequence;
            private List<Measurement> poses = List.of();
            private List<Measurement> targets = List.of();

            private CameraChannel(CameraDevice camera, Random random) {
                this.camera = camera;
                this.random = random;
            }

            private void restart() {
                pending.clear();
                nextCaptureSeconds = Double.NaN;
                poses = List.of();
                targets = List.of();
            }
        }
    }
}
