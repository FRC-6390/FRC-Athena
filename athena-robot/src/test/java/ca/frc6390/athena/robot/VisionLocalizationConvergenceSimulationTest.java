package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleModel;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.localization.pipeline.Localization;
import ca.frc6390.athena.localization.pipeline.LocalizationFilters;
import ca.frc6390.athena.localization.pipeline.Localizations;
import ca.frc6390.athena.localization.pipeline.VisionFilters;
import ca.frc6390.athena.localization.pipeline.VisionSourceStatus;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import ca.frc6390.athena.vision.config.Cameras;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import ca.frc6390.athena.vision.runtime.VisionSimNoise;
import ca.frc6390.athena.vision.runtime.VisionSimulation;
import ca.frc6390.athena.vision.runtime.VisionSimulationField;
import ca.frc6390.athena.vision.runtime.VisionSimulationTarget;
import ca.frc6390.athena.vision.runtime.VisionSimulations;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * End-to-end vision localization in the headless simulation: a swerve drivetrain with odometry, two simulated
 * PhotonVision cameras (one sees two tags, the rear one sees a single tag), per-camera filter stages and a Kalman
 * node that starts from an odometry origin far away from the simulated robot.
 *
 * <p>The default simulation uses {@link VisionSimNoise#realistic()}: 30 fps frames with 25 +- 5 ms latency, pose noise
 * that grows with the square of the tag distance (single-tag frames four times noisier), 2 % dropped frames and 5 %
 * single-tag ambiguity flips to a mirrored pose. The tests in this class use those frames end to end, so they check
 * that the Athena pipeline (latency compensation, ambiguity and range gating, Kalman fusion, camera status) converges
 * on imperfect vision. {@link #perfectNoiseKalmanConvergesQuickly()} keeps the exact-pose variant for pipeline-only
 * checks. The provider does not need the PhotonVision native simulation.</p>
 *
 * <p>Measured with the fixed seed of {@link VisionSimNoise#realistic()} (convergence metrics from the
 * test's {@code run} loop): realistic vision first enters the 0.15 m / 3 deg band at 0.52 s, stays within 0.10 m and
 * 3.5 deg afterwards, and ends 1.9 cm from truth. The one 3.5 deg transient at 4.3 s is a single-tag heading sample
 * at 2.9 m (sigma of about 3.4 deg), which is why the post-convergence check allows 2x the band. Perfect vision enters
 * the band at 0.30 s and ends at 0.0 m.</p>
 */
class VisionLocalizationConvergenceSimulationTest {
    private static final double DT = 0.02;
    private static final double TRUE_X = 5.0;
    private static final double TRUE_Y = 2.5;
    private static final double TRUE_HEADING_DEGREES = 160.0;
    private static final double POSITION_TOLERANCE_METERS = 0.15;
    private static final double HEADING_TOLERANCE_DEGREES = 3.0;
    private static final double CONVERGENCE_DEADLINE_SECONDS = 3.0;
    private static final double RUN_SECONDS = 10.0;
    /** Tags placed 3 m from the start pose. Front (yaw 0) sees tags 1 and 2; rear (yaw 180) sees only tag 3. */
    private static final VisionSimulationField FIELD = VisionSimulationField.of(
            VisionSimulationTarget.aprilTag(1, 2.18094, 3.52614, 1.3, Math.PI),
            VisionSimulationTarget.aprilTag(2, 2.01142, 2.23857, 1.3, Math.PI),
            VisionSimulationTarget.aprilTag(3, 7.59808, 1.0, 1.3, Math.PI))
            .noise(VisionSimNoise.realistic());

    @Test
    void kalmanConvergesFromFarOdometryOriginWithTwoPhotonCameras() {
        SimulationSession simulation = SimulationSession.create()
                .visionField(FIELD)
                .resetPose(truth());
        LocalizationRobot robot = new LocalizationRobot(
                photonCamera("Front", frontMount()),
                photonCamera("Rear", rearMount()),
                true);
        robot.odometry.reset(new PoseSnapshot(0.0, 0.0, 0.0));
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(robot);

        Convergence convergence = run(simulation, runtime, robot, RUN_SECONDS);

        assertTrue(convergence.firstConvergedSeconds() <= CONVERGENCE_DEADLINE_SECONDS,
                () -> "Estimate did not converge within " + CONVERGENCE_DEADLINE_SECONDS + " s: first converged at "
                        + convergence.firstConvergedSeconds() + " s, final error " + convergence.finalError());
        assertTrue(convergence.lastDivergedSeconds() < RUN_SECONDS - 1.0,
                () -> "Estimate left the tolerance band late: last out at " + convergence.lastDivergedSeconds()
                        + " s");
        assertTrue(convergence.worstPositionAfterConverged() < 2.0 * POSITION_TOLERANCE_METERS,
                () -> "Position excursion after convergence: " + convergence.worstPositionAfterConverged() + " m");
        assertTrue(convergence.worstHeadingDegreesAfterConverged() < 2.0 * HEADING_TOLERANCE_DEGREES,
                () -> "Heading excursion after convergence: " + convergence.worstHeadingDegreesAfterConverged()
                        + " deg");
        assertWithinTolerance(robot.estimate.pose(), simulation.pose());

        VisionSourceStatus front = status(robot.estimate, "Front");
        VisionSourceStatus rear = status(robot.estimate, "Rear");
        assertEquals(VisionSourceStatus.State.OK, front.state(), () -> "Front status: " + front);
        assertEquals(VisionSourceStatus.State.OK, rear.state(), () -> "Rear status: " + rear);
        assertTrue(front.acceptedPerSecond() > 0.0, () -> "Front accepted nothing: " + front);
        assertTrue(rear.acceptedPerSecond() > 0.0, () -> "Rear accepted nothing: " + rear);
        assertEquals(2, front.tagCount(), () -> "Front should see two tags: " + front);
        assertEquals(1, rear.tagCount(), () -> "Rear should see one tag (heading-constrained path): " + rear);
    }

    @Test
    void perfectNoiseKalmanConvergesQuickly() {
        SimulationSession simulation = SimulationSession.create()
                .visionField(FIELD.noise(VisionSimNoise.perfect()))
                .resetPose(truth());
        LocalizationRobot robot = new LocalizationRobot(
                photonCamera("Front", frontMount()),
                photonCamera("Rear", rearMount()),
                true);
        robot.odometry.reset(new PoseSnapshot(0.0, 0.0, 0.0));
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(robot);

        Convergence convergence = run(simulation, runtime, robot, RUN_SECONDS);

        assertTrue(convergence.firstConvergedSeconds() <= CONVERGENCE_DEADLINE_SECONDS,
                () -> "Perfect vision did not converge: first converged at " + convergence.firstConvergedSeconds());
        assertWithinTolerance(robot.estimate.pose(), simulation.pose());
    }

    @Test
    void frameStampedFarInTheFutureAtStartupDoesNotLockCamerasOut() {
        SimulationSession simulation = SimulationSession.create().visionField(FIELD).resetPose(truth());
        VisionSimulation photon = VisionSimulations.createDiscovered(
                        List.of(photonCamera("Front", frontMount()), photonCamera("Rear", rearMount())), FIELD)
                .orElseThrow(() -> new AssertionError("PhotonVision simulation provider not on the test classpath"));
        // The first delivered frame of each camera is stamped 1000 s in the future, as an unsynced camera clock
        // would report at startup. Later frames are correct.
        FirstFrameFromFuture stamped = new FirstFrameFromFuture(photon, 1000.0);
        PhotonVisionDevice front = (PhotonVisionDevice) stamped.bind(photonCamera("Front", frontMount()));
        PhotonVisionDevice rear = (PhotonVisionDevice) stamped.bind(photonCamera("Rear", rearMount()));
        simulation.vision(stamped);
        // The front camera feeds the Kalman node directly (no filter stage), so the Kalman's own future-frame guard
        // is what keeps the first frame from locking the camera out.
        LocalizationRobot robot = new LocalizationRobot(front, rear, false);
        robot.odometry.reset(new PoseSnapshot(0.0, 0.0, 0.0));
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(robot);

        Convergence convergence = run(simulation, runtime, robot, RUN_SECONDS);

        assertTrue(convergence.firstConvergedSeconds() <= CONVERGENCE_DEADLINE_SECONDS,
                () -> "Future frame locked the estimate out: first converged at "
                        + convergence.firstConvergedSeconds() + " s");
        assertWithinTolerance(robot.estimate.pose(), simulation.pose());
        VisionSourceStatus front2 = status(robot.estimate, "Front");
        assertTrue(front2.acceptedPerSecond() > 0.0, () -> "Front camera stayed locked out: " + front2);
        assertEquals(2, stamped.stampedCameras(), "both cameras must have delivered a future-stamped frame");
    }

    /**
     * Steps the simulation and runtime. {@code firstConverged} is the first time the estimate is inside the tolerance
     * band. After that, {@code worstAfterConverged*} record the largest excursions, which a realistic single-tag
     * heading transient can make slightly larger than the band.
     */
    private static Convergence run(
            SimulationSession simulation,
            RobotRuntime runtime,
            LocalizationRobot robot,
            double seconds) {
        double firstConverged = Double.NaN;
        double lastDiverged = 0.0;
        double finalError = Double.NaN;
        double worstPosition = 0.0;
        double worstHeading = 0.0;
        int cycles = (int) Math.round(seconds / DT);
        for (int cycle = 1; cycle <= cycles; cycle++) {
            double now = cycle * DT;
            simulation.step(DT);
            runtime.robotPeriodic(now, DT);
            PoseSnapshot truth = simulation.pose();
            PoseSnapshot estimate = robot.estimate.pose();
            finalError = positionError(estimate, truth);
            if (converged(estimate, truth)) {
                if (Double.isNaN(firstConverged)) {
                    firstConverged = now;
                }
            } else {
                lastDiverged = now;
            }
            if (!Double.isNaN(firstConverged)) {
                worstPosition = Math.max(worstPosition, positionError(estimate, truth));
                worstHeading = Math.max(worstHeading, headingErrorDegrees(estimate, truth));
            }
        }
        return new Convergence(firstConverged, lastDiverged, finalError, worstPosition, worstHeading);
    }

    private static boolean converged(PoseSnapshot estimate, PoseSnapshot truth) {
        return positionError(estimate, truth) < POSITION_TOLERANCE_METERS
                && headingErrorDegrees(estimate, truth) < HEADING_TOLERANCE_DEGREES;
    }

    private static void assertWithinTolerance(PoseSnapshot estimate, PoseSnapshot truth) {
        assertTrue(positionError(estimate, truth) < POSITION_TOLERANCE_METERS,
                () -> "Position error " + positionError(estimate, truth) + " m; truth=" + truth + ", estimate="
                        + estimate);
        assertTrue(headingErrorDegrees(estimate, truth) < HEADING_TOLERANCE_DEGREES,
                () -> "Heading error " + headingErrorDegrees(estimate, truth) + " deg; truth=" + truth
                        + ", estimate=" + estimate);
    }

    private static double positionError(PoseSnapshot estimate, PoseSnapshot truth) {
        return Math.hypot(estimate.xMeters() - truth.xMeters(), estimate.yMeters() - truth.yMeters());
    }

    private static double headingErrorDegrees(PoseSnapshot estimate, PoseSnapshot truth) {
        return Math.abs(Math.toDegrees(Math.IEEEremainder(
                estimate.headingRadians() - truth.headingRadians(), 2.0 * Math.PI)));
    }

    private static VisionSourceStatus status(Localization localization, String camera) {
        Optional<VisionSourceStatus> status = localization.visionStatus().stream()
                .filter(candidate -> camera.equals(candidate.name()))
                .findFirst();
        assertTrue(status.isPresent(), () -> "No vision status for camera " + camera + ": "
                + localization.visionStatus());
        return status.get();
    }

    private static PoseSnapshot truth() {
        return new PoseSnapshot(TRUE_X, TRUE_Y, Math.toRadians(TRUE_HEADING_DEGREES));
    }

    private static CameraMountPose frontMount() {
        return new CameraMountPose(0.3, 0.0, 0.5, 0.0, -15.0, 0.0);
    }

    private static CameraMountPose rearMount() {
        return new CameraMountPose(-0.3, 0.0, 0.5, 180.0, -15.0, 0.0);
    }

    private static PhotonVisionDevice photonCamera(String name, CameraMountPose mount) {
        return Cameras.photonVision(name).mount(mount).failurePolicy(FailurePolicy.DISABLE_DEVICE);
    }

    private record Convergence(
            double firstConvergedSeconds,
            double lastDivergedSeconds,
            double finalError,
            double worstPositionAfterConverged,
            double worstHeadingDegreesAfterConverged) {
    }

    /**
     * Delegates to a vision simulation and stamps only the first delivered frame of each bound camera with a time far
     * in the future. Frames are delivered through the bound pose supplier, so the stamp is applied there.
     */
    private static final class FirstFrameFromFuture implements VisionSimulation {
        private final VisionSimulation delegate;
        private final double futureSeconds;

        private final List<StampFirstFrame> stamps = new ArrayList<>();

        private FirstFrameFromFuture(VisionSimulation delegate, double futureSeconds) {
            this.delegate = delegate;
            this.futureSeconds = futureSeconds;
        }

        /** Number of bound cameras whose first frame has been stamped. */
        private int stampedCameras() {
            return (int) stamps.stream().filter(StampFirstFrame::stamped).count();
        }

        @Override
        public CameraDevice bind(CameraDevice camera) {
            if (camera == null || camera.hasBoundSignals()) {
                return camera;
            }
            // Bind the simulation through a fresh declaration: copies of a declaration share their signal bindings,
            // so binding the simulation to `camera` itself would replace the supplier this wrapper reads.
            CameraDevice inner = delegate.bind(Cameras.photonVision(camera.name()).mount(camera.mountPose()));
            StampFirstFrame poses = new StampFirstFrame(futureSeconds);
            stamps.add(poses);
            return camera.bindPose(() -> poses.apply(inner.sourcePose().measurements()))
                    .bindTargets(() -> inner.sourceTargets().measurements());
        }

        @Override
        public void update(PoseSnapshot robotPose) {
            delegate.update(robotPose);
        }

        @Override
        public void update(PoseSnapshot robotPose, double timestampSeconds) {
            delegate.update(robotPose, timestampSeconds);
        }
    }

    /** Stamps the first non-empty frame list it sees, and every read of that same list, into the future. */
    private static final class StampFirstFrame {
        private final double futureSeconds;
        private List<Measurement> stampedList;

        private StampFirstFrame(double futureSeconds) {
            this.futureSeconds = futureSeconds;
        }

        private boolean stamped() {
            return stampedList != null;
        }

        private List<Measurement> apply(List<Measurement> frames) {
            if (stampedList == null && !frames.isEmpty()) {
                stampedList = frames;
            }
            if (frames != stampedList) {
                return frames;
            }
            return frames.stream()
                    .map(measurement -> measurement instanceof PoseMeasurementSample pose
                            ? (Measurement) pose.timing(pose.timestampSeconds() + futureSeconds, pose.latencySeconds())
                            : measurement)
                    .toList();
        }
    }

    /** Swerve drivetrain with two cameras and per-camera stages feeding one Kalman node. */
    private static final class LocalizationRobot implements Mechanism {
        private final ImuDevice imuDevice = ImuDevice.of(ImuKinds.PIGEON_2, 90);
        private final ImuSource imu = imuDevice.relative();
        private final SwerveModule frontLeft = module(91, 101, 111);
        private final SwerveModule frontRight = module(92, 102, 112);
        private final SwerveModule backLeft = module(93, 103, 113);
        private final SwerveModule backRight = module(94, 104, 114);
        private final SwerveKinematics kinematics = SwerveKinematics.rectangular(
                0.5588, 0.5588, 5.0, frontLeft, frontRight, backLeft, backRight);
        final SwerveOdometry odometry = kinematics.odometry(imu);

        private final PoseSignal frontPose;
        private final PoseSignal rearPose;
        private final Localization frontStage;
        private final Localization rearStage;
        final Localization estimate;

        private LocalizationRobot(PhotonVisionDevice front, PhotonVisionDevice rear, boolean frontStaged) {
            frontPose = configured(front.pose()).multiTagStdDevs(0.25, 0.25, Math.toRadians(5.0));
            // Single-tag frames from the rear camera are solved with the robot heading.
            rearPose = configured(rear.pose().singleTagUsesHeading());
            frontStage = stage(frontPose, "frontStage");
            rearStage = stage(rearPose, "rearStage");
            estimate = Localizations.kalman()
                    .input(odometry, frontStaged ? frontStage : frontPose, rearStage)
                    .stateStdDevs(0.08, Math.toRadians(8.0))
                    .defaultVisionStdDevs(0.9, Math.toRadians(25.0))
                    .maxNormalizedVisionResidual(9.0)
                    .name("estimatedFieldPose");
        }

        private static PoseSignal configured(PoseSignal pose) {
            return pose.singleTagStdDevs(0.9, 0.9, Math.toRadians(12.0))
                    .distanceStdDevScaling(2.0, 2.0);
        }

        private static Localization stage(PoseSignal camera, String name) {
            return Localizations.filter()
                    .input(camera)
                    .filter(VisionFilters.finitePoseAndTimestamp())
                    .filter(VisionFilters.maxLatencySeconds(0.15))
                    .filter(VisionFilters.maxAmbiguity(0.20))
                    .filter(VisionFilters.maxAverageTagDistanceMeters(5.0))
                    .filter(LocalizationFilters.inside(Rectangle2d.field(16.541, 8.0692)))
                    .name(name);
        }

        private static SwerveModule module(int driveId, int steerId, int encoderId) {
            TestModule module = new TestModule();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId));
            module.steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId));
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, encoderId).units(EncoderUnit.ROTATIONS));
            return module
                    .driveMaxSpeedMetersPerSecond(5.0)
                    .steerPid(12.0, 0.0, 0.0);
        }
    }

    private static final class TestModule extends SwerveModule {
        private TestModule() {
            super(SwerveModuleModel.custom(2.0, 1.0, 1.0 / Math.PI));
        }
    }
}
