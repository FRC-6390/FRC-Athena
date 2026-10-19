package ca.frc6390.athena.localization.pipeline;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.drivetrain.swerve.SwerveKinematics;
import ca.frc6390.athena.drivetrain.swerve.SwerveModule;
import ca.frc6390.athena.drivetrain.swerve.SwerveModuleModel;
import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.Test;

class WpilibLocalizationParityTest {
    private static final double PARITY_TOLERANCE = 1.0e-8;
    private static final MeasurementStdDevs STATE_STD_DEVS = MeasurementStdDevs.of(0.08, 0.08, 0.04);
    private static final MeasurementStdDevs DEFAULT_VISION_STD_DEVS = MeasurementStdDevs.of(0.7, 0.7, 0.4);

    @Test
    void odometryOnlyMatchesWpilibFrameByFrame() {
        ParityRig rig = new ParityRig();

        rig.step(0.00, distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.02, distances(0.10, 0.10, 0.10, 0.10), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.04, distances(0.20, 0.20, 0.20, 0.20), angles(0.25, 0.25, 0.25, 0.25), 0.0);
        rig.step(0.06, distances(0.25, 0.35, 0.25, 0.35), angles(-0.125, 0.125, 0.125, -0.125), 12.0);
        rig.step(0.08, distances(0.31, 0.43, 0.30, 0.44), angles(-0.12, 0.13, 0.12, -0.13), 35.0);
        rig.step(0.10, distances(0.40, 0.52, 0.39, 0.53), angles(0.5, 0.5, 0.5, 0.5), 179.0);
        rig.step(0.12, distances(0.48, 0.61, 0.47, 0.62), angles(0.5, 0.5, 0.5, 0.5), -179.0);
    }

    @Test
    void immediateAndDelayedVisionMatchWpilibFrameByFrame() {
        ParityRig rig = new ParityRig();
        Object frontCamera = new Object();
        Object rearCamera = new Object();

        rig.step(0.0, distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.5, distances(0.5, 0.5, 0.5, 0.5), angles(0.0, 0.0, 0.0, 0.0), 0.0,
                vision(0.46, 0.03, 0.01, 0.5, MeasurementStdDevs.of(0.15, 0.18, 0.12), frontCamera));
        rig.step(1.0, distances(1.0, 1.0, 1.0, 1.0), angles(0.0, 0.0, 0.0, 0.0), 2.0);
        rig.step(1.5, distances(1.5, 1.5, 1.5, 1.5), angles(0.0, 0.0, 0.0, 0.0), 4.0,
                vision(0.92, -0.04, 0.02, 0.9, MeasurementStdDevs.of(0.10, 0.12, 0.08), rearCamera));
        rig.step(2.0, distances(2.0, 2.0, 2.0, 2.0), angles(0.0, 0.0, 0.0, 0.0), 6.0,
                vision(1.96, 0.08, 0.10, 2.0, MeasurementStdDevs.of(0.25, 0.20, 0.15), frontCamera));
        rig.step(2.5, distances(2.5, 2.5, 2.5, 2.5), angles(0.0, 0.0, 0.0, 0.0), 8.0);
    }

    @Test
    void resetMatchesFreshWpilibEstimatorAndDoesNotReuseOldWheelTravel() {
        ParityRig rig = new ParityRig();

        rig.step(0.0, distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.step(0.5, distances(0.5, 0.5, 0.5, 0.5), angles(0.0, 0.0, 0.0, 0.0), 0.0);
        rig.reset(new PoseSnapshot(4.0, 2.0, Math.toRadians(90.0)));
        rig.step(1.0, distances(0.9, 0.9, 0.9, 0.9), angles(0.0, 0.0, 0.0, 0.0), 15.0);
        rig.step(1.5, distances(1.2, 1.2, 1.2, 1.2), angles(0.0, 0.0, 0.0, 0.0), 15.0);
    }

    @Test
    void innovationGateAcquiresAConsistentFieldPoseAwayFromOdometryOrigin() {
        ParityRig rig = new ParityRig(9.0);
        Object camera = new Object();

        rig.stepAthenaOnly(0.00, vision(5.0, 2.0, 0.2, 0.00,
                MeasurementStdDevs.of(0.2, 0.2, 0.1), camera));
        rig.stepAthenaOnly(0.02, vision(5.02, 2.01, 0.2, 0.02,
                MeasurementStdDevs.of(0.2, 0.2, 0.1), camera));
        assertEquals(0.0, rig.athena.pose().xMeters(), 1.0e-9);

        rig.stepAthenaOnly(0.04, vision(5.01, 2.0, 0.2, 0.04,
                MeasurementStdDevs.of(0.2, 0.2, 0.1), camera));

        assertTrue(rig.athena.pose().xMeters() > 0.25);
    }

    @Test
    void translationOnlyVisionCorrectsPositionWithoutRotatingImuHeading() {
        ParityRig rig = new ParityRig(Double.POSITIVE_INFINITY, true);

        rig.stepAthenaOnly(0.0, vision(1.0, 0.0, Math.PI, 0.0,
                MeasurementStdDevs.of(0.1, 0.1, 0.01), new Object()));

        assertTrue(rig.athena.pose().xMeters() > 0.0);
        assertEquals(0.0, rig.athena.pose().headingRadians(), 1.0e-6);
    }

    @Test
    void headingResidualGateRejectsLargeJumpButAllowsSmallGyroCorrection() {
        ParityRig rig = new ParityRig(
                Double.POSITIVE_INFINITY,
                false,
                Math.toRadians(35.0));

        rig.stepAthenaOnly(0.0, vision(1.0, 0.0, Math.PI, 0.0,
                MeasurementStdDevs.of(0.1, 0.1, 0.05), new Object()));
        assertEquals(0.0, rig.athena.pose().headingRadians(), 1.0e-9);

        rig.stepAthenaOnly(0.02, vision(1.0, 0.0, Math.toRadians(10.0), 0.02,
                MeasurementStdDevs.of(0.1, 0.1, 0.05), new Object()));
        assertTrue(rig.athena.pose().headingRadians() > 0.0);
        assertTrue(rig.athena.pose().headingRadians() < Math.toRadians(10.0));
    }

    @Test
    void headingResidualGateRecoversFromConsistentCameraHeading() {
        ParityRig rig = new ParityRig(
                Double.POSITIVE_INFINITY,
                false,
                Math.toRadians(35.0));
        Object camera = new Object();

        rig.stepAthenaOnly(0.00, vision(1.0, 0.0, Math.PI / 2.0, 0.00,
                MeasurementStdDevs.of(0.1, 0.1, 0.05), camera));
        rig.stepAthenaOnly(0.02, vision(1.0, 0.0, Math.PI / 2.0, 0.02,
                MeasurementStdDevs.of(0.1, 0.1, 0.05), camera));
        assertEquals(0.0, rig.athena.pose().headingRadians(), 1.0e-9);

        rig.stepAthenaOnly(0.04, vision(1.0, 0.0, Math.PI / 2.0, 0.04,
                MeasurementStdDevs.of(0.1, 0.1, 0.05), camera));

        assertTrue(rig.athena.pose().headingRadians() > Math.toRadians(10.0));
    }

    @Test
    void headingOnlyRecoveryIgnoresSyntheticTranslationChanges() {
        ParityRig rig = new ParityRig(
                9.0,
                false,
                true,
                Double.POSITIVE_INFINITY);
        Object cameras = new Object();

        rig.stepAthenaOnly(0.00, vision(1.0, 1.0, Math.toRadians(90.0), 0.00,
                MeasurementStdDevs.of(0.2, 0.2, 0.1), cameras));
        rig.stepAthenaOnly(0.02, vision(8.0, 6.0, Math.toRadians(90.0), 0.02,
                MeasurementStdDevs.of(0.2, 0.2, 0.1), cameras));
        assertEquals(0.0, rig.athena.pose().headingRadians(), 1.0e-9);

        rig.stepAthenaOnly(0.04, vision(3.0, 7.0, Math.toRadians(90.0), 0.04,
                MeasurementStdDevs.of(0.2, 0.2, 0.1), cameras));

        assertTrue(rig.athena.pose().headingRadians() > Math.toRadians(10.0));
    }

    /** A camera whose ambiguity is too high is rejected by name, and the status says why. */
    @Test
    void ambiguousSingleTagFrameIsReportedAsRejectedByTheAmbiguityFilter() {
        ParityRig rig = new ParityRig(UnaryOperator.identity(), VisionFilters.maxAmbiguity(0.20));
        rig.stepAthenaOnly(0.0, singleTag(5.0, 2.0, 0.0, 0.0, 0.31, "Intake", RobotVelocity.zero()));

        var status = statusOf(rig, "Intake");
        assertEquals(VisionSourceStatus.State.REJECTED, status.state());
        assertEquals("ambiguity 0.31 > 0.20", status.reason());
        assertEquals(0.0, status.acceptedPerSecond(), 1e-12);
        assertEquals(1.0, status.rejectedPerSecond(), 1e-12);
        assertTrue(Double.isNaN(status.secondsSinceAccepted()), "never accepted");
    }

    /** An innovation-gated frame is held back by the recovery path; the status names the gate and the progress. */
    @Test
    void innovationGatedFrameIsRecoveringThenOkOnceSelfConsistent() {
        ParityRig rig = new ParityRig(9.0);
        String camera = "Turret";
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.2, 0.2, 0.1);

        // The robot is at the origin; a multi-tag frame at (5, 2) is far outside the innovation gate.
        rig.stepAthenaOnly(0.00, multiTag(5.0, 2.0, 0.0, 0.00, stdDevs, camera));
        var recovering = statusOf(rig, camera);
        assertEquals(VisionSourceStatus.State.RECOVERING, recovering.state());
        assertTrue(recovering.reason().startsWith("innovation "), recovering.reason());
        assertTrue(recovering.reason().endsWith("(recovering 1/3)"), recovering.reason());

        for (int i = 1; i <= 3; i++) {
            double t = i * 0.02;
            rig.stepAthenaOnly(t, multiTag(5.0, 2.0, 0.0, t, stdDevs, camera));
        }
        var recovered = statusOf(rig, camera);
        assertEquals(VisionSourceStatus.State.OK, recovered.state());
        assertEquals("", recovered.reason());
        assertTrue(recovered.acceptedPerSecond() > 0.0);
    }

    /** A frame whose clock runs ahead of robot time is UNSYNCED with the amount of skew in its reason. */
    @Test
    void frameFromTheFutureIsUnsyncedAndDoesNotLockTheCameraOut() {
        ParityRig rig = new ParityRig(Double.POSITIVE_INFINITY);
        String camera = "Side";
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.2, 0.2, 0.1);

        rig.stepAthenaOnly(0.0, multiTag(5.0, 2.0, 0.0, 3.2, stdDevs, camera));
        var status = statusOf(rig, camera);
        assertEquals(VisionSourceStatus.State.UNSYNCED, status.state());
        assertEquals("frame 3.2 s in the future", status.reason());
        assertTrue(status.frameAgeSeconds() < -3.0);

        rig.stepAthenaOnly(0.02, multiTag(5.0, 2.0, 0.0, 0.01, stdDevs, camera));
        assertEquals(VisionSourceStatus.State.OK, statusOf(rig, camera).state(),
                "a later correct frame is accepted, not locked out");
    }

    /** No new frame for more than half a second after frames arrived is STALE. */
    @Test
    void cameraWithoutNewFramesBecomesStale() {
        ParityRig rig = new ParityRig(Double.POSITIVE_INFINITY);
        String camera = "Climber";
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.2, 0.2, 0.1);
        rig.stepAthenaOnly(1.00, multiTag(5.0, 2.0, 0.0, 1.00, stdDevs, camera));
        assertEquals(VisionSourceStatus.State.OK, statusOf(rig, camera).state());

        rig.stepAthenaOnly(1.30, new VisionSample[0]);
        assertEquals(VisionSourceStatus.State.OK, statusOf(rig, camera).state(), "0.3 s is still fresh");

        rig.stepAthenaOnly(1.60, new VisionSample[0]);
        var stale = statusOf(rig, camera);
        assertEquals(VisionSourceStatus.State.STALE, stale.state());
        assertEquals("no frame for 0.6 s", stale.reason());
        assertEquals(0.6, stale.frameAgeSeconds(), 1e-9);
        assertEquals(0.6, stale.secondsSinceAccepted(), 1e-9);
    }

    /** A rotating robot's frame is refused past the spin limit and the reason gives the rate. */
    @Test
    void frameTakenWhileSpinningIsRejectedWithItsRate() {
        ParityRig rig = new ParityRig(kalman -> kalman.maxVisionAngularVelocity(3.0));
        String camera = "Intake";
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.2, 0.2, 0.1);

        rig.stepAthenaOnly(0.0, new VisionSample[] {
                multiTagWithSpeed(5.0, 2.0, 0.0, 0.0, stdDevs, camera, new RobotVelocity(0.0, 0.0, 4.1))});
        var status = statusOf(rig, camera);
        assertEquals(VisionSourceStatus.State.REJECTED, status.state());
        assertEquals("spinning 4.1 rad/s > 3.0", status.reason());
        assertEquals(0.0, rig.athena.pose().xMeters(), 1e-9, "the spinning frame did not move the estimate");
    }

    /** Speed scaling trusts a frame less while driving: the same frame moves the estimate less. */
    @Test
    void visionStdDevSpeedScalingMovesTheEstimateLessWhenDriving() {
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.3, 0.3, 0.1);
        ParityRig plain = new ParityRig(UnaryOperator.identity());
        ParityRig scaled = new ParityRig(kalman -> kalman.visionStdDevSpeedScaling(1.0, 0.0));
        RobotVelocity driving = new RobotVelocity(3.0, 0.0, 0.0);

        plain.stepAthenaOnly(0.0, new VisionSample[] {
                multiTagWithSpeed(1.0, 0.0, 0.0, 0.0, stdDevs, "Intake", driving)});
        scaled.stepAthenaOnly(0.0, new VisionSample[] {
                multiTagWithSpeed(1.0, 0.0, 0.0, 0.0, stdDevs, "Intake", driving)});

        double plainX = plain.athena.pose().xMeters();
        double scaledX = scaled.athena.pose().xMeters();
        assertTrue(plainX > 0.0 && scaledX > 0.0, "both pull toward the vision pose");
        assertTrue(scaledX < plainX, "scaled=" + scaledX + " plain=" + plainX);
    }

    /** A Kalman estimate is unseeded at boot, stays unseeded without vision, and is seeded by a reset or a vision frame. */
    @Test
    void seededTracksResetAndAcceptedVisionOnly() {
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.2, 0.2, 0.1);

        ParityRig idle = new ParityRig();
        idle.stepAthenaOnly(0.00, new VisionSample[0]);
        idle.stepAthenaOnly(0.02, new VisionSample[0]);
        assertFalse(idle.athena.seeded(), "no reset and no vision: unseeded");

        ParityRig reset = new ParityRig();
        reset.athena.reset(new PoseSnapshot(1.0, 2.0, 0.5)).apply(reset.context);
        assertTrue(reset.athena.seeded(), "a reset seeds the pose");

        ParityRig vision = new ParityRig();
        vision.stepAthenaOnly(0.00, new VisionSample[] {multiTagWithSpeed(5.0, 2.0, 0.3, 0.00, stdDevs, "Intake",
                RobotVelocity.zero())});
        assertTrue(vision.athena.seeded(), "an accepted vision frame seeds the pose");
    }

    /**
     * While disabled, a multi-tag frame resets the estimate to the vision pose at once; single-tag frames still
     * take the normal update, and the reset is rate limited to one per 0.25 s.
     */
    @Test
    void seedFromVisionWhileDisabledResetsToMultiTagPoseAtMostOncePerQuarterSecond() {
        boolean[] disabled = {true};
        ParityRig rig = new ParityRig(kalman -> kalman.seedFromVisionWhileDisabled(() -> disabled[0]));
        String camera = "Intake";
        MeasurementStdDevs stdDevs = MeasurementStdDevs.of(0.2, 0.2, 0.1);

        rig.stepAthenaOnly(0.00, new VisionSample[] {multiTagWithSpeed(5.0, 2.0, 0.3, 0.00, stdDevs, camera,
                RobotVelocity.zero())});
        assertEquals(5.0, rig.athena.pose().xMeters(), 1e-9, "seeded to the multi-tag pose");
        assertEquals(2.0, rig.athena.pose().yMeters(), 1e-9);
        assertEquals(0.3, rig.athena.pose().headingRadians(), 1e-9, "heading seeded too");

        rig.stepAthenaOnly(0.05, new VisionSample[] {multiTagWithSpeed(9.0, 9.0, 0.3, 0.05, stdDevs, camera,
                RobotVelocity.zero())});
        assertTrue(Math.abs(rig.athena.pose().xMeters() - 9.0) > 0.5,
                "within 0.25 s of the last seed the frame takes the normal, damped update");

        rig.stepAthenaOnly(0.30, new VisionSample[] {multiTagWithSpeed(7.0, 4.0, 0.3, 0.30, stdDevs, camera,
                RobotVelocity.zero())});
        assertEquals(7.0, rig.athena.pose().xMeters(), 1e-9, "a seed is allowed again after 0.25 s");
        assertEquals(4.0, rig.athena.pose().yMeters(), 1e-9);

        disabled[0] = false;
        rig.stepAthenaOnly(0.60, new VisionSample[] {multiTagWithSpeed(1.0, 1.0, 0.3, 0.60, stdDevs, camera,
                RobotVelocity.zero())});
        assertTrue(Math.abs(rig.athena.pose().xMeters() - 1.0) > 0.5, "enabled robots never seed");
    }

    /**
     * Two cameras fed straight into the Kalman node converge faster than the same frames fused first by covariance
     * intersection. Covariance intersection assumes the two cameras may be correlated, so it keeps the variance of one
     * camera; two independent updates keep the information of both. The comparison is the convergence transient, the
     * part where the extra information shows.
     */
    @Test
    void twoCamerasFeedingTheKalmanDirectlyConvergeFasterThanCovarianceIntersection() {
        ParityRig direct = new ParityRig(2, true);
        ParityRig fused = new ParityRig(2, false);
        java.util.Random noise = new java.util.Random(2026);
        MeasurementStdDevs cameraStdDevs = MeasurementStdDevs.of(0.45, 0.45, Math.toRadians(8.0));
        double directError = 0.0;
        double fusedError = 0.0;
        int steps = 15;
        for (int i = 0; i < steps; i++) {
            double t = i * 0.05;
            List<List<VisionSample>> frames = new ArrayList<>();
            for (int camera = 0; camera < 2; camera++) {
                frames.add(List.of(multiTagWithSpeed(
                        5.0 + noise.nextGaussian() * 0.45,
                        2.0 + noise.nextGaussian() * 0.45,
                        Math.toRadians(20.0) + noise.nextGaussian() * Math.toRadians(8.0),
                        t - 0.01, cameraStdDevs, "camera" + camera, RobotVelocity.zero())));
            }
            direct.stepCameras(t, frames);
            fused.stepCameras(t, frames);
            directError += squaredError(direct.athena.pose());
            fusedError += squaredError(fused.athena.pose());
        }
        assertTrue(directError < fusedError,
                "transient error direct=" + Math.sqrt(directError) + " fused=" + Math.sqrt(fusedError));
    }

    /**
     * The registry carries each refreshed camera's status for the ARCP bridge, and the dashboard path builder is the
     * one the NetworkTables publisher uses. The NetworkTables values themselves are not read here because the test
     * JVM has no ntcore native library.
     */
    @Test
    void refreshedCameraIsInTheRegistryUnderItsDashboardPaths() {
        ParityRig rig = new ParityRig(UnaryOperator.identity(), VisionFilters.maxAmbiguity(0.20));
        rig.stepAthenaOnly(0.0, singleTag(5.0, 2.0, 0.0, 0.0, 0.31, "RegCam", RobotVelocity.zero()));

        VisionSourceStatus status = VisionStatusRegistry.latest().stream()
                .filter(candidate -> candidate.name().equals("RegCam"))
                .findFirst()
                .orElseThrow(() -> new AssertionError("RegCam missing from the registry"));
        assertEquals(VisionSourceStatus.State.REJECTED, status.state());
        assertEquals("ambiguity 0.31 > 0.20", status.reason());
        assertEquals("athena/vision/RegCam/State", VisionSourceStatus.path("RegCam", "State"));
        assertEquals("athena/vision/RegCam/SecondsSinceAccepted",
                VisionSourceStatus.path("RegCam", "SecondsSinceAccepted"));
    }

    /** A filter that names no reason is reported by its name, and an unnamed one by its position. */
    @Test
    void customFiltersThatDoNotNameTheirRejectionAreReportedByNameOrPosition() {
        LocalizationFilter glare = new LocalizationFilter() {
            @Override
            public boolean accept(Localization localization, Measurement measurement, PoseSnapshot pose) {
                return false;
            }

            @Override
            public String name() {
                return "glare";
            }
        };
        ParityRig named = new ParityRig(UnaryOperator.identity(), glare);
        named.stepAthenaOnly(0.0, multiTag(5.0, 2.0, 0.0, 0.0, MeasurementStdDevs.of(0.2, 0.2, 0.1), "Named"));
        assertEquals("filter glare", statusOf(named, "Named").reason());

        LocalizationFilter unnamed = (localization, measurement, pose) -> false;
        ParityRig positional = new ParityRig(UnaryOperator.identity(), VisionFilters.maxAmbiguity(1.0), unnamed);
        positional.stepAthenaOnly(0.0, multiTag(5.0, 2.0, 0.0, 0.0, MeasurementStdDevs.of(0.2, 0.2, 0.1), "Plain"));
        assertEquals("filter #2", statusOf(positional, "Plain").reason());
    }

    private static VisionSourceStatus statusOf(ParityRig rig, String camera) {
        return rig.athena.visionStatus().stream()
                .filter(status -> status.name().equals(camera))
                .findFirst()
                .orElseThrow(() -> new AssertionError("no status for " + camera + " in " + rig.athena.visionStatus()));
    }

    private static double squaredError(PoseSnapshot pose) {
        double dx = pose.xMeters() - 5.0;
        double dy = pose.yMeters() - 2.0;
        double dh = Math.IEEEremainder(pose.headingRadians() - Math.toRadians(20.0), Math.PI * 2.0);
        return dx * dx + dy * dy + dh * dh;
    }

    private static VisionSample multiTag(
            double xMeters,
            double yMeters,
            double headingRadians,
            double timestampSeconds,
            MeasurementStdDevs stdDevs,
            Object source) {
        return multiTagWithSpeed(xMeters, yMeters, headingRadians, timestampSeconds, stdDevs, source,
                RobotVelocity.zero());
    }

    private static VisionSample multiTagWithSpeed(
            double xMeters,
            double yMeters,
            double headingRadians,
            double timestampSeconds,
            MeasurementStdDevs stdDevs,
            Object source,
            RobotVelocity speeds) {
        return new VisionSample(new PoseSnapshot(xMeters, yMeters, headingRadians), speeds, timestampSeconds,
                0.02, 0.05, 2, 2.0, stdDevs, source);
    }

    private static VisionSample singleTag(
            double xMeters,
            double yMeters,
            double headingRadians,
            double timestampSeconds,
            double ambiguity,
            Object source,
            RobotVelocity speeds) {
        return new VisionSample(new PoseSnapshot(xMeters, yMeters, headingRadians), speeds, timestampSeconds,
                0.02, ambiguity, 1, 2.0, MeasurementStdDevs.of(0.9, 0.9, Math.toRadians(12.0)), source);
    }

    private static double[] distances(double frontLeft, double frontRight, double backLeft, double backRight) {
        return new double[] {frontLeft, frontRight, backLeft, backRight};
    }

    private static double[] angles(double frontLeft, double frontRight, double backLeft, double backRight) {
        return new double[] {frontLeft, frontRight, backLeft, backRight};
    }

    /** Robot stationary at boot on red: gyro reads 0, the camera sees the field pose facing 180 degrees. */
    @Test
    void startupSingleTagOnRedConvergesWithRealisticNoise() {
        ParityRig rig = new ParityRig(9.0, false, Math.toRadians(35.0));
        Object camera = new Object();
        java.util.Random noise = new java.util.Random(6390);
        MeasurementStdDevs singleTag = MeasurementStdDevs.of(0.9, 0.9, Math.toRadians(12.0));
        for (int i = 0; i < 100; i++) {
            double t = i * 0.05;
            rig.stepAthenaOnly(t, vision(
                    13.0 + noise.nextGaussian() * 0.15,
                    4.0 + noise.nextGaussian() * 0.15,
                    Math.PI + noise.nextGaussian() * Math.toRadians(3.0),
                    t - 0.03, singleTag, camera));
        }
        PoseSnapshot pose = rig.athena.pose();
        assertEquals(13.0, pose.xMeters(), 0.3, "x after 5 s of single-tag vision");
        assertEquals(4.0, pose.yMeters(), 0.3, "y after 5 s of single-tag vision");
        assertEquals(0.0, Math.abs(Math.cos(pose.headingRadians()) + 1.0), 0.05, "heading after 5 s: " + Math.toDegrees(pose.headingRadians()));
    }

    /**
     * A camera whose clock is not synced yet can report a capture time in the future. One such frame must not
     * lock that camera out once its timestamps become correct.
     */
    @Test
    void oneFutureTimestampDoesNotLockOutACamera() {
        ParityRig rig = new ParityRig(9.0);
        Object camera = new Object();
        MeasurementStdDevs multiTag = MeasurementStdDevs.of(0.2, 0.2, 0.1);
        rig.stepAthenaOnly(0.00, vision(5.0, 2.0, 0.0, 4000.0, multiTag, camera));
        for (int i = 1; i <= 50; i++) {
            double t = i * 0.05;
            rig.stepAthenaOnly(t, vision(5.0, 2.0, 0.0, t - 0.03, multiTag, camera));
        }
        assertEquals(5.0, rig.athena.pose().xMeters(), 0.2);
        assertEquals(2.0, rig.athena.pose().yMeters(), 0.2);
    }

    @Test
    void kalmanReportsItsHeadingToUpstreamCamerasThatSolveWithIt() {
        ParityRig rig = new ParityRig();
        ca.frc6390.athena.vision.device.PhotonVisionDevice camera =
                new ca.frc6390.athena.vision.device.PhotonVisionDevice("front");
        rig.visionSource.camera = camera;
        assertTrue(Double.isNaN(camera.reportedRobotHeadingRadians()));

        rig.stepAthenaOnly(0.0, vision(1.0, 0.0, 0.6, 0.0, MeasurementStdDevs.of(0.1, 0.1, 0.05), camera));
        rig.stepAthenaOnly(0.02);

        assertEquals(rig.athena.pose().headingRadians(), camera.reportedRobotHeadingRadians(), 1.0e-12);
        assertTrue(camera.reportedRobotHeadingRadians() > 0.0);
    }

    private static VisionSample vision(
            double xMeters,
            double yMeters,
            double headingRadians,
            double timestampSeconds,
            MeasurementStdDevs stdDevs,
            Object source) {
        return new VisionSample(
                new PoseSnapshot(xMeters, yMeters, headingRadians),
                RobotVelocity.zero(),
                timestampSeconds,
                0.02,
                0.05,
                2,
                2.0,
                stdDevs,
                source);
    }

    private static final class ParityRig {
        private final Map<Integer, Double> driveRotations = initializedMap();
        private final Map<Integer, Double> moduleAngles = initializedMap();
        private final TestImu imu = new TestImu();
        private final SwerveModule frontLeft = module(1, 11, 21);
        private final SwerveModule frontRight = module(2, 12, 22);
        private final SwerveModule backLeft = module(3, 13, 23);
        private final SwerveModule backRight = module(4, 14, 24);
        private final SwerveKinematics athenaKinematics = SwerveKinematics.rectangular(
                0.5, 0.5, 5.0, frontLeft, frontRight, backLeft, backRight);
        private final SwerveDriveKinematics wpilibKinematics = new SwerveDriveKinematics(
                new Translation2d(0.25, 0.25),
                new Translation2d(0.25, -0.25),
                new Translation2d(-0.25, 0.25),
                new Translation2d(-0.25, -0.25));
        private final MutablePoseSignal visionSource = new MutablePoseSignal();
        private final List<MutablePoseSignal> cameraSignals = new ArrayList<>();
        private final Localization visionStage;
        private final SwerveOdometry odometry = athenaKinematics.odometry(imu);
        private Localization athena;
        private final ActionContext context = new ActionContext() {
            @Override
            public EncoderHandle encoder(EncoderDevice device) {
                if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated) {
                    return handle(device, driveRotations, integrated.motor().id());
                }
                return handle(device, moduleAngles, device.id());
            }
        };
        private SwerveDrivePoseEstimator wpilib;
        private PoseSnapshot pendingReset;

        private ParityRig() {
            this(Double.POSITIVE_INFINITY, false, false, Double.POSITIVE_INFINITY);
        }

        /** One camera behind the given filters, with a Kalman node tuned by {@code kalmanTuning}. */
        private ParityRig(UnaryOperator<Localization> kalmanTuning, LocalizationFilter... visionFilters) {
            visionStage = Localizations.filter().input(visionSource).filter(visionFilters);
            athena = kalmanTuning.apply(Localizations.kalman()
                    .input(odometry, visionStage)
                    .stateStdDevs(STATE_STD_DEVS.xMeters(), STATE_STD_DEVS.headingRadians())
                    .defaultVisionStdDevs(DEFAULT_VISION_STD_DEVS.xMeters(), DEFAULT_VISION_STD_DEVS.headingRadians()));
        }

        /**
         * Two cameras as separate Kalman inputs ({@code direct}) or fused by covariance intersection first. Each
         * camera has its own signal so {@link #stepCameras} can feed it.
         */
        private ParityRig(int cameraCount, boolean direct) {
            for (int camera = 0; camera < cameraCount; camera++) {
                cameraSignals.add(new MutablePoseSignal());
            }
            visionStage = Localizations.filter().input(visionSource);
            List<PoseSignal> inputs = new ArrayList<>(cameraSignals);
            athena = direct
                    ? Localizations.kalman().input(odometry).input(inputs.toArray(PoseSignal[]::new))
                    : Localizations.kalman().input(odometry, Localizations.covarianceIntersection()
                            .input(inputs.toArray(PoseSignal[]::new)));
            athena = athena
                    .stateStdDevs(STATE_STD_DEVS.xMeters(), STATE_STD_DEVS.headingRadians())
                    .defaultVisionStdDevs(DEFAULT_VISION_STD_DEVS.xMeters(), DEFAULT_VISION_STD_DEVS.headingRadians());
        }

        private ParityRig(double innovationGate) {
            this(innovationGate, false, false, Double.POSITIVE_INFINITY);
        }

        private ParityRig(double innovationGate, boolean translationOnlyVision) {
            this(innovationGate, translationOnlyVision, false, Double.POSITIVE_INFINITY);
        }

        private ParityRig(
                double innovationGate,
                boolean translationOnlyVision,
                double maxHeadingResidualRadians) {
            this(innovationGate, translationOnlyVision, false, maxHeadingResidualRadians);
        }

        private ParityRig(
                double innovationGate,
                boolean translationOnlyVision,
                boolean headingOnlyVision,
                double maxHeadingResidualRadians) {
            PoseSignal configuredVision = translationOnlyVision
                    ? visionSource.translationOnly()
                    : headingOnlyVision ? visionSource.headingOnly() : visionSource;
            visionStage = Localizations.filter().input(configuredVision);
            athena = Localizations.kalman()
                    .input(odometry, visionStage)
                    .stateStdDevs(STATE_STD_DEVS.xMeters(), STATE_STD_DEVS.headingRadians())
                    .defaultVisionStdDevs(DEFAULT_VISION_STD_DEVS.xMeters(), DEFAULT_VISION_STD_DEVS.headingRadians())
                    .maxHeadingDisagreementRadians(maxHeadingResidualRadians)
                    .maxNormalizedVisionResidual(innovationGate);
        }

        /** Steps a two-or-more camera rig: each list is one camera's frames for this cycle. */
        private void stepCameras(double timestampSeconds, List<List<VisionSample>> perCamera) {
            setModuleInputs(distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0));
            imu.yawDegrees = 0.0;
            for (int camera = 0; camera < perCamera.size(); camera++) {
                cameraSignals.get(camera).measurements = List.copyOf(perCamera.get(camera));
            }
            athena.refresh(context, timestampSeconds, 0.02);
        }

        private void stepAthenaOnly(double timestampSeconds, VisionSample... vision) {
            setModuleInputs(distances(0.0, 0.0, 0.0, 0.0), angles(0.0, 0.0, 0.0, 0.0));
            imu.yawDegrees = 0.0;
            visionSource.measurements = List.of(vision);
            athena.refresh(context, timestampSeconds, 0.02);
        }

        private void step(
                double timestampSeconds,
                double[] distancesMeters,
                double[] angleRotations,
                double yawDegrees,
                VisionSample... vision) {
            setModuleInputs(distancesMeters, angleRotations);
            imu.yawDegrees = yawDegrees;
            visionSource.measurements = List.of(vision);
            athena.refresh(context, timestampSeconds, 0.02);

            Rotation2d heading = new Rotation2d(odometry.headingRadians());
            SwerveModulePosition[] positions = wpilibPositions();
            if (wpilib == null || pendingReset != null) {
                PoseSnapshot initial = pendingReset == null ? odometry.pose() : pendingReset;
                wpilib = new SwerveDrivePoseEstimator(
                        wpilibKinematics,
                        heading,
                        positions,
                        toWpilib(initial),
                        VecBuilder.fill(
                                STATE_STD_DEVS.xMeters(),
                                STATE_STD_DEVS.yMeters(),
                                STATE_STD_DEVS.headingRadians()),
                        VecBuilder.fill(
                                DEFAULT_VISION_STD_DEVS.xMeters(),
                                DEFAULT_VISION_STD_DEVS.yMeters(),
                                DEFAULT_VISION_STD_DEVS.headingRadians()));
                pendingReset = null;
            }
            wpilib.updateWithTime(timestampSeconds, heading, positions);
            for (VisionSample sample : vision) {
                wpilib.addVisionMeasurement(
                        toWpilib(sample.pose()),
                        sample.timestampSeconds(),
                        VecBuilder.fill(
                                sample.stdDevs().xMeters(),
                                sample.stdDevs().yMeters(),
                                sample.stdDevs().headingRadians()));
            }
            assertPoseParity(timestampSeconds, athena.pose(), wpilib.getEstimatedPosition());
        }

        private void reset(PoseSnapshot pose) {
            athena.reset(pose).apply(context);
            pendingReset = pose;
        }

        private void setModuleInputs(double[] distancesMeters, double[] angleRotations) {
            if (distancesMeters.length != 4 || angleRotations.length != 4) {
                throw new IllegalArgumentException("Expected four module values.");
            }
            for (int index = 0; index < 4; index++) {
                driveRotations.put(index + 1, distancesMeters[index] * 2.0);
                moduleAngles.put(index + 21, angleRotations[index]);
            }
        }

        private SwerveModulePosition[] wpilibPositions() {
            return odometry.modulePositions().stream()
                    .map(position -> new SwerveModulePosition(
                            position.distanceMeters(),
                            Rotation2d.fromRotations(position.angleRotations())))
                    .toArray(SwerveModulePosition[]::new);
        }

        private static Map<Integer, Double> initializedMap() {
            Map<Integer, Double> values = new HashMap<>();
            for (int id = 1; id <= 24; id++) {
                values.put(id, 0.0);
            }
            return values;
        }

        private static EncoderHandle handle(EncoderDevice device, Map<Integer, Double> values, int id) {
            return new EncoderHandle() {
                @Override
                public EncoderDevice device() {
                    return device;
                }

                @Override
                public double positionRotations() {
                    return values.get(id);
                }

                @Override
                public double absolutePositionRotations() {
                    return values.get(id);
                }

                @Override
                public double velocityRotationsPerSecond() {
                    return 0.0;
                }
            };
        }

        private static SwerveModule module(int driveId, int steerId, int angleId) {
            TestModule module = new TestModule();
            module.drive.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, driveId));
            module.steer.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, steerId));
            module.angle.fill(EncoderDevice.of(EncoderKinds.CANCODER, angleId).measures(Units.Rotations));
            return module;
        }
    }

    private static Pose2d toWpilib(PoseSnapshot pose) {
        return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
    }

    private static void assertPoseParity(double timestampSeconds, PoseSnapshot athena, Pose2d wpilib) {
        assertEquals(wpilib.getX(), athena.xMeters(), PARITY_TOLERANCE, "x at t=" + timestampSeconds);
        assertEquals(wpilib.getY(), athena.yMeters(), PARITY_TOLERANCE, "y at t=" + timestampSeconds);
        double headingError = Math.IEEEremainder(
                athena.headingRadians() - wpilib.getRotation().getRadians(),
                Math.PI * 2.0);
        assertEquals(0.0, headingError, PARITY_TOLERANCE, "heading at t=" + timestampSeconds);
    }

    private static final class MutablePoseSignal implements ca.frc6390.athena.vision.signal.PoseSignal {
        private List<Measurement> measurements = List.of();
        private ca.frc6390.athena.vision.device.CameraDevice camera;

        @Override
        public ca.frc6390.athena.vision.device.CameraDevice camera() {
            return camera;
        }

        @Override
        public List<Measurement> measurements() {
            return measurements;
        }
    }

    private record VisionSample(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            double latencySeconds,
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters,
            MeasurementStdDevs stdDevs,
            Object source) implements PoseMeasurementSample {
    }

    private static final class TestModule extends SwerveModule {
        private TestModule() {
            super(SwerveModuleModel.custom(2.0, 1.0, 1.0 / Math.PI));
        }
    }

    private static final class TestImu implements ImuSource {
        private double yawDegrees;

        @Override public double yawDegrees() { return yawDegrees; }
        @Override public double pitchDegrees() { return 0.0; }
        @Override public double rollDegrees() { return 0.0; }
        @Override public double angleDegrees() { return yawDegrees; }
        @Override public double yawRateDegreesPerSecond() { return 0.0; }
        @Override public double linearAccelerationXG() { return 0.0; }
        @Override public double linearAccelerationYG() { return 0.0; }
        @Override public double linearAccelerationZG() { return 0.0; }
        @Override public void applyYawDegrees(ActionContext context, double yawDegrees) { this.yawDegrees = yawDegrees; }
    }
}
