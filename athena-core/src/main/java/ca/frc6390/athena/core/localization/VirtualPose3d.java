package ca.frc6390.athena.core.localization;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

final class VirtualPose3d {
    private final Supplier<Pose3d> baseSupplier;
    private Transform3d offset = new Transform3d();
    private Pose3d cachedPose = new Pose3d();
    private long cachedRevision = Long.MIN_VALUE;

    VirtualPose3d(Supplier<Pose3d> baseSupplier) {
        this.baseSupplier = baseSupplier;
    }

    Pose3d get(long revision) {
        if (cachedRevision != revision) {
            Pose3d base = safePose(baseSupplier.get());
            cachedPose = base.transformBy(offset);
            cachedRevision = revision;
        }
        return cachedPose;
    }

    void set(Pose3d desiredPose, long revision) {
        Pose3d base = safePose(baseSupplier.get());
        Pose3d safeDesired = safePose(desiredPose);
        offset = new Transform3d(base, safeDesired);
        cachedPose = safeDesired;
        cachedRevision = revision;
    }

    private static Pose3d safePose(Pose3d pose) {
        return pose != null ? pose : new Pose3d();
    }
}
