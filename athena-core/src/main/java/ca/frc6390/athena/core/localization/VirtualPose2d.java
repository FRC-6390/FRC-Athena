package ca.frc6390.athena.core.localization;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

final class VirtualPose2d {
    private final Supplier<Pose2d> baseSupplier;
    private Transform2d offset = new Transform2d();
    private Pose2d cachedPose = new Pose2d();
    private long cachedRevision = Long.MIN_VALUE;

    VirtualPose2d(Supplier<Pose2d> baseSupplier) {
        this.baseSupplier = baseSupplier;
    }

    Pose2d get(long revision) {
        if (cachedRevision != revision) {
            Pose2d base = safePose(baseSupplier.get());
            cachedPose = base.transformBy(offset);
            cachedRevision = revision;
        }
        return cachedPose;
    }

    void set(Pose2d desiredPose, long revision) {
        Pose2d base = safePose(baseSupplier.get());
        Pose2d safeDesired = safePose(desiredPose);
        offset = new Transform2d(base, safeDesired);
        cachedPose = safeDesired;
        cachedRevision = revision;
    }

    private static Pose2d safePose(Pose2d pose) {
        return pose != null ? pose : new Pose2d();
    }
}
