package ca.frc6390.athena.vendor.limelight;

/**
 * Limelight target values needed by Athena.
 *
 * @param hasTarget whether Limelight currently sees a target
 * @param tagId AprilTag id, or {@code -1} when unavailable
 * @param txDegrees horizontal offset
 * @param tyDegrees vertical offset
 * @param distanceMeters estimated distance
 * @param targetArea target area percentage
 */
record LimelightTarget(
        boolean hasTarget,
        int tagId,
        double txDegrees,
        double tyDegrees,
        double distanceMeters,
        double targetArea) {
    /**
     * Creates an AprilTag-backed target.
     *
     * @param tagId AprilTag id
     * @param txDegrees horizontal offset
     * @param tyDegrees vertical offset
     * @param distanceMeters estimated distance
     * @param targetArea target area percentage
     * @return target
     */
    public static LimelightTarget aprilTag(
            int tagId,
            double txDegrees,
            double tyDegrees,
            double distanceMeters,
            double targetArea) {
        return new LimelightTarget(true, tagId, txDegrees, tyDegrees, distanceMeters, targetArea);
    }

    /**
     * Creates a no-target result.
     *
     * @return no-target result
     */
    public static LimelightTarget noTarget() {
        return new LimelightTarget(false, -1, 0.0, 0.0, 0.0, 0.0);
    }

    public LimelightTarget {
        txDegrees = finiteOrZero(txDegrees);
        tyDegrees = finiteOrZero(tyDegrees);
        distanceMeters = Math.max(0.0, finiteOrZero(distanceMeters));
        targetArea = Math.max(0.0, finiteOrZero(targetArea));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
