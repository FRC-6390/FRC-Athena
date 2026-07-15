package ca.frc6390.athena.mechanism.sysid;

/** Feedback units accepted by a control characterization routine. */
public enum SysIdUnit {
    ROTATIONS(true),
    DEGREES(true),
    RADIANS(true),
    METERS(false);

    private final boolean angular;

    SysIdUnit(boolean angular) {
        this.angular = angular;
    }

    public boolean angular() {
        return angular;
    }

    double normalizePosition(double value) {
        return switch (this) {
            case ROTATIONS, METERS -> value;
            case DEGREES -> value / 360.0;
            case RADIANS -> value / (2.0 * Math.PI);
        };
    }

    double normalizeVelocity(double value) {
        return normalizePosition(value);
    }
}
