package ca.frc6390.athena.core.localization;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;

public final class RobotLocalizationZonesView {
    private final RobotLocalization<?> owner;

    RobotLocalizationZonesView(RobotLocalization<?> owner) {
        this.owner = owner;
    }

    public Map<String, PoseBoundingBox2d> all() {
        return owner.zonesMap();
    }

    public boolean contains(String name) {
        return name != null && owner.zonesMap().containsKey(name);
    }

    public boolean in(String name) {
        PoseBoundingBox2d box = owner.zonesMap().get(name);
        return box != null && owner.inZone(box);
    }

    public boolean in(PoseBoundingBox2d box) {
        return owner.inZone(box);
    }

    public boolean updateCorners(String name, Translation2d cornerA, Translation2d cornerB) {
        return owner.updateZoneCorners(name, cornerA, cornerB);
    }
}
