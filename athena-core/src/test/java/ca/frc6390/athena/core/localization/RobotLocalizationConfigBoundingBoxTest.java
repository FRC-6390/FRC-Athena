package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

final class RobotLocalizationConfigBoundingBoxTest {

    @Test
    void addBoundingBoxWithSameNameReplacesExistingEntry() {
        RobotLocalizationConfig config = new RobotLocalizationConfig();
        PoseBoundingBox2d first = new PoseBoundingBox2d(1.0, 2.0, 3.0, 4.0);
        PoseBoundingBox2d second = new PoseBoundingBox2d(5.0, 6.0, 7.0, 8.0);

        config.addBoundingBox("lane", first);
        config.addBoundingBox("lane", second);

        List<RobotLocalizationConfig.NamedBoundingBox> boxes = config.boundingBoxes();
        assertEquals(1, boxes.size());
        assertEquals("lane", boxes.get(0).name());
        assertEquals(second, boxes.get(0).box());
    }

    @Test
    void addBoundingBoxWithDifferentNameKeepsBoth() {
        RobotLocalizationConfig config = new RobotLocalizationConfig();
        PoseBoundingBox2d lane = new PoseBoundingBox2d(1.0, 2.0, 3.0, 4.0);
        PoseBoundingBox2d source = new PoseBoundingBox2d(5.0, 6.0, 7.0, 8.0);

        config.addBoundingBox("lane", lane);
        config.addBoundingBox("source", source);

        List<RobotLocalizationConfig.NamedBoundingBox> boxes = config.boundingBoxes();
        assertEquals(2, boxes.size());
        assertEquals("lane", boxes.get(0).name());
        assertEquals("source", boxes.get(1).name());
    }
}
