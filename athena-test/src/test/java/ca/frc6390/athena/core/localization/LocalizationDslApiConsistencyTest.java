package ca.frc6390.athena.core.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.lang.reflect.Method;
import java.util.List;
import java.util.function.Consumer;

import ca.frc6390.athena.core.RobotCoreConfig;
import org.junit.jupiter.api.Test;

final class LocalizationDslApiConsistencyTest {

    @Test
    void localizationSectionsExposeDslPoseApisOnly() throws Exception {
        Method coreDsl = RobotCoreConfig.LocalizationSection.class.getMethod(
                "pose",
                String.class,
                Consumer.class);
        assertEquals(RobotCoreConfig.LocalizationSection.class, coreDsl.getReturnType());

        Method posesDsl = RobotLocalizationConfig.PosesSection.class.getMethod(
                "pose",
                String.class,
                Consumer.class);
        assertEquals(RobotLocalizationConfig.PosesSection.class, posesDsl.getReturnType());

        assertThrows(
                NoSuchMethodException.class,
                () -> RobotCoreConfig.LocalizationSection.class.getMethod("poseConfig", PoseConfig.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> RobotLocalizationConfig.PosesSection.class.getMethod("pose", PoseConfig.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> RobotLocalizationConfig.PosesSection.class.getMethod("poseConfigs", List.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> RobotLocalizationConfig.ConfigSection.class.getMethod("pose", PoseConfig.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> RobotLocalizationConfig.ConfigSection.class.getMethod("poseConfigs", List.class));
    }

    @Test
    void poseConfigKeepsDataRecordOnlyWithoutLegacyFactoriesOrFluentMutators() throws Exception {
        Method nameAccessor = PoseConfig.class.getMethod("name");
        assertEquals(String.class, nameAccessor.getReturnType());

        assertThrows(
                NoSuchMethodException.class,
                () -> PoseConfig.class.getMethod("odometry", String.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> PoseConfig.class.getMethod("defaults", String.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> PoseConfig.class.getMethod("vision", String.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> PoseConfig.class.getMethod("custom", String.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> PoseConfig.class.getMethod("withBackendOverride", RobotLocalizationConfig.BackendConfig.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> PoseConfig.class.getMethod("withNetworkTablesPublishing", boolean.class));
        assertThrows(
                NoSuchMethodException.class,
                () -> PoseConfig.class.getMethod("withActive", boolean.class));
    }
}
