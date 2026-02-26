package ca.frc6390.athena.arcp.examples;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotCoreConfig;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;

/**
 * RobotCore integration examples for ARCP.
 */
public final class ArcpRobotCoreExamples {
    private ArcpRobotCoreExamples() {}

    public static <T extends RobotDrivetrain<T>> RobotCoreConfig.Builder<T> enableArcpInBuilder(
            RobotCoreConfig.Builder<T> builder,
            String layoutProfileName) {
        builder.core(core -> core.arcp(arcp -> arcp
                .enabled(true)
                .layoutProfileName(layoutProfileName)
                .autoMechanismPages(true)
                .legacyNt4MirrorEnabled(false)));
        return builder;
    }

    public static <T extends RobotDrivetrain<T>> RobotCore.RobotCoreConfig<T> enableArcpInImmutableConfig(
            RobotCore.RobotCoreConfig<T> config,
            String layoutProfileName) {
        RobotCore.ArcpConfig current = config.arcpConfig();
        return config.arcp(new RobotCore.ArcpConfig(
                true,
                layoutProfileName,
                current.autoMechanismPages(),
                current.legacyNt4MirrorEnabled()));
    }

    public static void runtimeStart(RobotCore<?> core) {
        core.arcp().start();
    }

    public static void runtimeStop(RobotCore<?> core) {
        core.arcp().stop();
    }

    public static void runtimeRestart(RobotCore<?> core) {
        core.arcp().restart();
    }

    public static void runtimePublishPage(RobotCore<?> core) {
        core.arcp().page("Operator", page -> page
                .id("tab-operator")
                .name("Operator")
                .widget(ArcpDashboardLayout.Widget.builder()
                        .id("operator-status")
                        .kind("title")
                        .title("Robot Status")
                        .layout(0, 0, 12, 1)
                        .config("text", "Robot Status")
                        .config("color", "#ef4444")
                        .build())
                .widget(ArcpDashboardLayout.Widget.builder()
                        .id("operator-enabled")
                        .kind("toggle")
                        .title("Enabled")
                        .layout(0, 1, 2, 1)
                        .config("bindings", java.util.Map.of("value", "Athena/Drive/Enabled"))
                        .build()));
        core.arcp().publishMechanismPages();
    }

    public static String[] runtimeListProfiles(RobotCore<?> core) {
        return core.arcp().listLayoutProfiles();
    }

    public static void runtimeSaveProfile(RobotCore<?> core, String profileName, String layoutJson) {
        core.arcp().saveLayoutProfile(profileName, layoutJson);
    }

    public static String runtimeLoadProfile(RobotCore<?> core, String profileName) {
        return core.arcp().loadLayoutProfile(profileName);
    }

    public static String runtimeEndpointSummary(RobotCore<?> core) {
        return "running=" + core.arcp().running()
                + ", controlPort=" + core.arcp().controlPort()
                + ", realtimePort=" + core.arcp().realtimePort()
                + ", profile=" + core.arcp().layoutProfileName();
    }
}
