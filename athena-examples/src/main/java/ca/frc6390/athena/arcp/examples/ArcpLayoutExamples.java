package ca.frc6390.athena.arcp.examples;

import ca.frc6390.athena.core.arcp.ARCP;
import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;
import ca.frc6390.athena.core.arcp.ArcpDeviceWidgets;

/**
 * Layout/widget examples for ARCP host dashboards.
 */
public final class ArcpLayoutExamples {
    private ArcpLayoutExamples() {}

    public static void publishDslLayout(ARCP arcp) {
        ARCP.NumberInputWidgetSpec kpInput = ARCP.Widgets.numberInput()
                .min(0.0)
                .max(2.0)
                .step(0.001);

        arcp.layout("atheana-generated")
                .page("Drivetrain")
                .gridColumns(12)
                .put(ARCP.Widgets.custom("title")
                        .id("drive-title")
                        .title("Drivetrain")
                        .layout(0, 0, 12, 1)
                        .config("text", "Drivetrain")
                        .config("color", "#ef4444"))
                .put(ARCP.Widgets.motor()
                        .id("motor-front-left")
                        .title("Front Left Motor")
                        .topic("Athena/Mechanisms/Drive/Motors/FrontLeft")
                        .layout(0, 1, 3, 5))
                .put(ARCP.Widgets.encoder()
                        .id("encoder-front-left")
                        .title("Front Left Encoder")
                        .topic("Athena/Mechanisms/Drive/Encoders/FrontLeft")
                        .layout(3, 1, 3, 7))
                .put(ARCP.Widgets.imu()
                        .id("imu-main")
                        .title("Main IMU")
                        .topic("Athena/Mechanisms/Drive/Imu")
                        .layout(6, 1, 3, 5))
                .put(ARCP.Widgets.custom("team6390/swerve-module")
                        .id("swerve-front-left")
                        .title("FL Module")
                        .layout(9, 1, 3, 3)
                        .bind("speed", "Athena/Mechanisms/Drive/Swerve/FrontLeft/Speed")
                        .bind("angle", "Athena/Mechanisms/Drive/Swerve/FrontLeft/Angle")
                        .config("showVector", true))
                .endPage()
                .page("Commands")
                .gridAuto()
                .put(kpInput
                        .id("drive-kp")
                        .title("Drive kP")
                        .layout(0, 0, 3, 1)
                        .bind("value", "Athena/Drive/Kp"))
                .put(ARCP.Widgets.toggle()
                        .id("drive-enabled")
                        .title("Drive Enabled")
                        .layout(3, 0, 2, 1)
                        .bind("value", "Athena/Drive/Enabled"))
                .put(ARCP.Widgets.button("Zero Gyro")
                        .id("zero-gyro")
                        .layout(5, 0, 2, 1)
                        .bind("invoke", "Athena/Drive/ZeroGyro"))
                .endPage()
                .publish();
    }

    public static String buildLayoutJsonWithTypedDeviceBuilders() {
        ArcpDashboardLayout layout = ArcpDashboardLayout.builder()
                .activeTabId("tab-hardware")
                .page(page -> page
                        .id("tab-hardware")
                        .name("Hardware")
                        .widget(ArcpDeviceWidgets.motor("Athena/Mechanisms/Drive/Motors/FrontLeft", motor -> motor
                                .id("hw-motor-front-left")
                                .title("Front Left Motor")
                                .layout(0, 0, 3, 5)
                                .showOtherFields(true)
                                .outputSignalPath("Athena/Mechanisms/Drive/Motors/FrontLeft/output")
                                .velocitySignalPath("Athena/Mechanisms/Drive/Motors/FrontLeft/velocity")
                                .positionSignalPath("Athena/Mechanisms/Drive/Motors/FrontLeft/position")
                                .currentSignalPath("Athena/Mechanisms/Drive/Motors/FrontLeft/current")
                                .temperatureSignalPath("Athena/Mechanisms/Drive/Motors/FrontLeft/temperature")
                                .commandSignalPath("Athena/Mechanisms/Drive/Motors/FrontLeft/command")))
                        .widget(ArcpDeviceWidgets.encoder("Athena/Mechanisms/Drive/Encoders/FrontLeft", encoder -> encoder
                                .id("hw-encoder-front-left")
                                .title("Front Left Encoder")
                                .layout(3, 0, 3, 7)
                                .positionSignalPath("Athena/Mechanisms/Drive/Encoders/FrontLeft/position")
                                .velocitySignalPath("Athena/Mechanisms/Drive/Encoders/FrontLeft/velocity")
                                .absoluteSignalPath("Athena/Mechanisms/Drive/Encoders/FrontLeft/absolute")
                                .conversionSignalPath("Athena/Mechanisms/Drive/Encoders/FrontLeft/conversion")
                                .conversionOffsetSignalPath("Athena/Mechanisms/Drive/Encoders/FrontLeft/conversionOffset")
                                .positionViewMode("continuous")
                                .absoluteViewMode("zero_to_one")))
                        .widget(ArcpDeviceWidgets.imu("Athena/Mechanisms/Drive/Imu", imu -> imu
                                .id("hw-imu")
                                .title("Drive IMU")
                                .layout(6, 0, 3, 5)
                                .yawSignalPath("Athena/Mechanisms/Drive/Imu/yaw")
                                .pitchSignalPath("Athena/Mechanisms/Drive/Imu/pitch")
                                .rollSignalPath("Athena/Mechanisms/Drive/Imu/roll")
                                .gyroSignalPath("Athena/Mechanisms/Drive/Imu/gyro")
                                .accelSignalPath("Athena/Mechanisms/Drive/Imu/accel")
                                .orientationViewMode("3d")
                                .units("deg")))
                        .widget(ArcpDeviceWidgets.dio("Athena/Mechanisms/Drive/Dio/BeamBreak", dio -> dio
                                .id("hw-dio-beam-break")
                                .title("Beam Break")
                                .layout(9, 0, 3, 3)
                                .valueSignalPath("Athena/Mechanisms/Drive/Dio/BeamBreak/value")
                                .modeSignalPath("Athena/Mechanisms/Drive/Dio/BeamBreak/mode")
                                .channelSignalPath("Athena/Mechanisms/Drive/Dio/BeamBreak/channel")))
                        .widget(ArcpDeviceWidgets.vision("Athena/Vision/FrontCam", vision -> vision
                                .id("hw-front-cam")
                                .title("Front Camera")
                                .layout(0, 7, 6, 4)
                                .streamSignalPath("Athena/Vision/FrontCam/stream")
                                .poseXSignalPath("Athena/Vision/FrontCam/pose/x")
                                .poseYSignalPath("Athena/Vision/FrontCam/pose/y")
                                .headingSignalPath("Athena/Vision/FrontCam/pose/heading")
                                .detectionsSignalPath("Athena/Vision/FrontCam/detections")
                                .targetsSignalPath("Athena/Vision/FrontCam/targets")
                                .sourceSize(960, 540)
                                .showTargets(true)
                                .showDetections(true))))
                .build();

        return layout.toJson();
    }
}
