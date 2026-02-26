package ca.frc6390.athena.core.arcp.widgets;

import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;

public class VisionWidgetBuilder extends WidgetBuilderBase<VisionWidgetBuilder> {
    public VisionWidgetBuilder(int signalId) {
        super(signalId, "camera_overlay", "Vision");
        config("streamUrl", "");
        config("showPose", true);
        config("showTargets", true);
        config("showDetections", true);
        config("mirrorX", false);
        config("sourceWidth", 640);
        config("sourceHeight", 360);
    }

    public VisionWidgetBuilder streamSignal(int id) { return signalConfig("streamSignalId", id); }
    public VisionWidgetBuilder streamSignalPath(String path) { return signalPathConfig("streamSignalPath", path); }
    public VisionWidgetBuilder streamUrl(String url) { return config("streamUrl", url != null ? url : ""); }
    public VisionWidgetBuilder poseXSignal(int id) { return signalConfig("poseXSignalId", id); }
    public VisionWidgetBuilder poseXSignalPath(String path) { return signalPathConfig("poseXSignalPath", path); }
    public VisionWidgetBuilder poseYSignal(int id) { return signalConfig("poseYSignalId", id); }
    public VisionWidgetBuilder poseYSignalPath(String path) { return signalPathConfig("poseYSignalPath", path); }
    public VisionWidgetBuilder headingSignal(int id) { return signalConfig("headingSignalId", id); }
    public VisionWidgetBuilder headingSignalPath(String path) { return signalPathConfig("headingSignalPath", path); }
    public VisionWidgetBuilder targetsSignal(int id) { return signalConfig("targetsSignalId", id); }
    public VisionWidgetBuilder targetsSignalPath(String path) { return signalPathConfig("targetsSignalPath", path); }
    public VisionWidgetBuilder detectionsSignal(int id) { return signalConfig("detectionsSignalId", id); }
    public VisionWidgetBuilder detectionsSignalPath(String path) { return signalPathConfig("detectionsSignalPath", path); }
    public VisionWidgetBuilder showPose(boolean show) { return config("showPose", show); }
    public VisionWidgetBuilder showTargets(boolean show) { return config("showTargets", show); }
    public VisionWidgetBuilder showDetections(boolean show) { return config("showDetections", show); }
    public VisionWidgetBuilder mirrorX(boolean mirror) { return config("mirrorX", mirror); }
    public VisionWidgetBuilder sourceSize(int width, int height) {
        config("sourceWidth", Math.max(1, width));
        return config("sourceHeight", Math.max(1, height));
    }

    public ArcpDashboardLayout.Widget build() {
        return buildInternal();
    }
}

