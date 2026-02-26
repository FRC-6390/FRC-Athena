package ca.frc6390.athena.core.arcp.widgets;

import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;

public class DioWidgetBuilder extends WidgetBuilderBase<DioWidgetBuilder> {
    public DioWidgetBuilder(int signalId) {
        super(signalId, "dio", "DIO");
        config("showOtherFields", true);
    }

    public DioWidgetBuilder showOtherFields(boolean show) { return config("showOtherFields", show); }
    public DioWidgetBuilder valueSignal(int id) { return signalConfig("valueSignalId", id); }
    public DioWidgetBuilder valueSignalPath(String path) { return signalPathConfig("valueSignalPath", path); }
    public DioWidgetBuilder outputSignal(int id) { return signalConfig("outputSignalId", id); }
    public DioWidgetBuilder outputSignalPath(String path) { return signalPathConfig("outputSignalPath", path); }
    public DioWidgetBuilder invertedSignal(int id) { return signalConfig("invertedSignalId", id); }
    public DioWidgetBuilder invertedSignalPath(String path) { return signalPathConfig("invertedSignalPath", path); }
    public DioWidgetBuilder channelSignal(int id) { return signalConfig("channelSignalId", id); }
    public DioWidgetBuilder channelSignalPath(String path) { return signalPathConfig("channelSignalPath", path); }
    public DioWidgetBuilder portSignal(int id) { return signalConfig("portSignalId", id); }
    public DioWidgetBuilder portSignalPath(String path) { return signalPathConfig("portSignalPath", path); }
    public DioWidgetBuilder modeSignal(int id) { return signalConfig("modeSignalId", id); }
    public DioWidgetBuilder modeSignalPath(String path) { return signalPathConfig("modeSignalPath", path); }
    public DioWidgetBuilder nameSignal(int id) { return signalConfig("nameSignalId", id); }
    public DioWidgetBuilder nameSignalPath(String path) { return signalPathConfig("nameSignalPath", path); }
    public DioWidgetBuilder typeSignal(int id) { return signalConfig("typeSignalId", id); }
    public DioWidgetBuilder typeSignalPath(String path) { return signalPathConfig("typeSignalPath", path); }

    public ArcpDashboardLayout.Widget build() {
        return buildInternal();
    }
}

