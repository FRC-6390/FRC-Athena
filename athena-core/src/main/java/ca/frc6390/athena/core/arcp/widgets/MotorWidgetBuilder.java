package ca.frc6390.athena.core.arcp.widgets;

import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;

public class MotorWidgetBuilder extends WidgetBuilderBase<MotorWidgetBuilder> {
    public MotorWidgetBuilder(int signalId) {
        super(signalId, "motor", "Motor");
        layout(0, 0, 3, 5);
        config("showOtherFields", false);
    }

    public MotorWidgetBuilder showOtherFields(boolean show) {
        return config("showOtherFields", show);
    }

    public MotorWidgetBuilder outputSignal(int id) {
        return signalConfig("outputSignalId", id);
    }

    public MotorWidgetBuilder outputSignalPath(String path) {
        return signalPathConfig("outputSignalPath", path);
    }

    public MotorWidgetBuilder velocitySignal(int id) {
        return signalConfig("velocitySignalId", id);
    }

    public MotorWidgetBuilder velocitySignalPath(String path) {
        return signalPathConfig("velocitySignalPath", path);
    }

    public MotorWidgetBuilder positionSignal(int id) {
        return signalConfig("positionSignalId", id);
    }

    public MotorWidgetBuilder positionSignalPath(String path) {
        return signalPathConfig("positionSignalPath", path);
    }

    public MotorWidgetBuilder currentSignal(int id) {
        return signalConfig("currentSignalId", id);
    }

    public MotorWidgetBuilder currentSignalPath(String path) {
        return signalPathConfig("currentSignalPath", path);
    }

    public MotorWidgetBuilder temperatureSignal(int id) {
        return signalConfig("temperatureSignalId", id);
    }

    public MotorWidgetBuilder temperatureSignalPath(String path) {
        return signalPathConfig("temperatureSignalPath", path);
    }

    public MotorWidgetBuilder voltageSignal(int id) {
        return signalConfig("voltageSignalId", id);
    }

    public MotorWidgetBuilder voltageSignalPath(String path) {
        return signalPathConfig("voltageSignalPath", path);
    }

    public MotorWidgetBuilder commandSignal(int id) {
        return signalConfig("commandSignalId", id);
    }

    public MotorWidgetBuilder commandSignalPath(String path) {
        return signalPathConfig("commandSignalPath", path);
    }

    public MotorWidgetBuilder connectedSignal(int id) {
        return signalConfig("connectedSignalId", id);
    }

    public MotorWidgetBuilder connectedSignalPath(String path) {
        return signalPathConfig("connectedSignalPath", path);
    }

    public MotorWidgetBuilder stalledSignal(int id) {
        return signalConfig("stalledSignalId", id);
    }

    public MotorWidgetBuilder stalledSignalPath(String path) {
        return signalPathConfig("stalledSignalPath", path);
    }

    public MotorWidgetBuilder canIdSignal(int id) {
        return signalConfig("canIdSignalId", id);
    }

    public MotorWidgetBuilder canIdSignalPath(String path) {
        return signalPathConfig("canIdSignalPath", path);
    }

    public MotorWidgetBuilder canbusSignal(int id) {
        return signalConfig("canbusSignalId", id);
    }

    public MotorWidgetBuilder canbusSignalPath(String path) {
        return signalPathConfig("canbusSignalPath", path);
    }

    public MotorWidgetBuilder typeSignal(int id) {
        return signalConfig("typeSignalId", id);
    }

    public MotorWidgetBuilder typeSignalPath(String path) {
        return signalPathConfig("typeSignalPath", path);
    }

    public MotorWidgetBuilder neutralModeSignal(int id) {
        return signalConfig("neutralModeSignalId", id);
    }

    public MotorWidgetBuilder neutralModeSignalPath(String path) {
        return signalPathConfig("neutralModeSignalPath", path);
    }

    public MotorWidgetBuilder currentLimitSignal(int id) {
        return signalConfig("currentLimitSignalId", id);
    }

    public MotorWidgetBuilder currentLimitSignalPath(String path) {
        return signalPathConfig("currentLimitSignalPath", path);
    }

    public MotorWidgetBuilder invertedSignal(int id) {
        return signalConfig("invertedSignalId", id);
    }

    public MotorWidgetBuilder invertedSignalPath(String path) {
        return signalPathConfig("invertedSignalPath", path);
    }

    public MotorWidgetBuilder brakeModeSignal(int id) {
        return signalConfig("brakeModeSignalId", id);
    }

    public MotorWidgetBuilder brakeModeSignalPath(String path) {
        return signalPathConfig("brakeModeSignalPath", path);
    }

    public ArcpDashboardLayout.Widget build() {
        return buildInternal();
    }
}

