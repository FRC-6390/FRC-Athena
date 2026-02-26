package ca.frc6390.athena.core.arcp.widgets;

import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;
import java.util.Objects;

public class ImuWidgetBuilder extends WidgetBuilderBase<ImuWidgetBuilder> {
    public ImuWidgetBuilder(int signalId) {
        super(signalId, "imu", "IMU");
        config("orientationViewMode", "auto");
        config("accelViewMode", "auto");
        config("gyroViewMode", "auto");
        config("magViewMode", "auto");
        config("units", "deg");
    }

    public ImuWidgetBuilder rollSignal(int id) { return signalConfig("rollSignalId", id); }
    public ImuWidgetBuilder rollSignalPath(String path) { return signalPathConfig("rollSignalPath", path); }
    public ImuWidgetBuilder pitchSignal(int id) { return signalConfig("pitchSignalId", id); }
    public ImuWidgetBuilder pitchSignalPath(String path) { return signalPathConfig("pitchSignalPath", path); }
    public ImuWidgetBuilder yawSignal(int id) { return signalConfig("yawSignalId", id); }
    public ImuWidgetBuilder yawSignalPath(String path) { return signalPathConfig("yawSignalPath", path); }
    public ImuWidgetBuilder headingSignal(int id) { return signalConfig("headingSignalId", id); }
    public ImuWidgetBuilder headingSignalPath(String path) { return signalPathConfig("headingSignalPath", path); }
    public ImuWidgetBuilder connectedSignal(int id) { return signalConfig("connectedSignalId", id); }
    public ImuWidgetBuilder connectedSignalPath(String path) { return signalPathConfig("connectedSignalPath", path); }
    public ImuWidgetBuilder accelSignal(int id) { return signalConfig("accelSignalId", id); }
    public ImuWidgetBuilder accelSignalPath(String path) { return signalPathConfig("accelSignalPath", path); }
    public ImuWidgetBuilder gyroSignal(int id) { return signalConfig("gyroSignalId", id); }
    public ImuWidgetBuilder gyroSignalPath(String path) { return signalPathConfig("gyroSignalPath", path); }
    public ImuWidgetBuilder magSignal(int id) { return signalConfig("magSignalId", id); }
    public ImuWidgetBuilder magSignalPath(String path) { return signalPathConfig("magSignalPath", path); }
    public ImuWidgetBuilder canIdSignal(int id) { return signalConfig("canIdSignalId", id); }
    public ImuWidgetBuilder canIdSignalPath(String path) { return signalPathConfig("canIdSignalPath", path); }
    public ImuWidgetBuilder canbusSignal(int id) { return signalConfig("canbusSignalId", id); }
    public ImuWidgetBuilder canbusSignalPath(String path) { return signalPathConfig("canbusSignalPath", path); }
    public ImuWidgetBuilder typeSignal(int id) { return signalConfig("typeSignalId", id); }
    public ImuWidgetBuilder typeSignalPath(String path) { return signalPathConfig("typeSignalPath", path); }
    public ImuWidgetBuilder invertedSignal(int id) { return signalConfig("invertedSignalId", id); }
    public ImuWidgetBuilder invertedSignalPath(String path) { return signalPathConfig("invertedSignalPath", path); }
    public ImuWidgetBuilder orientationViewMode(String mode) { return config("orientationViewMode", Objects.requireNonNull(mode)); }
    public ImuWidgetBuilder accelViewMode(String mode) { return config("accelViewMode", Objects.requireNonNull(mode)); }
    public ImuWidgetBuilder gyroViewMode(String mode) { return config("gyroViewMode", Objects.requireNonNull(mode)); }
    public ImuWidgetBuilder magViewMode(String mode) { return config("magViewMode", Objects.requireNonNull(mode)); }
    public ImuWidgetBuilder units(String units) { return config("units", Objects.requireNonNull(units)); }

    public ArcpDashboardLayout.Widget build() {
        return buildInternal();
    }
}

