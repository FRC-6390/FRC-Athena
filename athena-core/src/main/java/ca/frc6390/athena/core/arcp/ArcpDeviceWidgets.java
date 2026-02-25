package ca.frc6390.athena.core.arcp;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;

/**
 * Typed ARCP widget config builders for hardware-centric widgets.
 *
 * <p>These builders let teams define which widget is used for each device and
 * exactly which signal IDs bind into each widget field.
 */
public final class ArcpDeviceWidgets {
    private ArcpDeviceWidgets() {
    }

    public static ArcpDashboardLayout.Widget motor(
            int signalId,
            Consumer<MotorWidgetBuilder> section) {
        MotorWidgetBuilder builder = new MotorWidgetBuilder(signalId);
        if (section != null) {
            section.accept(builder);
        }
        return builder.build();
    }

    public static ArcpDashboardLayout.Widget encoder(
            int signalId,
            Consumer<EncoderWidgetBuilder> section) {
        EncoderWidgetBuilder builder = new EncoderWidgetBuilder(signalId);
        if (section != null) {
            section.accept(builder);
        }
        return builder.build();
    }

    public static ArcpDashboardLayout.Widget imu(
            int signalId,
            Consumer<ImuWidgetBuilder> section) {
        ImuWidgetBuilder builder = new ImuWidgetBuilder(signalId);
        if (section != null) {
            section.accept(builder);
        }
        return builder.build();
    }

    public static ArcpDashboardLayout.Widget dio(
            int signalId,
            Consumer<DioWidgetBuilder> section) {
        DioWidgetBuilder builder = new DioWidgetBuilder(signalId);
        if (section != null) {
            section.accept(builder);
        }
        return builder.build();
    }

    public static ArcpDashboardLayout.Widget vision(
            int signalId,
            Consumer<VisionWidgetBuilder> section) {
        VisionWidgetBuilder builder = new VisionWidgetBuilder(signalId);
        if (section != null) {
            section.accept(builder);
        }
        return builder.build();
    }

    private abstract static class WidgetBuilderBase<Self extends WidgetBuilderBase<Self>> {
        protected final int signalId;
        protected final String kind;
        protected String id;
        protected String title;
        protected ArcpDashboardLayout.LayoutRect layout = new ArcpDashboardLayout.LayoutRect(0, 0, 8, 6);
        protected final Map<String, Object> config = new LinkedHashMap<>();

        protected WidgetBuilderBase(int signalId, String kind, String defaultTitle) {
            this.signalId = signalId;
            this.kind = kind;
            this.title = defaultTitle;
        }

        @SuppressWarnings("unchecked")
        private Self self() {
            return (Self) this;
        }

        public Self id(String id) {
            this.id = id;
            return self();
        }

        public Self title(String title) {
            if (title != null && !title.isBlank()) {
                this.title = title.trim();
            }
            return self();
        }

        public Self layout(int x, int y, int w, int h) {
            this.layout = new ArcpDashboardLayout.LayoutRect(x, y, w, h);
            return self();
        }

        protected Self config(String key, Object value) {
            if (key == null || key.isBlank()) {
                return self();
            }
            config.put(key, value);
            return self();
        }

        protected Self signalConfig(String key, int signalId) {
            return config(key, signalId);
        }

        protected ArcpDashboardLayout.Widget buildInternal() {
            ArcpDashboardLayout.Widget.Builder widget = ArcpDashboardLayout.Widget.builder()
                    .signalId(signalId)
                    .kind(kind)
                    .title(title)
                    .layout(layout.x(), layout.y(), layout.w(), layout.h())
                    .config(config);
            if (id != null && !id.isBlank()) {
                widget.id(id);
            }
            return widget.build();
        }
    }

    public static final class MotorWidgetBuilder extends WidgetBuilderBase<MotorWidgetBuilder> {
        public MotorWidgetBuilder(int signalId) {
            super(signalId, "motor", "Motor");
            config("showOtherFields", false);
        }

        public MotorWidgetBuilder showOtherFields(boolean show) {
            return config("showOtherFields", show);
        }

        public MotorWidgetBuilder outputSignal(int id) { return signalConfig("outputSignalId", id); }
        public MotorWidgetBuilder velocitySignal(int id) { return signalConfig("velocitySignalId", id); }
        public MotorWidgetBuilder positionSignal(int id) { return signalConfig("positionSignalId", id); }
        public MotorWidgetBuilder currentSignal(int id) { return signalConfig("currentSignalId", id); }
        public MotorWidgetBuilder temperatureSignal(int id) { return signalConfig("temperatureSignalId", id); }
        public MotorWidgetBuilder voltageSignal(int id) { return signalConfig("voltageSignalId", id); }
        public MotorWidgetBuilder commandSignal(int id) { return signalConfig("commandSignalId", id); }
        public MotorWidgetBuilder connectedSignal(int id) { return signalConfig("connectedSignalId", id); }
        public MotorWidgetBuilder stalledSignal(int id) { return signalConfig("stalledSignalId", id); }
        public MotorWidgetBuilder canIdSignal(int id) { return signalConfig("canIdSignalId", id); }
        public MotorWidgetBuilder canbusSignal(int id) { return signalConfig("canbusSignalId", id); }
        public MotorWidgetBuilder typeSignal(int id) { return signalConfig("typeSignalId", id); }
        public MotorWidgetBuilder neutralModeSignal(int id) { return signalConfig("neutralModeSignalId", id); }
        public MotorWidgetBuilder currentLimitSignal(int id) { return signalConfig("currentLimitSignalId", id); }
        public MotorWidgetBuilder invertedSignal(int id) { return signalConfig("invertedSignalId", id); }
        public MotorWidgetBuilder brakeModeSignal(int id) { return signalConfig("brakeModeSignalId", id); }

        public ArcpDashboardLayout.Widget build() {
            return buildInternal();
        }
    }

    public static final class EncoderWidgetBuilder extends WidgetBuilderBase<EncoderWidgetBuilder> {
        public EncoderWidgetBuilder(int signalId) {
            super(signalId, "encoder", "Encoder");
            config("positionViewMode", "continuous");
            config("absoluteViewMode", "zero_to_one");
            config("positionMin", -180.0);
            config("positionMax", 180.0);
            config("absoluteMin", 0.0);
            config("absoluteMax", 1.0);
            config("positionUnit", "rot");
            config("absoluteUnit", "rot");
        }

        public EncoderWidgetBuilder positionSignal(int id) { return signalConfig("positionSignalId", id); }
        public EncoderWidgetBuilder velocitySignal(int id) { return signalConfig("velocitySignalId", id); }
        public EncoderWidgetBuilder absoluteSignal(int id) { return signalConfig("absoluteSignalId", id); }
        public EncoderWidgetBuilder connectedSignal(int id) { return signalConfig("connectedSignalId", id); }
        public EncoderWidgetBuilder ratioSignal(int id) { return signalConfig("ratioSignalId", id); }
        public EncoderWidgetBuilder offsetSignal(int id) { return signalConfig("offsetSignalId", id); }
        public EncoderWidgetBuilder canIdSignal(int id) { return signalConfig("canIdSignalId", id); }
        public EncoderWidgetBuilder canbusSignal(int id) { return signalConfig("canbusSignalId", id); }
        public EncoderWidgetBuilder typeSignal(int id) { return signalConfig("typeSignalId", id); }
        public EncoderWidgetBuilder invertedSignal(int id) { return signalConfig("invertedSignalId", id); }
        public EncoderWidgetBuilder supportsSimulationSignal(int id) { return signalConfig("supportsSimulationSignalId", id); }
        public EncoderWidgetBuilder rawAbsoluteSignal(int id) { return signalConfig("rawAbsoluteSignalId", id); }

        public EncoderWidgetBuilder positionViewMode(String mode) { return config("positionViewMode", Objects.requireNonNull(mode)); }
        public EncoderWidgetBuilder absoluteViewMode(String mode) { return config("absoluteViewMode", Objects.requireNonNull(mode)); }
        public EncoderWidgetBuilder positionRange(double min, double max, String unit) {
            config("positionMin", min);
            config("positionMax", max);
            return config("positionUnit", unit);
        }

        public EncoderWidgetBuilder absoluteRange(double min, double max, String unit) {
            config("absoluteMin", min);
            config("absoluteMax", max);
            return config("absoluteUnit", unit);
        }

        public ArcpDashboardLayout.Widget build() {
            return buildInternal();
        }
    }

    public static final class ImuWidgetBuilder extends WidgetBuilderBase<ImuWidgetBuilder> {
        public ImuWidgetBuilder(int signalId) {
            super(signalId, "imu", "IMU");
            config("orientationViewMode", "auto");
            config("accelViewMode", "auto");
            config("gyroViewMode", "auto");
            config("magViewMode", "auto");
            config("units", "deg");
        }

        public ImuWidgetBuilder rollSignal(int id) { return signalConfig("rollSignalId", id); }
        public ImuWidgetBuilder pitchSignal(int id) { return signalConfig("pitchSignalId", id); }
        public ImuWidgetBuilder yawSignal(int id) { return signalConfig("yawSignalId", id); }
        public ImuWidgetBuilder headingSignal(int id) { return signalConfig("headingSignalId", id); }
        public ImuWidgetBuilder connectedSignal(int id) { return signalConfig("connectedSignalId", id); }
        public ImuWidgetBuilder accelSignal(int id) { return signalConfig("accelSignalId", id); }
        public ImuWidgetBuilder gyroSignal(int id) { return signalConfig("gyroSignalId", id); }
        public ImuWidgetBuilder magSignal(int id) { return signalConfig("magSignalId", id); }
        public ImuWidgetBuilder canIdSignal(int id) { return signalConfig("canIdSignalId", id); }
        public ImuWidgetBuilder canbusSignal(int id) { return signalConfig("canbusSignalId", id); }
        public ImuWidgetBuilder typeSignal(int id) { return signalConfig("typeSignalId", id); }
        public ImuWidgetBuilder invertedSignal(int id) { return signalConfig("invertedSignalId", id); }
        public ImuWidgetBuilder orientationViewMode(String mode) { return config("orientationViewMode", Objects.requireNonNull(mode)); }
        public ImuWidgetBuilder accelViewMode(String mode) { return config("accelViewMode", Objects.requireNonNull(mode)); }
        public ImuWidgetBuilder gyroViewMode(String mode) { return config("gyroViewMode", Objects.requireNonNull(mode)); }
        public ImuWidgetBuilder magViewMode(String mode) { return config("magViewMode", Objects.requireNonNull(mode)); }
        public ImuWidgetBuilder units(String units) { return config("units", Objects.requireNonNull(units)); }

        public ArcpDashboardLayout.Widget build() {
            return buildInternal();
        }
    }

    public static final class DioWidgetBuilder extends WidgetBuilderBase<DioWidgetBuilder> {
        public DioWidgetBuilder(int signalId) {
            super(signalId, "dio", "DIO");
            config("showOtherFields", true);
        }

        public DioWidgetBuilder showOtherFields(boolean show) { return config("showOtherFields", show); }
        public DioWidgetBuilder valueSignal(int id) { return signalConfig("valueSignalId", id); }
        public DioWidgetBuilder outputSignal(int id) { return signalConfig("outputSignalId", id); }
        public DioWidgetBuilder invertedSignal(int id) { return signalConfig("invertedSignalId", id); }
        public DioWidgetBuilder channelSignal(int id) { return signalConfig("channelSignalId", id); }
        public DioWidgetBuilder portSignal(int id) { return signalConfig("portSignalId", id); }
        public DioWidgetBuilder modeSignal(int id) { return signalConfig("modeSignalId", id); }
        public DioWidgetBuilder nameSignal(int id) { return signalConfig("nameSignalId", id); }
        public DioWidgetBuilder typeSignal(int id) { return signalConfig("typeSignalId", id); }

        public ArcpDashboardLayout.Widget build() {
            return buildInternal();
        }
    }

    public static final class VisionWidgetBuilder extends WidgetBuilderBase<VisionWidgetBuilder> {
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
        public VisionWidgetBuilder streamUrl(String url) { return config("streamUrl", url != null ? url : ""); }
        public VisionWidgetBuilder poseXSignal(int id) { return signalConfig("poseXSignalId", id); }
        public VisionWidgetBuilder poseYSignal(int id) { return signalConfig("poseYSignalId", id); }
        public VisionWidgetBuilder headingSignal(int id) { return signalConfig("headingSignalId", id); }
        public VisionWidgetBuilder targetsSignal(int id) { return signalConfig("targetsSignalId", id); }
        public VisionWidgetBuilder detectionsSignal(int id) { return signalConfig("detectionsSignalId", id); }
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
}
