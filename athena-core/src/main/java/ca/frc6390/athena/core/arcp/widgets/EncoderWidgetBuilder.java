package ca.frc6390.athena.core.arcp.widgets;

import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;
import java.util.Objects;

public class EncoderWidgetBuilder extends WidgetBuilderBase<EncoderWidgetBuilder> {
    public EncoderWidgetBuilder(int signalId) {
        super(signalId, "encoder", "Encoder");
        layout(0, 0, 3, 7);
        config("positionViewMode", "continuous");
        config("absoluteViewMode", "zero_to_one");
        config("positionMin", -180.0);
        config("positionMax", 180.0);
        config("absoluteMin", 0.0);
        config("absoluteMax", 1.0);
        config("positionUnit", "");
        config("absoluteUnit", "");
    }

    public EncoderWidgetBuilder positionSignal(int id) { return signalConfig("positionSignalId", id); }
    public EncoderWidgetBuilder positionSignalPath(String path) { return signalPathConfig("positionSignalPath", path); }
    public EncoderWidgetBuilder velocitySignal(int id) { return signalConfig("velocitySignalId", id); }
    public EncoderWidgetBuilder velocitySignalPath(String path) { return signalPathConfig("velocitySignalPath", path); }
    public EncoderWidgetBuilder rateSignal(int id) { return signalConfig("rateSignalId", id); }
    public EncoderWidgetBuilder rateSignalPath(String path) { return signalPathConfig("rateSignalPath", path); }
    public EncoderWidgetBuilder absoluteSignal(int id) { return signalConfig("absoluteSignalId", id); }
    public EncoderWidgetBuilder absoluteSignalPath(String path) { return signalPathConfig("absoluteSignalPath", path); }
    public EncoderWidgetBuilder connectedSignal(int id) { return signalConfig("connectedSignalId", id); }
    public EncoderWidgetBuilder connectedSignalPath(String path) { return signalPathConfig("connectedSignalPath", path); }
    public EncoderWidgetBuilder conversionSignal(int id) { return signalConfig("conversionSignalId", id); }
    public EncoderWidgetBuilder conversionSignalPath(String path) { return signalPathConfig("conversionSignalPath", path); }
    public EncoderWidgetBuilder conversionOffsetSignal(int id) { return signalConfig("conversionOffsetSignalId", id); }
    public EncoderWidgetBuilder conversionOffsetSignalPath(String path) { return signalPathConfig("conversionOffsetSignalPath", path); }
    public EncoderWidgetBuilder discontinuityPointSignal(int id) { return signalConfig("discontinuityPointSignalId", id); }
    public EncoderWidgetBuilder discontinuityPointSignalPath(String path) { return signalPathConfig("discontinuityPointSignalPath", path); }
    public EncoderWidgetBuilder discontinuityRangeSignal(int id) { return signalConfig("discontinuityRangeSignalId", id); }
    public EncoderWidgetBuilder discontinuityRangeSignalPath(String path) { return signalPathConfig("discontinuityRangeSignalPath", path); }
    public EncoderWidgetBuilder ratioSignal(int id) { return signalConfig("ratioSignalId", id); }
    public EncoderWidgetBuilder ratioSignalPath(String path) { return signalPathConfig("ratioSignalPath", path); }
    public EncoderWidgetBuilder offsetSignal(int id) { return signalConfig("offsetSignalId", id); }
    public EncoderWidgetBuilder offsetSignalPath(String path) { return signalPathConfig("offsetSignalPath", path); }
    public EncoderWidgetBuilder canIdSignal(int id) { return signalConfig("canIdSignalId", id); }
    public EncoderWidgetBuilder canIdSignalPath(String path) { return signalPathConfig("canIdSignalPath", path); }
    public EncoderWidgetBuilder canbusSignal(int id) { return signalConfig("canbusSignalId", id); }
    public EncoderWidgetBuilder canbusSignalPath(String path) { return signalPathConfig("canbusSignalPath", path); }
    public EncoderWidgetBuilder typeSignal(int id) { return signalConfig("typeSignalId", id); }
    public EncoderWidgetBuilder typeSignalPath(String path) { return signalPathConfig("typeSignalPath", path); }
    public EncoderWidgetBuilder invertedSignal(int id) { return signalConfig("invertedSignalId", id); }
    public EncoderWidgetBuilder invertedSignalPath(String path) { return signalPathConfig("invertedSignalPath", path); }
    public EncoderWidgetBuilder supportsSimulationSignal(int id) { return signalConfig("supportsSimulationSignalId", id); }
    public EncoderWidgetBuilder supportsSimulationSignalPath(String path) { return signalPathConfig("supportsSimulationSignalPath", path); }
    public EncoderWidgetBuilder rawAbsoluteSignal(int id) { return signalConfig("rawAbsoluteSignalId", id); }
    public EncoderWidgetBuilder rawAbsoluteSignalPath(String path) { return signalPathConfig("rawAbsoluteSignalPath", path); }

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
