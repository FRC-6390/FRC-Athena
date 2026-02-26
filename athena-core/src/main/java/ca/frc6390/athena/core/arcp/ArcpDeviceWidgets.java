package ca.frc6390.athena.core.arcp;

import java.util.function.Consumer;

/**
 * Typed ARCP widget config builders for hardware-centric widgets.
 *
 * <p>This class remains as a compatibility façade. Concrete widget builders
 * are now split into {@code ca.frc6390.athena.core.arcp.widgets.*} files.</p>
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

    public static ArcpDashboardLayout.Widget motor(
            String topicPath,
            Consumer<MotorWidgetBuilder> section) {
        MotorWidgetBuilder builder = new MotorWidgetBuilder(0);
        builder.topicPath(topicPath);
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

    public static ArcpDashboardLayout.Widget encoder(
            String topicPath,
            Consumer<EncoderWidgetBuilder> section) {
        EncoderWidgetBuilder builder = new EncoderWidgetBuilder(0);
        builder.topicPath(topicPath);
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

    public static ArcpDashboardLayout.Widget imu(
            String topicPath,
            Consumer<ImuWidgetBuilder> section) {
        ImuWidgetBuilder builder = new ImuWidgetBuilder(0);
        builder.topicPath(topicPath);
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

    public static ArcpDashboardLayout.Widget dio(
            String topicPath,
            Consumer<DioWidgetBuilder> section) {
        DioWidgetBuilder builder = new DioWidgetBuilder(0);
        builder.topicPath(topicPath);
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

    public static ArcpDashboardLayout.Widget vision(
            String topicPath,
            Consumer<VisionWidgetBuilder> section) {
        VisionWidgetBuilder builder = new VisionWidgetBuilder(0);
        builder.topicPath(topicPath);
        if (section != null) {
            section.accept(builder);
        }
        return builder.build();
    }

    public static final class MotorWidgetBuilder
            extends ca.frc6390.athena.core.arcp.widgets.MotorWidgetBuilder {
        public MotorWidgetBuilder(int signalId) {
            super(signalId);
        }
    }

    public static final class EncoderWidgetBuilder
            extends ca.frc6390.athena.core.arcp.widgets.EncoderWidgetBuilder {
        public EncoderWidgetBuilder(int signalId) {
            super(signalId);
        }
    }

    public static final class ImuWidgetBuilder
            extends ca.frc6390.athena.core.arcp.widgets.ImuWidgetBuilder {
        public ImuWidgetBuilder(int signalId) {
            super(signalId);
        }
    }

    public static final class DioWidgetBuilder
            extends ca.frc6390.athena.core.arcp.widgets.DioWidgetBuilder {
        public DioWidgetBuilder(int signalId) {
            super(signalId);
        }
    }

    public static final class VisionWidgetBuilder
            extends ca.frc6390.athena.core.arcp.widgets.VisionWidgetBuilder {
        public VisionWidgetBuilder(int signalId) {
            super(signalId);
        }
    }
}
