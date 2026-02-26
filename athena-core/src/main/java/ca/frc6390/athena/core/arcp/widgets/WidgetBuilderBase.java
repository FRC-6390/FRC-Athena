package ca.frc6390.athena.core.arcp.widgets;

import ca.frc6390.athena.core.arcp.ArcpDashboardLayout;
import java.util.LinkedHashMap;
import java.util.Map;

public abstract class WidgetBuilderBase<Self extends WidgetBuilderBase<Self>> {
    protected final int signalId;
    protected final String kind;
    protected String id;
    protected String title;
    protected ArcpDashboardLayout.LayoutRect layout = new ArcpDashboardLayout.LayoutRect(0, 0, 8, 6);
    protected String parentLayoutId;
    protected final Map<String, Object> config = new LinkedHashMap<>();

    protected WidgetBuilderBase(int signalId, String kind, String defaultTitle) {
        this.signalId = signalId;
        this.kind = kind;
        this.title = defaultTitle;
    }

    @SuppressWarnings("unchecked")
    protected final Self self() {
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

    public Self parentLayoutId(String parentLayoutId) {
        if (parentLayoutId != null && !parentLayoutId.isBlank()) {
            this.parentLayoutId = parentLayoutId.trim();
        } else {
            this.parentLayoutId = null;
        }
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

    protected Self signalPathConfig(String key, String signalPath) {
        String normalized = normalizePath(signalPath);
        if (normalized.isEmpty()) {
            return self();
        }
        return config(key, normalized);
    }

    public Self topicPath(String topicPath) {
        String normalized = normalizePath(topicPath);
        if (normalized.isEmpty()) {
            return self();
        }
        return config("topicPath", normalized);
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
        if (parentLayoutId != null && !parentLayoutId.isBlank()) {
            widget.parentLayoutId(parentLayoutId);
        }
        return widget.build();
    }

    protected static String normalizePath(String rawPath) {
        if (rawPath == null) {
            return "";
        }
        String trimmed = rawPath.trim();
        if (trimmed.isEmpty()) {
            return "";
        }
        String slashNormalized = trimmed.replace('\\', '/');
        while (slashNormalized.startsWith("/")) {
            slashNormalized = slashNormalized.substring(1);
        }
        while (slashNormalized.endsWith("/")) {
            slashNormalized = slashNormalized.substring(0, slashNormalized.length() - 1);
        }
        return slashNormalized;
    }
}
