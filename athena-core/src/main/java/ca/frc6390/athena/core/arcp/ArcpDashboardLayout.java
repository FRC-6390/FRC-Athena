package ca.frc6390.athena.core.arcp;

import java.time.Instant;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * Builder/model for ARCP host-dashboard layout exports.
 */
public final class ArcpDashboardLayout {
    private static final String LAYOUT_FORMAT = "arcp.host.layout.export";
    private static final int LAYOUT_VERSION = 1;
    private static final ObjectMapper MAPPER = new ObjectMapper();

    private final String activeTabId;
    private final List<Page> pages;

    private ArcpDashboardLayout(String activeTabId, List<Page> pages) {
        this.activeTabId = sanitizeId(activeTabId, "tab");
        this.pages = List.copyOf(pages != null ? pages : List.of());
    }

    public static Builder builder() {
        return new Builder();
    }

    public String activeTabId() {
        return activeTabId;
    }

    public List<Page> pages() {
        return pages;
    }

    public String toJson() {
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("format", LAYOUT_FORMAT);
        root.put("version", LAYOUT_VERSION);
        root.put("exported_at", Instant.now().toString());

        Map<String, Object> layout = new LinkedHashMap<>();
        layout.put("activeTabId", activeTabId);

        List<Map<String, Object>> tabs = new ArrayList<>(pages.size());
        for (Page page : pages) {
            tabs.add(page.toMap());
        }
        layout.put("tabs", tabs);
        root.put("layout", layout);

        try {
            return MAPPER.writeValueAsString(root);
        } catch (JsonProcessingException ex) {
            throw new IllegalStateException("Failed to encode ARCP layout JSON", ex);
        }
    }

    public static final class Builder {
        private String activeTabId;
        private final List<Page> pages = new ArrayList<>();

        private Builder() {
        }

        public Builder activeTabId(String tabId) {
            this.activeTabId = tabId;
            return this;
        }

        public Builder page(Page page) {
            if (page != null) {
                pages.add(page);
            }
            return this;
        }

        public Builder page(Consumer<Page.Builder> section) {
            if (section == null) {
                return this;
            }
            Page.Builder builder = Page.builder();
            section.accept(builder);
            pages.add(builder.build());
            return this;
        }

        public ArcpDashboardLayout build() {
            List<Page> resolvedPages = pages.isEmpty() ? List.of(Page.builder().name("Dashboard").build()) : List.copyOf(pages);
            String resolvedActive = activeTabId;
            if (resolvedActive == null || resolvedActive.isBlank()) {
                resolvedActive = resolvedPages.get(0).id();
            }
            return new ArcpDashboardLayout(resolvedActive, resolvedPages);
        }
    }

    public record Page(String id, String name, List<Widget> widgets) {
        public Page {
            id = sanitizeId(id, "tab");
            name = sanitizeLabel(name, "Dashboard");
            widgets = List.copyOf(widgets != null ? widgets : List.of());
        }

        public static Builder builder() {
            return new Builder();
        }

        private Map<String, Object> toMap() {
            Map<String, Object> map = new LinkedHashMap<>();
            map.put("id", id);
            map.put("name", name);
            List<Map<String, Object>> widgetMaps = new ArrayList<>(widgets.size());
            for (Widget widget : widgets) {
                widgetMaps.add(widget.toMap());
            }
            map.put("widgets", widgetMaps);
            return map;
        }

        public static final class Builder {
            private String id;
            private String name = "Dashboard";
            private final List<Widget> widgets = new ArrayList<>();

            private Builder() {
            }

            public Builder id(String id) {
                this.id = id;
                return this;
            }

            public Builder name(String name) {
                this.name = name;
                return this;
            }

            public Builder widget(Widget widget) {
                if (widget != null) {
                    widgets.add(widget);
                }
                return this;
            }

            public Builder widget(Consumer<Widget.Builder> section) {
                if (section == null) {
                    return this;
                }
                Widget.Builder builder = Widget.builder();
                section.accept(builder);
                widgets.add(builder.build());
                return this;
            }

            public Builder motor(int signalId, Consumer<ArcpDeviceWidgets.MotorWidgetBuilder> section) {
                return widget(ArcpDeviceWidgets.motor(signalId, section));
            }

            public Builder encoder(int signalId, Consumer<ArcpDeviceWidgets.EncoderWidgetBuilder> section) {
                return widget(ArcpDeviceWidgets.encoder(signalId, section));
            }

            public Builder imu(int signalId, Consumer<ArcpDeviceWidgets.ImuWidgetBuilder> section) {
                return widget(ArcpDeviceWidgets.imu(signalId, section));
            }

            public Builder dio(int signalId, Consumer<ArcpDeviceWidgets.DioWidgetBuilder> section) {
                return widget(ArcpDeviceWidgets.dio(signalId, section));
            }

            public Builder vision(int signalId, Consumer<ArcpDeviceWidgets.VisionWidgetBuilder> section) {
                return widget(ArcpDeviceWidgets.vision(signalId, section));
            }

            public Page build() {
                String resolvedName = sanitizeLabel(name, "Dashboard");
                String resolvedId = sanitizeId(id, "tab-" + slug(resolvedName));
                return new Page(resolvedId, resolvedName, widgets);
            }
        }
    }

    public record LayoutRect(int x, int y, int w, int h) {
        public LayoutRect {
            w = Math.max(1, w);
            h = Math.max(1, h);
            x = Math.max(0, x);
            y = Math.max(0, y);
        }
    }

    public record Widget(
            String id,
            int signalId,
            String kind,
            String title,
            LayoutRect layout,
            String parentLayoutId,
            Map<String, Object> config) {
        public Widget {
            id = sanitizeId(id, "widget");
            kind = sanitizeLabel(kind, "metric");
            title = sanitizeLabel(title, "Widget");
            layout = layout != null ? layout : new LayoutRect(0, 0, 2, 1);
            parentLayoutId = normalizeOptional(parentLayoutId);
            config = config != null ? Map.copyOf(config) : Map.of();
        }

        public static Builder builder() {
            return new Builder();
        }

        private Map<String, Object> toMap() {
            Map<String, Object> map = new LinkedHashMap<>();
            map.put("id", id);
            map.put("signalId", signalId);
            map.put("kind", kind);
            map.put("title", title);
            // Host dashboard grid coordinates are 1-based.
            map.put("layout", Map.of(
                    "x", layout.x() + 1,
                    "y", layout.y() + 1,
                    "w", layout.w(),
                    "h", layout.h()));
            if (parentLayoutId != null) {
                map.put("parentLayoutId", parentLayoutId);
            }
            if (!config.isEmpty()) {
                map.put("config", config);
            }
            return map;
        }

        public static final class Builder {
            private String id;
            private int signalId;
            private String kind = "metric";
            private String title = "Widget";
            private LayoutRect layout = new LayoutRect(0, 0, 2, 1);
            private String parentLayoutId;
            private final Map<String, Object> config = new LinkedHashMap<>();

            private Builder() {
            }

            public Builder id(String id) {
                this.id = id;
                return this;
            }

            public Builder signalId(int signalId) {
                this.signalId = signalId;
                return this;
            }

            public Builder kind(String kind) {
                this.kind = kind;
                return this;
            }

            public Builder title(String title) {
                this.title = title;
                return this;
            }

            public Builder layout(int x, int y, int w, int h) {
                this.layout = new LayoutRect(x, y, w, h);
                return this;
            }

            public Builder parentLayoutId(String parentLayoutId) {
                this.parentLayoutId = parentLayoutId;
                return this;
            }

            public Builder config(String key, Object value) {
                if (key == null || key.isBlank()) {
                    return this;
                }
                config.put(key, value);
                return this;
            }

            public Builder config(Map<String, ?> entries) {
                if (entries == null || entries.isEmpty()) {
                    return this;
                }
                for (Map.Entry<String, ?> entry : entries.entrySet()) {
                    if (entry == null || entry.getKey() == null || entry.getKey().isBlank()) {
                        continue;
                    }
                    config.put(entry.getKey(), entry.getValue());
                }
                return this;
            }

            public Widget build() {
                String resolvedKind = sanitizeLabel(kind, "metric");
                String resolvedTitle = sanitizeLabel(title, "Widget");
                String fallbackId = "w-" + slug(resolvedTitle) + "-" + Math.max(0, signalId);
                String resolvedId = sanitizeId(id, fallbackId);
                return new Widget(
                        resolvedId,
                        signalId,
                        resolvedKind,
                        resolvedTitle,
                        layout,
                        parentLayoutId,
                        config);
            }
        }
    }

    private static String sanitizeId(String value, String fallback) {
        String normalized = normalizeOptional(value);
        return normalized != null ? normalized : fallback;
    }

    private static String sanitizeLabel(String value, String fallback) {
        String normalized = normalizeOptional(value);
        return normalized != null ? normalized : fallback;
    }

    private static String normalizeOptional(String value) {
        if (value == null) {
            return null;
        }
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private static String slug(String value) {
        Objects.requireNonNull(value, "value");
        String lower = value.trim().toLowerCase();
        if (lower.isEmpty()) {
            return "page";
        }
        StringBuilder out = new StringBuilder(lower.length());
        boolean lastDash = false;
        for (int i = 0; i < lower.length(); i++) {
            char ch = lower.charAt(i);
            boolean valid = (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9');
            if (valid) {
                out.append(ch);
                lastDash = false;
                continue;
            }
            if (!lastDash) {
                out.append('-');
                lastDash = true;
            }
        }
        String slug = out.toString();
        while (slug.startsWith("-")) {
            slug = slug.substring(1);
        }
        while (slug.endsWith("-")) {
            slug = slug.substring(0, slug.length() - 1);
        }
        return slug.isEmpty() ? "page" : slug;
    }
}
