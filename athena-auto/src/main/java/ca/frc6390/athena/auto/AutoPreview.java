package ca.frc6390.athena.auto;

import java.util.List;

/** Selected autonomous name, readable Action plan, and field geometry. */
public record AutoPreview(String name, List<String> steps, List<PathPreview> paths) {
    public AutoPreview {
        name = name == null || name.isBlank() ? "auto" : name.trim();
        steps = List.copyOf(steps);
        paths = List.copyOf(paths);
    }
}
