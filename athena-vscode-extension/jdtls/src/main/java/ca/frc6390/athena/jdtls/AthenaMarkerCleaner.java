package ca.frc6390.athena.jdtls;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.List;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IMarker;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.jdt.core.ICompilationUnit;
import org.eclipse.jdt.core.IJavaModelMarker;
import org.eclipse.jdt.core.JavaCore;
import org.eclipse.jdt.core.dom.CompilationUnit;
import org.eclipse.core.runtime.jobs.Job;

final class AthenaMarkerCleaner {
    private static final Path LOG_PATH = Path.of(System.getProperty("java.io.tmpdir"), "athena-jdtls.log");

    private AthenaMarkerCleaner() {}

    static void sweepWorkspace() {
        try {
            ResourcesPlugin.getWorkspace().getRoot().accept(resource -> {
                if (resource instanceof IFile file && "java".equals(file.getFileExtension())) {
                    clearFalseAthenaStateMarkers(file);
                }
                return true;
            });
        } catch (CoreException exception) {
            log("workspaceSweep failed: " + exception);
        }
    }

    static void clearFalseAthenaStateMarkers(ICompilationUnit unit) {
        if (unit == null || !(unit.getResource() instanceof IFile file)) {
            return;
        }
        clearFalseAthenaStateMarkers(file);
    }

    static void scheduleCleanup(ICompilationUnit unit, long delayMillis) {
        if (unit == null || !(unit.getResource() instanceof IFile file)) {
            return;
        }
        Job job = new Job("Athena marker cleanup") {
            @Override
            protected IStatus run(IProgressMonitor monitor) {
                clearFalseAthenaStateMarkers(file);
                return Status.OK_STATUS;
            }
        };
        job.setSystem(true);
        job.schedule(delayMillis);
    }

    static void clearFalseAthenaStateMarkers(IFile file) {
        try {
            ICompilationUnit unit = JavaCore.createCompilationUnitFrom(file);
            if (unit == null) {
                return;
            }

            String source = AthenaStateModel.safeSource(unit);
            if (!AthenaStateModel.mightContainAthenaState(source)) {
                return;
            }

            CompilationUnit ast = AthenaStateModel.parse(unit);
            List<AthenaStateModel.SourceRange> constructorSites = AthenaStateModel.findConstructorSites(ast);
            if (constructorSites.isEmpty()) {
                return;
            }

            IMarker[] markers = file.findMarkers(IJavaModelMarker.JAVA_MODEL_PROBLEM_MARKER, true, IResource.DEPTH_ZERO);
            int removed = 0;
            for (IMarker marker : markers) {
                int id = marker.getAttribute(IJavaModelMarker.ID, -1);
                int start = marker.getAttribute(IMarker.CHAR_START, -1);
                int endExclusive = marker.getAttribute(IMarker.CHAR_END, -1);
                int end = Math.max(start, endExclusive - 1);
                if (!AthenaStateModel.shouldSuppress(id, start, end, constructorSites)) {
                    continue;
                }
                marker.delete();
                removed++;
            }

            if (removed > 0) {
                log("markerSweep " + file.getName() + " removed=" + removed);
            }
        } catch (CoreException exception) {
            log("markerSweep failed for " + file.getName() + ": " + exception);
        }
    }

    private static void log(String message) {
        try {
            Files.writeString(
                    LOG_PATH,
                    Instant.now() + " " + message + System.lineSeparator(),
                    StandardOpenOption.CREATE,
                    StandardOpenOption.APPEND);
        } catch (Exception ignored) {
            // no-op
        }
    }
}
