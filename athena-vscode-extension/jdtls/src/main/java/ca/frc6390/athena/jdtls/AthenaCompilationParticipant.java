package ca.frc6390.athena.jdtls;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import org.eclipse.jdt.core.ICompilationUnit;
import org.eclipse.jdt.core.IJavaModelMarker;
import org.eclipse.jdt.core.IJavaProject;
import org.eclipse.jdt.core.JavaModelException;
import org.eclipse.jdt.core.compiler.CategorizedProblem;
import org.eclipse.jdt.core.compiler.CompilationParticipant;
import org.eclipse.jdt.core.compiler.ReconcileContext;
import org.eclipse.jdt.core.dom.AST;
import org.eclipse.jdt.core.dom.CompilationUnit;

public final class AthenaCompilationParticipant extends CompilationParticipant {
    private static final Path LOG_PATH = Path.of(System.getProperty("java.io.tmpdir"), "athena-jdtls.log");

    @Override
    public boolean isActive(IJavaProject project) {
        try {
            return project.findType(AthenaStateModel.ATHENA_STATE_FQ) != null
                    || project.findType(AthenaStateModel.LEGACY_STATE_FQ) != null;
        } catch (JavaModelException ignored) {
            return false;
        }
    }

    @Override
    public void reconcile(ReconcileContext context) {
        try {
            ICompilationUnit unit = context.getWorkingCopy();
            if (unit == null) {
                return;
            }

            String source = AthenaStateModel.safeSource(unit);
            if (!AthenaStateModel.mightContainAthenaState(source)) {
                return;
            }

            CategorizedProblem[] problems = context.getProblems(IJavaModelMarker.JAVA_MODEL_PROBLEM_MARKER);
            if (problems == null || problems.length == 0) {
                return;
            }

            debug(unit, "incomingProblems=" + problems.length);
            for (CategorizedProblem problem : problems) {
                debug(
                        unit,
                        "problem id=" + problem.getID()
                                + " range=" + problem.getSourceStart() + ":" + problem.getSourceEnd()
                                + " msg=" + problem.getMessage());
            }

            CompilationUnit ast = context.getAST(AST.getJLSLatest());
            if (ast == null) {
                return;
            }

            List<AthenaStateModel.SourceRange> constructorSites = AthenaStateModel.findConstructorSites(ast);
            for (AthenaStateModel.SourceRange site : constructorSites) {
                debug(unit, "constructorSite=" + site.start() + ":" + site.end());
            }
            if (constructorSites.isEmpty()) {
                return;
            }

            AthenaMarkerCleaner.clearFalseAthenaStateMarkers(unit);
            AthenaMarkerCleaner.scheduleCleanup(unit, 250);

            List<CategorizedProblem> filtered = new ArrayList<>(problems.length);
            boolean changed = false;
            for (CategorizedProblem problem : problems) {
                if (shouldSuppress(problem, constructorSites)) {
                    changed = true;
                    continue;
                }
                filtered.add(problem);
            }

            if (changed) {
                context.putProblems(
                        IJavaModelMarker.JAVA_MODEL_PROBLEM_MARKER,
                        filtered.toArray(CategorizedProblem[]::new));
                debug(unit, "suppressed=" + (problems.length - filtered.size()));
            }
        } catch (JavaModelException ignored) {
            // Keep JDT stable if the working copy is unavailable mid-reconcile.
        }
    }

    @Override
    public void buildFinished(IJavaProject project) {
        AthenaMarkerCleaner.sweepWorkspace();
    }

    static boolean shouldSuppress(CategorizedProblem problem, List<AthenaStateModel.SourceRange> constructorSites) {
        if (problem == null) {
            return false;
        }
        return AthenaStateModel.shouldSuppress(
                problem.getID(), problem.getSourceStart(), problem.getSourceEnd(), constructorSites);
    }

    private static void debug(ICompilationUnit unit, String message) {
        try {
            Files.writeString(
                    LOG_PATH,
                    Instant.now() + " [Athena JDTLS] " + unit.getElementName() + " " + message + System.lineSeparator(),
                    StandardOpenOption.CREATE,
                    StandardOpenOption.APPEND);
        } catch (Exception ignored) {
            // no-op
        }
    }
}
