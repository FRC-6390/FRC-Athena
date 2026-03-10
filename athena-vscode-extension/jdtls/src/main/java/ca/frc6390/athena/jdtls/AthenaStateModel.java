package ca.frc6390.athena.jdtls;

import java.util.ArrayList;
import java.util.List;
import org.eclipse.jdt.core.ICompilationUnit;
import org.eclipse.jdt.core.JavaModelException;
import org.eclipse.jdt.core.compiler.IProblem;
import org.eclipse.jdt.core.dom.AST;
import org.eclipse.jdt.core.dom.ASTNode;
import org.eclipse.jdt.core.dom.ASTParser;
import org.eclipse.jdt.core.dom.ASTVisitor;
import org.eclipse.jdt.core.dom.Annotation;
import org.eclipse.jdt.core.dom.CompilationUnit;
import org.eclipse.jdt.core.dom.EnumConstantDeclaration;
import org.eclipse.jdt.core.dom.EnumDeclaration;
import org.eclipse.jdt.core.dom.Name;

final class AthenaStateModel {
    static final String ATHENA_STATE = "AthenaState";
    static final String ATHENA_STATE_FQ = "ca.frc6390.athena.mechanisms.statespec.AthenaState";
    static final String LEGACY_STATE = "GenerateStateSpec";
    static final String LEGACY_STATE_FQ = "ca.frc6390.athena.mechanisms.statespec.GenerateStateSpec";

    private AthenaStateModel() {}

    static boolean mightContainAthenaState(String source) {
        return source != null && (source.contains(ATHENA_STATE) || source.contains(LEGACY_STATE));
    }

    static CompilationUnit parse(ICompilationUnit unit) {
        ASTParser parser = ASTParser.newParser(AST.getJLSLatest());
        parser.setSource(unit);
        parser.setResolveBindings(false);
        return (CompilationUnit) parser.createAST(null);
    }

    static List<SourceRange> findConstructorSites(CompilationUnit ast) {
        List<SourceRange> ranges = new ArrayList<>();
        ast.accept(new ASTVisitor() {
            @Override
            public boolean visit(EnumDeclaration node) {
                if (!isAthenaStateEnum(node)) {
                    return false;
                }
                for (Object constantObject : node.enumConstants()) {
                    EnumConstantDeclaration constant = (EnumConstantDeclaration) constantObject;
                    if (constant.arguments().isEmpty()) {
                        continue;
                    }
                    ASTNode name = constant.getName();
                    ranges.add(new SourceRange(name.getStartPosition(), name.getStartPosition() + name.getLength() - 1));
                }
                return false;
            }
        });
        return ranges;
    }

    static boolean shouldSuppress(int problemId, int sourceStart, int sourceEnd, List<SourceRange> constructorSites) {
        if (problemId != IProblem.UndefinedConstructor) {
            return false;
        }
        for (SourceRange range : constructorSites) {
            if (range.contains(sourceStart, sourceEnd) || range.overlaps(sourceStart, sourceEnd)) {
                return true;
            }
        }
        return false;
    }

    static String safeSource(ICompilationUnit unit) throws JavaModelException {
        return unit == null ? null : unit.getSource();
    }

    private static boolean isAthenaStateEnum(EnumDeclaration declaration) {
        for (Object modifierObject : declaration.modifiers()) {
            if (!(modifierObject instanceof Annotation annotation)) {
                continue;
            }
            Name typeName = annotation.getTypeName();
            String qualified = typeName.getFullyQualifiedName();
            if (ATHENA_STATE.equals(qualified)
                    || ATHENA_STATE_FQ.equals(qualified)
                    || LEGACY_STATE.equals(qualified)
                    || LEGACY_STATE_FQ.equals(qualified)) {
                return true;
            }
        }
        return false;
    }

    record SourceRange(int start, int end) {
        boolean contains(int sourceStart, int sourceEnd) {
            return sourceStart >= start && sourceEnd <= end;
        }

        boolean overlaps(int sourceStart, int sourceEnd) {
            return sourceStart <= end && sourceEnd >= start;
        }
    }
}
