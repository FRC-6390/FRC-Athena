package ca.frc6390.athena.plugin.statespec;

import com.sun.source.util.JavacTask;
import com.sun.source.util.Plugin;
import com.sun.source.util.TaskEvent;
import com.sun.source.util.TaskListener;
import com.sun.tools.javac.api.BasicJavacTask;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.tree.TreeCopier;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.tree.TreeTranslator;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Names;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Javac plugin that injects enum DSL boilerplate for Athena plugin-enabled enums.
 */
public final class AthenaStateDslJavacPlugin implements Plugin {
    private static final String PLUGIN_NAME = "AthenaStateDsl";

    private static final String FQ_ATHENA_STATE = "ca.frc6390.athena.mechanisms.statespec.AthenaState";
    private static final String FQ_ATHENA_STATE_LOGIC = "ca.frc6390.athena.mechanisms.statespec.AthenaStateLogic";
    private static final String FQ_LEGACY_GENERATE_STATE_SPEC = "ca.frc6390.athena.mechanisms.statespec.GenerateStateSpec";
    private static final String FQ_LEGACY_STATE_LOGIC = "ca.frc6390.athena.mechanisms.statespec.StateLogic";
    private static final String FQ_STATE_BUILDER = "ca.frc6390.athena.mechanisms.statespec.StateBuilder";
    private static final String FQ_STATE_DSL = "ca.frc6390.athena.mechanisms.statespec.StateDsl";
    private static final String FQ_STATE_SEED = "ca.frc6390.athena.mechanisms.statespec.StateSeed";
    private static final String FQ_STATE_SEED_PROVIDER = "ca.frc6390.athena.mechanisms.statespec.StateSeedProvider";
    private static final String FQ_SETPOINT_PROVIDER = "ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider";

    private static final String SEED_METHOD = "seed";
    private static final String GET_SETPOINT_METHOD = "getSetpoint";

    @Override
    public String getName() {
        return PLUGIN_NAME;
    }

    @Override
    public void init(JavacTask task, String... args) {
        if (!(task instanceof BasicJavacTask basicTask)) {
            return;
        }

        Context context = basicTask.getContext();
        TreeMaker maker = TreeMaker.instance(context);
        Names names = Names.instance(context);

        task.addTaskListener(new TaskListener() {
            @Override
            public void started(TaskEvent e) {
                // no-op
            }

            @Override
            public void finished(TaskEvent e) {
                if (e.getKind() != TaskEvent.Kind.PARSE) {
                    return;
                }
                if (!(e.getCompilationUnit() instanceof JCTree.JCCompilationUnit unit)) {
                    return;
                }
                unit.accept(new StateEnumTranslator(maker, names));
            }
        });
    }

    private static final class StateEnumTranslator extends TreeTranslator {
        private final TreeMaker maker;
        private final Names names;
        private final TreeCopier<Void> copier;

        private StateEnumTranslator(TreeMaker maker, Names names) {
            this.maker = maker;
            this.names = names;
            this.copier = new TreeCopier<>(maker);
        }

        @Override
        public void visitClassDef(JCTree.JCClassDecl classDecl) {
            if (isGeneratedStateEnum(classDecl)) {
                String setpointTypeName = resolveSetpointTypeName(classDecl);
                Set<String> enumConstants = enumConstantNames(classDecl);
                Map<String, List<JCTree.JCExpression>> constantArgs = stripEnumConstantArgs(classDecl);
                rewriteStateLogicAnnotations(classDecl, enumConstants);
                if (setpointTypeName != null) {
                    ensureSetpointProviderInterface(classDecl, setpointTypeName);
                    ensureGetSetpointMethod(classDecl, setpointTypeName, constantArgs);
                }
                ensureStateSeedProviderInterface(classDecl);
                ensureStateSeedMethod(classDecl, constantArgs);
            }
            super.visitClassDef(classDecl);
        }

        private boolean isGeneratedStateEnum(JCTree.JCClassDecl classDecl) {
            if ((classDecl.mods.flags & Flags.ENUM) == 0) {
                return false;
            }
            return hasAnnotation(classDecl.mods.annotations, "AthenaState", FQ_ATHENA_STATE)
                    || hasAnnotation(classDecl.mods.annotations, "GenerateStateSpec", FQ_LEGACY_GENERATE_STATE_SPEC);
        }

        private String resolveSetpointTypeName(JCTree.JCClassDecl classDecl) {
            for (JCTree.JCAnnotation annotation : classDecl.mods.annotations) {
                if (matchesAnnotation(annotation, "AthenaState", FQ_ATHENA_STATE)
                        || matchesAnnotation(annotation, "GenerateStateSpec", FQ_LEGACY_GENERATE_STATE_SPEC)) {
                    String typeName = annotationSetpointTypeName(annotation);
                    if (typeName != null && !typeName.equals("java.lang.Void") && !typeName.equals("Void")) {
                        return typeName;
                    }
                }
            }
            return null;
        }

        private String annotationSetpointTypeName(JCTree.JCAnnotation annotation) {
            if (annotation.args == null || annotation.args.isEmpty()) {
                return null;
            }
            JCTree.JCExpression first = annotation.args.head;
            return extractClassLiteralTypeName(first);
        }

        private String extractClassLiteralTypeName(JCTree.JCExpression expression) {
            if (expression instanceof JCTree.JCAssign assign) {
                return extractClassLiteralTypeName(assign.rhs);
            }
            if (expression instanceof JCTree.JCFieldAccess fieldAccess
                    && fieldAccess.name.contentEquals("class")
                    && fieldAccess.selected != null) {
                return fieldAccess.selected.toString();
            }
            return null;
        }

        private Set<String> enumConstantNames(JCTree.JCClassDecl classDecl) {
            Set<String> constants = new HashSet<>();
            for (JCTree def : classDecl.defs) {
                if (!(def instanceof JCTree.JCVariableDecl var)) {
                    continue;
                }
                if ((var.mods.flags & Flags.ENUM) == 0) {
                    continue;
                }
                constants.add(var.getName().toString());
            }
            return constants;
        }

        private Map<String, List<JCTree.JCExpression>> stripEnumConstantArgs(JCTree.JCClassDecl classDecl) {
            Map<String, List<JCTree.JCExpression>> argsByConstant = new LinkedHashMap<>();
            for (JCTree def : classDecl.defs) {
                if (!(def instanceof JCTree.JCVariableDecl var)) {
                    continue;
                }
                if ((var.mods.flags & Flags.ENUM) == 0) {
                    continue;
                }
                if (!(var.init instanceof JCTree.JCNewClass init)) {
                    continue;
                }

                argsByConstant.put(var.getName().toString(), init.args);
                if (init.args != null && !init.args.isEmpty()) {
                    argsByConstant.put(var.getName().toString(), copyExpressions(init.args));
                    init.args = List.nil();
                } else {
                    argsByConstant.put(var.getName().toString(), List.nil());
                }
            }
            return argsByConstant;
        }

        private void rewriteStateLogicAnnotations(
                JCTree.JCClassDecl classDecl,
                Set<String> enumConstants) {
            ListBuffer<JCTree> rewritten = new ListBuffer<>();
            for (JCTree def : classDecl.defs) {
                if (def instanceof JCTree.JCMethodDecl method) {
                    rewritten.add(rewriteMethodStateLogic(method, enumConstants));
                } else {
                    rewritten.add(def);
                }
            }
            classDecl.defs = rewritten.toList();
        }

        private JCTree.JCMethodDecl rewriteMethodStateLogic(
                JCTree.JCMethodDecl method,
                Set<String> enumConstants) {
            if (method.mods == null || method.mods.annotations == null || method.mods.annotations.isEmpty()) {
                return method;
            }

            ListBuffer<JCTree.JCAnnotation> annotations = new ListBuffer<>();
            for (JCTree.JCAnnotation annotation : method.mods.annotations) {
                if (!matchesAnnotation(annotation, "AthenaStateLogic", FQ_ATHENA_STATE_LOGIC)
                        && !matchesAnnotation(annotation, "StateLogic", FQ_LEGACY_STATE_LOGIC)) {
                    annotations.add(annotation);
                    continue;
                }

                ListBuffer<JCTree.JCExpression> args = new ListBuffer<>();
                for (JCTree.JCExpression arg : annotation.args) {
                    args.add(rewriteStateLogicArgument(arg, enumConstants));
                }
                annotation.args = args.toList();
                annotations.add(annotation);
            }
            method.mods.annotations = annotations.toList();
            return method;
        }

        private JCTree.JCExpression rewriteStateLogicArgument(
                JCTree.JCExpression arg,
                Set<String> enumConstants) {
            if (arg instanceof JCTree.JCAssign assign) {
                String stateName = extractEnumName(assign.rhs);
                if (stateName == null || (!enumConstants.isEmpty() && !enumConstants.contains(stateName))) {
                    return arg;
                }
                return maker.Assign(assign.lhs, maker.Literal(stateName));
            }

            String stateName = extractEnumName(arg);
            if (stateName == null || (!enumConstants.isEmpty() && !enumConstants.contains(stateName))) {
                return arg;
            }
            return maker.Literal(stateName);
        }

        private String extractEnumName(JCTree.JCExpression expr) {
            if (expr instanceof JCTree.JCLiteral literal) {
                Object value = literal.getValue();
                return value instanceof String ? (String) value : null;
            }
            if (expr instanceof JCTree.JCIdent ident) {
                return ident.name.toString();
            }
            if (expr instanceof JCTree.JCFieldAccess fieldAccess) {
                return fieldAccess.name.toString();
            }
            return null;
        }

        private void ensureStateSeedProviderInterface(JCTree.JCClassDecl classDecl) {
            if (alreadyImplements(classDecl, "StateSeedProvider", FQ_STATE_SEED_PROVIDER)) {
                return;
            }
            JCTree.JCExpression providerType = maker.TypeApply(
                    qualIdent(FQ_STATE_SEED_PROVIDER),
                    List.of(maker.Ident(classDecl.name)));
            classDecl.implementing = classDecl.implementing.append(providerType);
        }

        private void ensureSetpointProviderInterface(JCTree.JCClassDecl classDecl, String setpointTypeName) {
            if (alreadyImplements(classDecl, "SetpointProvider", FQ_SETPOINT_PROVIDER)) {
                return;
            }
            JCTree.JCExpression providerType = maker.TypeApply(
                    qualIdent(FQ_SETPOINT_PROVIDER),
                    List.of(typeExpression(setpointTypeName)));
            classDecl.implementing = classDecl.implementing.append(providerType);
        }

        private void ensureGetSetpointMethod(
                JCTree.JCClassDecl classDecl,
                String setpointTypeName,
                Map<String, List<JCTree.JCExpression>> constantArgs) {
            if (hasMethod(classDecl, GET_SETPOINT_METHOD)) {
                return;
            }

            ListBuffer<JCTree.JCStatement> statements = new ListBuffer<>();
            for (JCTree def : classDecl.defs) {
                if (!(def instanceof JCTree.JCVariableDecl var)) {
                    continue;
                }
                if ((var.mods.flags & Flags.ENUM) == 0) {
                    continue;
                }

                JCTree.JCExpression setpointExpr = setpointExpression(
                        classDecl.name.toString(),
                        setpointTypeName,
                        copyExpressions(constantArgs.get(var.getName().toString())));
                JCTree.JCExpression cond = maker.Binary(
                        JCTree.Tag.EQ,
                        maker.Ident(names._this),
                        maker.Ident(var.name));
                statements.add(maker.If(
                        cond,
                        maker.Block(0, List.of(maker.Return(setpointExpr))),
                        null));
            }
            statements.add(maker.Return(nullLiteral()));

            JCTree.JCMethodDecl method = maker.MethodDef(
                    maker.Modifiers(Flags.PUBLIC),
                    names.fromString(GET_SETPOINT_METHOD),
                    typeExpression(setpointTypeName),
                    List.nil(),
                    List.nil(),
                    List.nil(),
                    maker.Block(0, statements.toList()),
                    null);

            classDecl.defs = classDecl.defs.append(method);
        }

        private void ensureStateSeedMethod(
                JCTree.JCClassDecl classDecl,
                Map<String, List<JCTree.JCExpression>> constantArgs) {
            if (hasMethod(classDecl, SEED_METHOD)) {
                return;
            }

            ListBuffer<JCTree.JCStatement> statements = new ListBuffer<>();
            for (JCTree def : classDecl.defs) {
                if (!(def instanceof JCTree.JCVariableDecl var)) {
                    continue;
                }
                if ((var.mods.flags & Flags.ENUM) == 0) {
                    continue;
                }

                JCTree.JCExpression seedExpr = seedExpression(copyExpressions(constantArgs.get(var.getName().toString())));
                JCTree.JCExpression cond = maker.Binary(
                        JCTree.Tag.EQ,
                        maker.Ident(names._this),
                        maker.Ident(var.name));
                statements.add(maker.If(
                        cond,
                        maker.Block(0, List.of(maker.Return(seedExpr))),
                        null));
            }
            statements.add(maker.Return(autoSeedCall()));

            JCTree.JCMethodDecl method = maker.MethodDef(
                    maker.Modifiers(Flags.PUBLIC),
                    names.fromString(SEED_METHOD),
                    maker.TypeApply(qualIdent(FQ_STATE_SEED), List.of(maker.Ident(classDecl.name))),
                    List.nil(),
                    List.nil(),
                    List.nil(),
                    maker.Block(0, statements.toList()),
                    null);

            classDecl.defs = classDecl.defs.append(method);
        }

        private JCTree.JCExpression seedExpression(List<JCTree.JCExpression> args) {
            if (args == null || args.isEmpty()) {
                return autoSeedCall();
            }
            if (args.size() != 1) {
                return autoSeedCall();
            }
            JCTree.JCExpression first = args.head;
            if (isNumeric(first)) {
                return setpointSeedCall(first);
            }
            if (isDslExpression(first)) {
                return dslSeedCall(first);
            }
            return setpointSeedCall(first);
        }

        private JCTree.JCExpression setpointExpression(
                String enumTypeName,
                String setpointTypeName,
                List<JCTree.JCExpression> args) {
            if (args == null || args.isEmpty()) {
                return nullLiteral();
            }
            if (args.size() == 1) {
                JCTree.JCExpression first = args.head;
                if (isDoubleSetpointType(setpointTypeName) && isDslExpression(first)) {
                    return dslDoubleSetpointExpression(enumTypeName, first);
                }
                return first;
            }
            return maker.NewClass(
                    null,
                    List.nil(),
                    typeExpression(setpointTypeName),
                    args,
                    null);
        }

        private JCTree.JCExpression dslDoubleSetpointExpression(String enumTypeName, JCTree.JCExpression dslExpr) {
            JCTree.JCExpression enumType = typeExpression(enumTypeName);
            JCTree.JCExpression stateDslType = maker.TypeApply(qualIdent(FQ_STATE_DSL), List.of(enumType));
            JCTree.JCExpression castDsl = maker.TypeCast(stateDslType, dslExpr);
            JCTree.JCExpression builderType = maker.TypeApply(qualIdent(FQ_STATE_BUILDER), List.of(enumType));
            JCTree.JCExpression builder = maker.NewClass(null, List.nil(), builderType, List.nil(), null);
            JCTree.JCExpression apply = maker.Apply(
                    List.nil(),
                    maker.Select(castDsl, names.fromString("apply")),
                    List.of(builder));
            return maker.Apply(
                    List.nil(),
                    maker.Select(apply, names.fromString("setpoint")),
                    List.nil());
        }

        private boolean isDoubleSetpointType(String setpointTypeName) {
            return "Double".equals(setpointTypeName) || "java.lang.Double".equals(setpointTypeName);
        }

        private boolean isDslExpression(JCTree.JCExpression expr) {
            return expr instanceof JCTree.JCLambda || expr instanceof JCTree.JCMemberReference;
        }

        private JCTree.JCExpression nullLiteral() {
            return maker.Literal(TypeTag.BOT, null);
        }

        private boolean isNumeric(JCTree.JCExpression expr) {
            if (expr instanceof JCTree.JCLiteral literal) {
                return literal.getValue() instanceof Number;
            }
            if (expr instanceof JCTree.JCUnary unary) {
                return unary.getTag() == JCTree.Tag.NEG
                        && unary.arg instanceof JCTree.JCLiteral lit
                        && lit.getValue() instanceof Number;
            }
            return false;
        }

        private JCTree.JCExpression autoSeedCall() {
            return maker.Apply(
                    List.nil(),
                    maker.Select(qualIdent(FQ_STATE_SEED), names.fromString("auto")),
                    List.nil());
        }

        private JCTree.JCExpression setpointSeedCall(JCTree.JCExpression value) {
            return maker.Apply(
                    List.nil(),
                    maker.Select(qualIdent(FQ_STATE_SEED), names.fromString("setpoint")),
                    List.of(value));
        }

        private JCTree.JCExpression dslSeedCall(JCTree.JCExpression dsl) {
            return maker.Apply(
                    List.nil(),
                    maker.Select(qualIdent(FQ_STATE_SEED), names.fromString("dsl")),
                    List.of(dsl));
        }

        private JCTree.JCExpression typeExpression(String typeName) {
            String[] parts = typeName.split("\\.");
            JCTree.JCExpression expr = maker.Ident(names.fromString(parts[0]));
            for (int i = 1; i < parts.length; i++) {
                expr = maker.Select(expr, names.fromString(parts[i]));
            }
            return expr;
        }

        private JCTree.JCExpression qualIdent(String fqcn) {
            String[] parts = fqcn.split("\\.");
            JCTree.JCExpression expr = maker.Ident(names.fromString(parts[0]));
            for (int i = 1; i < parts.length; i++) {
                expr = maker.Select(expr, names.fromString(parts[i]));
            }
            return expr;
        }

        private List<JCTree.JCExpression> copyExpressions(List<JCTree.JCExpression> source) {
            if (source == null || source.isEmpty()) {
                return List.nil();
            }
            ListBuffer<JCTree.JCExpression> copied = new ListBuffer<>();
            for (JCTree.JCExpression expression : source) {
                copied.add((JCTree.JCExpression) copier.copy(expression));
            }
            return copied.toList();
        }

        private boolean hasMethod(JCTree.JCClassDecl classDecl, String methodName) {
            for (JCTree def : classDecl.defs) {
                if (def instanceof JCTree.JCMethodDecl method && method.name.contentEquals(methodName)) {
                    return true;
                }
            }
            return false;
        }

        private boolean alreadyImplements(JCTree.JCClassDecl classDecl, String simpleName, String fqcn) {
            for (JCTree.JCExpression impl : classDecl.implementing) {
                String text = impl.toString();
                if (text.equals(simpleName)
                        || text.equals(fqcn)
                        || text.endsWith('.' + simpleName)) {
                    return true;
                }
            }
            return false;
        }

        private boolean hasAnnotation(List<JCTree.JCAnnotation> annotations, String simpleName, String fqcn) {
            for (JCTree.JCAnnotation annotation : annotations) {
                if (matchesAnnotation(annotation, simpleName, fqcn)) {
                    return true;
                }
            }
            return false;
        }

        private boolean matchesAnnotation(JCTree.JCAnnotation annotation, String simpleName, String fqcn) {
            if (annotation == null || annotation.annotationType == null) {
                return false;
            }
            String text = annotation.annotationType.toString();
            return text.equals(simpleName)
                    || text.equals(fqcn)
                    || text.endsWith('.' + simpleName);
        }
    }
}
