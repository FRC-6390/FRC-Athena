package ca.frc6390.athena.api.superstructure.behavior;

import java.util.List;

import ca.frc6390.athena.api.superstructure.behavior.constraint.ConstraintsConfigurer;
import ca.frc6390.athena.api.superstructure.behavior.constraint.SuperstructureConstraints;
import ca.frc6390.athena.api.superstructure.behavior.hook.HooksConfigurer;
import ca.frc6390.athena.api.superstructure.behavior.hook.SuperstructureHooks;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureConstraintDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureHookDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureTransitionHookDefinition;

public final class SuperstructureBehavior<S, SP> {
    private final SuperstructureConstraints<S, SP> constraints;
    private final SuperstructureHooks<S, SP> hooks;

    private SuperstructureBehavior() {
        this.constraints = SuperstructureConstraints.create();
        this.hooks = SuperstructureHooks.create();
    }

    public static <S, SP> SuperstructureBehavior<S, SP> create() {
        return new SuperstructureBehavior<>();
    }

    public static <S, SP> SuperstructureBehavior<S, SP> from(
            List<SuperstructureConstraintDefinition<SP>> constraints,
            List<SuperstructureHookDefinition<SP>> hooks,
            List<SuperstructureTransitionHookDefinition<SP>> transitions) {
        SuperstructureBehavior<S, SP> behavior = create();
        behavior.constraints.merge(SuperstructureConstraints.from(constraints));
        behavior.hooks.merge(SuperstructureHooks.from(hooks, transitions));
        return behavior;
    }

    public SuperstructureBehavior<S, SP> constraints(ConstraintsConfigurer<S, SP> configurer) {
        if (configurer != null) {
            configurer.apply(constraints);
        }
        return this;
    }

    public SuperstructureBehavior<S, SP> hooks(HooksConfigurer<S, SP> configurer) {
        if (configurer != null) {
            configurer.apply(hooks);
        }
        return this;
    }

    public SuperstructureBehavior<S, SP> merge(SuperstructureBehavior<S, SP> other) {
        if (other != null) {
            constraints.merge(other.constraints);
            hooks.merge(other.hooks);
        }
        return this;
    }

    public SuperstructureConstraints<S, SP> constraintsSection() {
        return constraints;
    }

    public SuperstructureHooks<S, SP> hooksSection() {
        return hooks;
    }

    public List<SuperstructureConstraintDefinition<SP>> constraintDefinitions() {
        return constraints.definitions();
    }

    public List<SuperstructureHookDefinition<SP>> hookDefinitions() {
        return hooks.hookDefinitions();
    }

    public List<SuperstructureTransitionHookDefinition<SP>> transitionDefinitions() {
        return hooks.transitionDefinitions();
    }
}
