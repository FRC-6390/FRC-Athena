package ca.frc6390.athena.api.mechanism.behavior.control;

import java.util.List;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.definition.LoopActivation;
import ca.frc6390.athena.api.mechanism.definition.LoopDeclarationKind;
import ca.frc6390.athena.api.mechanism.definition.MechanismCustomControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.mechanisms.OutputType;

public final class MechanismControlLoop {
    private String name;
    private OutputType output = OutputType.PERCENT;
    private LoopMode mode = LoopMode.ENABLED;
    private List<String> states = List.of();
    private MechanismLoopCallback callback;

    private MechanismControlLoop() {
    }

    public static MechanismControlLoop create() {
        return new MechanismControlLoop();
    }

    public static MechanismControlLoop create(String name) {
        return create().named(name);
    }

    public static MechanismControlLoop from(MechanismLoopDefinition definition) {
        MechanismControlLoop loop = create(definition.name())
            .output(definition.output())
            .schedule(definition.activation().mode(), definition.activation().states().toArray(String[]::new));
        if (definition.controller() instanceof MechanismCustomControllerDefinition custom) {
            custom.callback().ifPresent(loop::custom);
        }
        return loop;
    }

    public MechanismControlLoop named(String name) {
        this.name = name;
        return this;
    }

    public MechanismControlLoop output(OutputType output) {
        this.output = output;
        return this;
    }

    public MechanismControlLoop custom(MechanismLoopCallback callback) {
        this.callback = callback;
        return this;
    }

    public MechanismControlLoop schedule(LoopMode mode, String... states) {
        this.mode = mode;
        this.states = states == null ? List.of() : List.of(states);
        return this;
    }

    public MechanismControlLoop manual() {
        return schedule(LoopMode.MANUAL);
    }

    public MechanismControlLoop enabled(String... states) {
        return schedule(LoopMode.ENABLED, states);
    }

    public MechanismLoopDefinition definition() {
        return new MechanismLoopDefinition(
            requiredName(),
            output,
            new LoopActivation(mode, states),
            LoopDeclarationKind.OBJECT,
            callback != null
                ? MechanismCustomControllerDefinition.of(callback)
                : MechanismCustomControllerDefinition.EMPTY,
            getClass());
    }

    private String requiredName() {
        if (name == null || name.isBlank()) {
            throw new IllegalStateException("custom loop name is required");
        }
        return name;
    }
}
