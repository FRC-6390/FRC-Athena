package ca.frc6390.athena.api.mechanism.behavior.control;

import java.util.List;
import java.util.OptionalDouble;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.definition.LoopActivation;
import ca.frc6390.athena.api.mechanism.definition.LoopDeclarationKind;
import ca.frc6390.athena.api.mechanism.definition.MechanismBangBangControllerDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.mechanisms.MechanismInputSource;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;
import ca.frc6390.athena.mechanisms.OutputType;

public final class MechanismBangBang {
    private String name;
    private OutputType output = OutputType.PERCENT;
    private LoopMode mode = LoopMode.ENABLED;
    private List<String> states = List.of();
    private double highOutput = 1.0;
    private double lowOutput = -1.0;
    private Double tolerance;
    private MechanismInputSource inputSource = MechanismInputSource.Position;
    private MechanismSetpointSource setpointSource = MechanismSetpointSource.Setpoint;

    private MechanismBangBang() {
    }

    public static MechanismBangBang create() {
        return new MechanismBangBang();
    }

    public static MechanismBangBang create(String name) {
        return create().named(name);
    }

    public static MechanismBangBang from(MechanismLoopDefinition definition) {
        if (!(definition.controller() instanceof MechanismBangBangControllerDefinition controller)) {
            throw new IllegalArgumentException("loop is not a bang-bang declaration: " + definition.name());
        }
        MechanismBangBang bangBang = create(definition.name())
            .output(definition.output())
            .schedule(definition.activation().mode(), definition.activation().states().toArray(String[]::new))
            .high(controller.highOutput())
            .low(controller.lowOutput())
            .inputSource(controller.inputSource())
            .setpointSource(controller.setpointSource());
        controller.tolerance().ifPresent(bangBang::tolerance);
        return bangBang;
    }

    public MechanismBangBang named(String name) {
        this.name = name;
        return this;
    }

    public MechanismBangBang output(OutputType output) {
        this.output = output;
        return this;
    }

    public MechanismBangBang schedule(LoopMode mode, String... states) {
        this.mode = mode;
        this.states = states == null ? List.of() : List.of(states);
        return this;
    }

    public MechanismBangBang manual() {
        return schedule(LoopMode.MANUAL);
    }

    public MechanismBangBang enabled(String... states) {
        return schedule(LoopMode.ENABLED, states);
    }

    public MechanismBangBang high(double highOutput) {
        this.highOutput = highOutput;
        return this;
    }

    public MechanismBangBang low(double lowOutput) {
        this.lowOutput = lowOutput;
        return this;
    }

    public MechanismBangBang tolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public MechanismBangBang inputSource(MechanismInputSource inputSource) {
        this.inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
        return this;
    }

    public MechanismBangBang inputInput(String key) {
        return inputSource(MechanismInputSource.input(key));
    }

    public MechanismBangBang setpointSource(MechanismSetpointSource setpointSource) {
        this.setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        return this;
    }

    public MechanismBangBang setpointInput(String key) {
        return setpointSource(MechanismSetpointSource.input(key));
    }

    public MechanismLoopDefinition definition() {
        return new MechanismLoopDefinition(
            requiredName(),
            output,
            new LoopActivation(mode, states),
            LoopDeclarationKind.OBJECT,
            new MechanismBangBangControllerDefinition(
                highOutput,
                lowOutput,
                tolerance != null ? OptionalDouble.of(tolerance) : OptionalDouble.empty(),
                inputSource,
                setpointSource),
            getClass());
    }

    private String requiredName() {
        if (name == null || name.isBlank()) {
            throw new IllegalStateException("bang-bang loop name is required");
        }
        return name;
    }
}
