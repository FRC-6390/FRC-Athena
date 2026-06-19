package ca.frc6390.athena.api.mechanism.behavior.control;

import java.util.List;
import java.util.OptionalDouble;

import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.definition.LoopActivation;
import ca.frc6390.athena.api.mechanism.definition.LoopDeclarationKind;
import ca.frc6390.athena.api.mechanism.definition.MechanismLoopDefinition;
import ca.frc6390.athena.api.mechanism.definition.MechanismPidControllerDefinition;
import ca.frc6390.athena.mechanisms.MechanismInputSource;
import ca.frc6390.athena.mechanisms.MechanismSetpointSource;
import ca.frc6390.athena.mechanisms.OutputType;

public final class MechanismPid {
    private String name;
    private OutputType output = OutputType.PERCENT;
    private LoopMode mode = LoopMode.ENABLED;
    private List<String> states = List.of();
    private double kP;
    private double kI;
    private double kD;
    private Double tolerance;
    private Double maxVelocity;
    private Double maxAcceleration;
    private MechanismInputSource inputSource = MechanismInputSource.Position;
    private MechanismSetpointSource setpointSource = MechanismSetpointSource.Setpoint;

    private MechanismPid() {
    }

    public static MechanismPid create() {
        return new MechanismPid();
    }

    public static MechanismPid create(String name) {
        return create().named(name);
    }

    public static MechanismPid from(MechanismLoopDefinition definition) {
        if (!(definition.controller() instanceof MechanismPidControllerDefinition controller)) {
            throw new IllegalArgumentException("loop is not a PID declaration: " + definition.name());
        }
        MechanismPid pid = create(definition.name())
            .output(definition.output())
            .schedule(definition.activation().mode(), definition.activation().states().toArray(String[]::new))
            .kp(controller.kP())
            .ki(controller.kI())
            .kd(controller.kD())
            .inputSource(controller.inputSource())
            .setpointSource(controller.setpointSource());
        controller.tolerance().ifPresent(pid::tolerance);
        if (controller.maxVelocity().isPresent() || controller.maxAcceleration().isPresent()) {
            pid.constraints(
                controller.maxVelocity().orElse(Double.NaN),
                controller.maxAcceleration().orElse(Double.NaN));
        }
        return pid;
    }

    public MechanismPid named(String name) {
        this.name = name;
        return this;
    }

    public MechanismPid output(OutputType output) {
        this.output = output;
        return this;
    }

    public MechanismPid schedule(LoopMode mode, String... states) {
        this.mode = mode;
        this.states = states == null ? List.of() : List.of(states);
        return this;
    }

    public MechanismPid manual() {
        return schedule(LoopMode.MANUAL);
    }

    public MechanismPid enabled(String... states) {
        return schedule(LoopMode.ENABLED, states);
    }

    public MechanismPid kp(double kP) {
        this.kP = kP;
        return this;
    }

    public MechanismPid ki(double kI) {
        this.kI = kI;
        return this;
    }

    public MechanismPid kd(double kD) {
        this.kD = kD;
        return this;
    }

    public MechanismPid tolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public MechanismPid constraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        return this;
    }

    public MechanismPid profiled(double maxVelocity, double maxAcceleration) {
        return constraints(maxVelocity, maxAcceleration);
    }

    public MechanismPid inputSource(MechanismInputSource inputSource) {
        this.inputSource = inputSource != null ? inputSource : MechanismInputSource.Position;
        return this;
    }

    public MechanismPid inputInput(String key) {
        return inputSource(MechanismInputSource.input(key));
    }

    public MechanismPid setpointSource(MechanismSetpointSource setpointSource) {
        this.setpointSource = setpointSource != null ? setpointSource : MechanismSetpointSource.Setpoint;
        return this;
    }

    public MechanismPid setpointInput(String key) {
        return setpointSource(MechanismSetpointSource.input(key));
    }

    public MechanismLoopDefinition definition() {
        boolean hasProfiledVelocity = maxVelocity != null && Double.isFinite(maxVelocity) && maxVelocity > 0.0;
        boolean hasProfiledAcceleration = maxAcceleration != null && Double.isFinite(maxAcceleration) && maxAcceleration > 0.0;
        if (hasProfiledVelocity != hasProfiledAcceleration) {
            throw new IllegalArgumentException(
                "PID constraints require both maxVelocity and maxAcceleration (both > 0)");
        }
        if ((maxVelocity != null && Double.isFinite(maxVelocity) && maxVelocity <= 0.0)
                || (maxAcceleration != null && Double.isFinite(maxAcceleration) && maxAcceleration <= 0.0)) {
            throw new IllegalArgumentException("PID constraints must be > 0 when provided");
        }
        return new MechanismLoopDefinition(
            requiredName(),
            output,
            new LoopActivation(mode, states),
            LoopDeclarationKind.OBJECT,
            new MechanismPidControllerDefinition(
                kP,
                kI,
                kD,
                tolerance != null ? OptionalDouble.of(tolerance) : OptionalDouble.empty(),
                maxVelocity != null ? OptionalDouble.of(maxVelocity) : OptionalDouble.empty(),
                maxAcceleration != null ? OptionalDouble.of(maxAcceleration) : OptionalDouble.empty(),
                inputSource,
                setpointSource),
            getClass());
    }

    private String requiredName() {
        if (name == null || name.isBlank()) {
            throw new IllegalStateException("PID loop name is required");
        }
        return name;
    }
}
