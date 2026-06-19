package ca.frc6390.athena.api.superstructure.input;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.api.superstructure.definition.SuperstructureBooleanInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureDoubleInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureIntInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureObjectInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructurePose2dInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructurePose3dInputDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureStringInputDefinition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public final class SuperstructureInputs {
    private final List<SuperstructureInputDefinition> inputs = new ArrayList<>();

    private SuperstructureInputs() {
    }

    public static SuperstructureInputs create() {
        return new SuperstructureInputs();
    }

    public static SuperstructureInputs from(List<SuperstructureInputDefinition> definitions) {
        SuperstructureInputs inputs = create();
        if (definitions != null) {
            inputs.inputs.addAll(definitions);
        }
        return inputs;
    }

    public SuperstructureInputs boolVal(String name, boolean value) {
        return boolVal(name, () -> value);
    }

    public SuperstructureInputs boolVal(String name, BooleanSupplier supplier) {
        inputs.add(new SuperstructureBooleanInputDefinition(name, Objects.requireNonNull(supplier, "supplier")));
        return this;
    }

    public SuperstructureInputs doubleVal(String name, double value) {
        return doubleVal(name, () -> value);
    }

    public SuperstructureInputs doubleVal(String name, DoubleSupplier supplier) {
        inputs.add(new SuperstructureDoubleInputDefinition(name, Objects.requireNonNull(supplier, "supplier")));
        return this;
    }

    public SuperstructureInputs intVal(String name, int value) {
        return intVal(name, () -> value);
    }

    public SuperstructureInputs intVal(String name, IntSupplier supplier) {
        inputs.add(new SuperstructureIntInputDefinition(name, Objects.requireNonNull(supplier, "supplier")));
        return this;
    }

    public SuperstructureInputs stringVal(String name, String value) {
        return stringVal(name, () -> value);
    }

    public SuperstructureInputs stringVal(String name, Supplier<String> supplier) {
        inputs.add(new SuperstructureStringInputDefinition(name, Objects.requireNonNull(supplier, "supplier")));
        return this;
    }

    public SuperstructureInputs pose2dVal(String name, Supplier<Pose2d> supplier) {
        inputs.add(new SuperstructurePose2dInputDefinition(name, Objects.requireNonNull(supplier, "supplier")));
        return this;
    }

    public SuperstructureInputs pose3dVal(String name, Supplier<Pose3d> supplier) {
        inputs.add(new SuperstructurePose3dInputDefinition(name, Objects.requireNonNull(supplier, "supplier")));
        return this;
    }

    public SuperstructureInputs objVal(String name, Supplier<?> supplier) {
        inputs.add(new SuperstructureObjectInputDefinition(name, Objects.requireNonNull(supplier, "supplier")));
        return this;
    }

    public SuperstructureInputs merge(SuperstructureInputs other) {
        if (other != null) {
            inputs.addAll(other.inputs);
        }
        return this;
    }

    public List<SuperstructureInputDefinition> definitions() {
        return List.copyOf(inputs);
    }
}
