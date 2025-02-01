package ca.frc6390.athena.filters;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class FilteredValue extends FilterList {

    private DoubleSupplier supplier;

    public FilteredValue(DoubleSupplier supplier) {
        super();
        this.supplier = supplier;
    }

    public double get() {
        double value = getUnfiltered();
        for (Function<Double, Double> filter : getFilterFunctions()) {
            value = filter.apply(value);
        }
        return value;
    }

    public double getUnfiltered() {
        return supplier.getAsDouble();
    }
}
