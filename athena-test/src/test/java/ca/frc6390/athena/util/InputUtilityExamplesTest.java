package ca.frc6390.athena.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.util.examples.InputUtilityExamples;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

final class InputUtilityExamplesTest {

    @Test
    void safeGainSupplierFallsBackToZero() {
        assertEquals(0.0, InputUtilityExamples.safeGainSupplier(null).getAsDouble(), 1e-9);
        assertEquals(0.6, InputUtilityExamples.safeGainSupplier(() -> 0.6).getAsDouble(), 1e-9);
    }

    @Test
    void optionalTemperatureRejectsOutOfRangeValues() {
        OptionalDouble valid = InputUtilityExamples.optionalSafeTemperature(() -> 35.0);
        OptionalDouble invalid = InputUtilityExamples.optionalSafeTemperature(() -> 200.0);

        assertTrue(valid.isPresent());
        assertEquals(35.0, valid.getAsDouble(), 1e-9);
        assertFalse(invalid.isPresent());
    }

    @Test
    void setpointRefCanBeMutatedAndCleared() {
        InputRef<Double> ref = InputUtilityExamples.createSetpointRef(1.2);
        assertTrue(ref.isSet());
        assertEquals(1.2, ref.get(), 1e-9);

        ref.set(2.4);
        assertEquals(2.4, ref.get(), 1e-9);

        ref.clear();
        assertFalse(ref.isSet());
    }
}
