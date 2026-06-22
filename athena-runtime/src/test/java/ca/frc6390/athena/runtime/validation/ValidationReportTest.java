package ca.frc6390.athena.runtime.validation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ValidationReportTest {
    @Test
    void okReportHasNoErrors() {
        ValidationReport report = ValidationReport.ok();

        assertFalse(report.hasErrors());
        assertEquals("Validation OK", report.summary());
    }

    @Test
    void filtersByCodeAndFindsFirstError() {
        ValidationReport report = ValidationReport.builder()
                .error("motor.missing", "intake.roller", "Missing motor")
                .error("motor.missing", "shooter.leader", "Missing motor")
                .error("input.bad", "intake.beam", "Bad input")
                .build();

        assertEquals("intake.roller", report.firstError().orElseThrow().path());
        assertEquals(2, report.errorsWithCode("motor.missing").size());
        assertEquals(0, report.errorsWithCode("missing").size());
    }

    @Test
    void assertValidThrowsWithReport() {
        ValidationReport report = ValidationReport.builder()
                .error("hardware.missing-backend", "shooter.leader", "No backend")
                .build();

        AthenaValidationException exception = assertThrows(AthenaValidationException.class, report::assertValid);

        assertEquals(report, exception.report());
        assertTrue(exception.getMessage().contains("hardware.missing-backend"));
    }

    @Test
    void builderAcceptsErrorObjectsAndMergesReports() {
        AthenaError error = new AthenaError("a", "path.a", "A");
        ValidationReport child = ValidationReport.builder()
                .error("b", "path.b", "B")
                .build();

        ValidationReport report = ValidationReport.builder()
                .error(error)
                .addAll(child)
                .build();

        assertEquals(2, report.errors().size());
        assertEquals("path.a", report.errors().get(0).path());
        assertEquals("path.b", report.errors().get(1).path());
    }
}
