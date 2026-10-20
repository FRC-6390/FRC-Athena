package ca.frc6390.athena.localization.pipeline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Locale;
import java.util.Random;
import org.junit.jupiter.api.Test;

class LocalizationFiltersFormatTest {
    private static final String[] PATTERNS = {
        "no frame for %.1f s",
        "latency %.3f s > %.3f s",
        "jump %.2f m > %.2f m",
        "tags %d < %d",
        "%s (recovering %d/%d)",
        "whole %.0f",
        "fine %.6f",
    };

    @Test
    void matchesStringFormatForReasonPatterns() {
        Random random = new Random(6390);
        for (int iteration = 0; iteration < 200_000; iteration++) {
            double value = switch (iteration % 6) {
                case 0 -> random.nextDouble() * 10.0;
                // Values on or next to a rounding tie, which String.format decides from the decimal digits.
                case 1 -> Math.round(random.nextDouble() * 100_000.0) / 1000.0 + (random.nextInt(3) - 1) * 0.0005;
                case 2 -> random.nextInt(2000) * 0.05;
                case 3 -> random.nextInt(100_000) / 100.0 + 0.005;
                case 4 -> -random.nextDouble() * 5.0;
                default -> Math.pow(10.0, random.nextDouble() * 14.0 - 4.0);
            };
            String pattern = PATTERNS[iteration % PATTERNS.length];
            Object[] values = switch (pattern) {
                case "tags %d < %d" -> new Object[] {random.nextInt(5) - 1, 2L};
                case "%s (recovering %d/%d)" -> new Object[] {"reason " + iteration % 7, random.nextInt(5), 5};
                case "latency %.3f s > %.3f s", "jump %.2f m > %.2f m" ->
                        new Object[] {value, random.nextDouble() * 3.0};
                default -> new Object[] {value};
            };
            assertEquals(String.format(Locale.ROOT, pattern, values), LocalizationFilters.format(pattern, values),
                    pattern);
        }
    }

    @Test
    void fallsBackForSpecialValuesAndPatterns() {
        Object[][] cases = {
            {"%.1f", new Object[] {Double.NaN}},
            {"%.1f", new Object[] {Double.POSITIVE_INFINITY}},
            {"%.1f", new Object[] {-0.0}},
            {"%.1f", new Object[] {-0.04}},
            {"%.1f", new Object[] {0.25}},
            {"%.1f", new Object[] {0.15}},
            {"%.2f", new Object[] {1.005}},
            {"%.1f", new Object[] {2.0e12}},
            {"%.1f%%", new Object[] {12.5}},
            {"%s", new Object[] {null}},
            {"%d", new Object[] {(short) 3}},
            {"%.1f", new Object[] {1.5f}},
            {"plain", new Object[] {}},
        };
        for (Object[] testCase : cases) {
            String pattern = (String) testCase[0];
            Object[] values = (Object[]) testCase[1];
            assertEquals(String.format(Locale.ROOT, pattern, values), LocalizationFilters.format(pattern, values),
                    pattern);
        }
    }
}
