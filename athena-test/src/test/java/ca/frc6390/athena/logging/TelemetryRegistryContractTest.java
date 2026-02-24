package ca.frc6390.athena.logging;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.TelemetryRegistryExamples;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

final class TelemetryRegistryContractTest {

    @Test
    void destinationRoutingHonorsTelemetryDestination() {
        RecordingSink disk = new RecordingSink();
        RecordingSink nt = new RecordingSink();
        TelemetryRegistry registry = new TelemetryRegistry(disk, nt, 1);
        RoutingModel model = new RoutingModel();

        registry.register(model);
        model.disk = 2.0;
        model.nt = 3.0;
        model.both = 4.0;
        registry.tick(10);

        assertEquals(List.of(2.0), disk.values("disk"));
        assertEquals(List.of(4.0), disk.values("both"));
        assertEquals(List.of(3.0), nt.values("nt"));
        assertEquals(List.of(4.0), nt.values("both"));
    }

    @Test
    void epsilonSuppressesSmallChangesAndPublishesLargeChanges() {
        RecordingSink disk = new RecordingSink();
        RecordingSink nt = new RecordingSink();
        TelemetryRegistry registry = new TelemetryRegistry(disk, nt, 1);
        EpsilonModel model = new EpsilonModel();

        registry.register(model);
        model.value = 1.2;
        registry.tick(100);
        assertTrue(disk.values("eps").isEmpty());

        model.value = 1.7;
        registry.tick(200);
        assertEquals(List.of(1.7), disk.values("eps"));
    }

    @Test
    void highBandwidthTypesEnforceMinimumPublishPeriod() {
        RecordingSink disk = new RecordingSink();
        RecordingSink nt = new RecordingSink();
        TelemetryRegistry registry = new TelemetryRegistry(disk, nt, 1);
        ArrayModel model = new ArrayModel();

        registry.register(model);
        model.values = new double[] {2.0, 3.0};
        registry.tick(100);
        assertTrue(disk.values("arr").isEmpty());

        registry.tick(200);
        assertEquals(1, disk.values("arr").size());
        assertTrue(Arrays.equals(new double[] {2.0, 3.0}, (double[]) disk.values("arr").get(0)));

        model.values = new double[] {9.0};
        registry.tick(250);
        assertEquals(1, disk.values("arr").size());
        registry.tick(400);
        assertEquals(2, disk.values("arr").size());
    }

    @Test
    void disabledRegistrySkipsPublishing() {
        RecordingSink disk = new RecordingSink();
        RecordingSink nt = new RecordingSink();
        TelemetryRegistry registry = new TelemetryRegistry(disk, nt, 1);
        RoutingModel model = new RoutingModel();

        registry.register(model);
        registry.setEnabled(false);
        model.disk = 5.0;
        registry.tick(100);
        assertTrue(disk.values("disk").isEmpty());
    }

    @Test
    void publicFactoryAndExampleCanRegisterWithoutSinks() {
        TelemetryRegistry registry = TelemetryRegistryExamples.createNoSinkRegistry();
        TelemetryRegistryExamples.DriveTelemetryModel model = new TelemetryRegistryExamples.DriveTelemetryModel();

        TelemetryRegistryExamples.registerAndTick(registry, model, 100);

        assertNotNull(registry);
        assertTrue(registry.isEnabled());
        assertFalse(registry.entryCount() == 0);
    }

    private static final class RoutingModel {
        @Telemetry(key = "disk", destination = TelemetryDestination.DISK, periodMs = 1)
        private double disk = 1.0;

        @Telemetry(key = "nt", destination = TelemetryDestination.SHUFFLEBOARD, periodMs = 1)
        private double nt = 1.0;

        @Telemetry(key = "both", destination = TelemetryDestination.BOTH, periodMs = 1)
        private double both = 1.0;
    }

    private static final class EpsilonModel {
        @Telemetry(key = "eps", destination = TelemetryDestination.BOTH, periodMs = 1, epsilon = 0.5)
        private double value = 1.0;
    }

    private static final class ArrayModel {
        @Telemetry(key = "arr", destination = TelemetryDestination.BOTH, periodMs = 10)
        private double[] values = new double[] {1.0};
    }

    private static final class RecordingSink implements TelemetrySink {
        private final Map<String, List<Object>> valuesByKey = new HashMap<>();

        @Override
        public TelemetryOutput create(String key, TelemetryValueType type, Object initialValue) {
            valuesByKey.computeIfAbsent(key, unused -> new ArrayList<>());
            return value -> valuesByKey.get(key).add(copyValue(value));
        }

        public List<Object> values(String key) {
            return valuesByKey.getOrDefault(key, List.of());
        }

        private Object copyValue(Object value) {
            if (value instanceof double[] data) {
                return Arrays.copyOf(data, data.length);
            }
            if (value instanceof boolean[] data) {
                return Arrays.copyOf(data, data.length);
            }
            if (value instanceof int[] data) {
                return Arrays.copyOf(data, data.length);
            }
            if (value instanceof long[] data) {
                return Arrays.copyOf(data, data.length);
            }
            if (value instanceof String[] data) {
                return Arrays.copyOf(data, data.length);
            }
            return value;
        }
    }
}
