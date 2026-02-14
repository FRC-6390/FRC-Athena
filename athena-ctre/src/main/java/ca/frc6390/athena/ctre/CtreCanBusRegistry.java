package ca.frc6390.athena.ctre;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

import com.ctre.phoenix6.CANBus;

/**
 * Reuses CANBus handles across CTRE devices.
 *
 * <p>For roboRIO hardware we always resolve to an explicit {@code "rio"} bus
 * instead of relying on the empty-string default bus selection.</p>
 */
public final class CtreCanBusRegistry {
    private static final String RIO_KEY = "rio";
    private static final CANBus RIO_BUS = CANBus.roboRIO();
    private static final ConcurrentMap<String, CANBus> BUS_CACHE = new ConcurrentHashMap<>();

    private CtreCanBusRegistry() {}

    public static CANBus resolve(String canbus) {
        String normalized = normalize(canbus);
        if (RIO_KEY.equals(normalized)) {
            return RIO_BUS;
        }
        return BUS_CACHE.computeIfAbsent(normalized, CANBus::new);
    }

    private static String normalize(String canbus) {
        if (canbus == null || canbus.isBlank()) {
            return RIO_KEY;
        }
        String normalized = canbus.trim();
        return normalized.equalsIgnoreCase(RIO_KEY) ? RIO_KEY : normalized;
    }
}
