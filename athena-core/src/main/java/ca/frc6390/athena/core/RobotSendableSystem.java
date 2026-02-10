package ca.frc6390.athena.core;

import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Lightweight "publishable" interface for Athena systems.
 *
 * <p>We publish directly to NetworkTables (not WPILib Shuffleboard containers) so we have full
 * control over paths, update rate, and runtime gating. Dashboards (Elastic, Glass, custom UIs)
 * can consume the raw topics.</p>
 */
public interface RobotSendableSystem {

    RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node);

    interface RobotSendableDevice {
        RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node);
    }

    static DoubleSupplier rateLimit(DoubleSupplier supplier, DoubleSupplier periodSecondsSupplier) {
        return new RateLimitedDoubleSupplier(supplier, periodSecondsSupplier);
    }

    static BooleanSupplier rateLimit(BooleanSupplier supplier, DoubleSupplier periodSecondsSupplier) {
        return new RateLimitedBooleanSupplier(supplier, periodSecondsSupplier);
    }

    static <T> Supplier<T> rateLimit(Supplier<T> supplier, DoubleSupplier periodSecondsSupplier) {
        return new RateLimitedSupplier<>(supplier, periodSecondsSupplier);
    }

    final class RateLimiter {
        private RateLimiter() {}

        static boolean shouldUpdate(double nowSeconds, double lastUpdateSeconds, double periodSeconds) {
            if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
                return true;
            }
            if (!Double.isFinite(nowSeconds) || Double.isNaN(lastUpdateSeconds)) {
                return true;
            }
            return (nowSeconds - lastUpdateSeconds) >= periodSeconds;
        }
    }

    final class RateLimitedDoubleSupplier implements DoubleSupplier {
        private final DoubleSupplier supplier;
        private final DoubleSupplier periodSecondsSupplier;
        private double lastUpdateSeconds = Double.NaN;
        private double lastValue;

        private RateLimitedDoubleSupplier(DoubleSupplier supplier, DoubleSupplier periodSecondsSupplier) {
            this.supplier = supplier;
            this.periodSecondsSupplier = periodSecondsSupplier;
        }

        @Override
        public double getAsDouble() {
            double nowSeconds = RobotTime.nowSeconds();
            if (!Double.isFinite(nowSeconds)) {
                nowSeconds = Timer.getFPGATimestamp();
            }
            double periodSeconds = periodSecondsSupplier.getAsDouble();
            if (RateLimiter.shouldUpdate(nowSeconds, lastUpdateSeconds, periodSeconds)) {
                lastValue = supplier.getAsDouble();
                lastUpdateSeconds = nowSeconds;
            }
            return lastValue;
        }
    }

    final class RateLimitedBooleanSupplier implements BooleanSupplier {
        private final BooleanSupplier supplier;
        private final DoubleSupplier periodSecondsSupplier;
        private double lastUpdateSeconds = Double.NaN;
        private boolean lastValue;

        private RateLimitedBooleanSupplier(BooleanSupplier supplier, DoubleSupplier periodSecondsSupplier) {
            this.supplier = supplier;
            this.periodSecondsSupplier = periodSecondsSupplier;
        }

        @Override
        public boolean getAsBoolean() {
            double nowSeconds = RobotTime.nowSeconds();
            if (!Double.isFinite(nowSeconds)) {
                nowSeconds = Timer.getFPGATimestamp();
            }
            double periodSeconds = periodSecondsSupplier.getAsDouble();
            if (RateLimiter.shouldUpdate(nowSeconds, lastUpdateSeconds, periodSeconds)) {
                lastValue = supplier.getAsBoolean();
                lastUpdateSeconds = nowSeconds;
            }
            return lastValue;
        }
    }

    final class RateLimitedSupplier<T> implements Supplier<T> {
        private final Supplier<T> supplier;
        private final DoubleSupplier periodSecondsSupplier;
        private double lastUpdateSeconds = Double.NaN;
        private T lastValue;

        private RateLimitedSupplier(Supplier<T> supplier, DoubleSupplier periodSecondsSupplier) {
            this.supplier = supplier;
            this.periodSecondsSupplier = periodSecondsSupplier;
        }

        @Override
        public T get() {
            double nowSeconds = RobotTime.nowSeconds();
            if (!Double.isFinite(nowSeconds)) {
                nowSeconds = Timer.getFPGATimestamp();
            }
            double periodSeconds = periodSecondsSupplier.getAsDouble();
            if (RateLimiter.shouldUpdate(nowSeconds, lastUpdateSeconds, periodSeconds)) {
                lastValue = supplier.get();
                lastUpdateSeconds = nowSeconds;
            }
            return lastValue;
        }
    }
}
