package ca.frc6390.athena.core;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public interface RobotSendableSystem {

    String SHUFFLEBOARD_ROOT = "Athena";
    static boolean isShuffleboardEnabled() {
      return ShuffleboardGate.ENABLED;
    }

    static void setShuffleboardEnabled(boolean enabled) {
      ShuffleboardGate.ENABLED = enabled;
    }

    public static enum SendableLevel {
      COMP,
      DEBUG,
    }

    public interface RobotSendableDevice {

      default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout) {
        return shuffleboard(layout, SendableLevel.COMP);
      }

      default double getShuffleboardPeriodSeconds() {
        return RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();
      }

      default void setShuffleboardPeriodSeconds(double periodSeconds) {}

      ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, SendableLevel level);
    }
    
    default RobotSendableSystem shuffleboard(String tab) {
      shuffleboard(tab, SendableLevel.COMP);
      return this;
    }

    default RobotSendableSystem shuffleboard(String tab, SendableLevel level) {
      if (!isShuffleboardEnabled()) {
        return this;
      }
      shuffleboard(Shuffleboard.getTab(resolveShuffleboardTab(tab)), level);
      return this;
    }

    ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level);

    private static String resolveShuffleboardTab(String tab) {
      if (tab == null || tab.isBlank()) {
        return SHUFFLEBOARD_ROOT;
      }
      if (tab.equals(SHUFFLEBOARD_ROOT) || tab.startsWith(SHUFFLEBOARD_ROOT + "/")) {
        return tab;
      }
      return SHUFFLEBOARD_ROOT + "/" + tab;
    }

    static double getDefaultShuffleboardPeriodSeconds() {
      return ShuffleboardRate.DEFAULT_PERIOD_SECONDS;
    }

    static void setDefaultShuffleboardPeriodSeconds(double periodSeconds) {
      if (!Double.isFinite(periodSeconds) || periodSeconds <= 0.0) {
        return;
      }
      ShuffleboardRate.DEFAULT_PERIOD_SECONDS = periodSeconds;
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

    final class ShuffleboardGate {
      private ShuffleboardGate() {}
      private static volatile boolean ENABLED = true;
    }

    final class ShuffleboardRate {
      private ShuffleboardRate() {}
      private static volatile double DEFAULT_PERIOD_SECONDS = 0.25;
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
        double nowSeconds = Timer.getFPGATimestamp();
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
        double nowSeconds = Timer.getFPGATimestamp();
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
        double nowSeconds = Timer.getFPGATimestamp();
        double periodSeconds = periodSecondsSupplier.getAsDouble();
        if (RateLimiter.shouldUpdate(nowSeconds, lastUpdateSeconds, periodSeconds)) {
          lastValue = supplier.get();
          lastUpdateSeconds = nowSeconds;
        }
        return lastValue;
      }
    }
}
