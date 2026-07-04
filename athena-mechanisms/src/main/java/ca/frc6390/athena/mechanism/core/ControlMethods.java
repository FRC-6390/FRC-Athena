package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.GearRatioRef;
import ca.frc6390.athena.mechanism.ref.FeedforwardRef;
import ca.frc6390.athena.mechanism.ref.PidRef;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Factories for common control method descriptors.
 */
public final class ControlMethods {
    private ControlMethods() {
    }

    public static ControlMethod pid(double p, double i, double d) {
        return pid(PidRef.of(p, i, d));
    }

    public static ControlMethod pid(PidRef pid) {
        return new Pid(Objects.requireNonNull(pid, "pid"));
    }

    public static ControlMethod ff(double staticGain, double velocityGain, double gravityGain) {
        return feedforward(FeedforwardRef.of(staticGain, velocityGain, gravityGain));
    }

    public static ControlMethod feedforward(FeedforwardRef feedforward) {
        return new Feedforward(Objects.requireNonNull(feedforward, "feedforward"));
    }

    public static ControlMethod crt(EncoderRef coarse, EncoderRef fine, GearRatioRef ratio) {
        return new Crt(coarse, fine, ratio);
    }

    public static ControlMethod custom(String kind, Object... refs) {
        return new Custom(kind, List.copyOf(Arrays.asList(refs)));
    }

    public record Pid(PidRef pid) implements ControlMethod {
        public Pid {
            Objects.requireNonNull(pid, "pid");
        }

        @Override
        public String kind() {
            return "pid";
        }

        @Override
        public Map<String, Double> values() {
            return Map.of(
                    "p", pid.p(),
                    "i", pid.i(),
                    "d", pid.d(),
                    "iZone", pid.iZone(),
                    "tolerance", pid.tolerance());
        }
    }

    public record Feedforward(FeedforwardRef feedforward) implements ControlMethod {
        public Feedforward {
            Objects.requireNonNull(feedforward, "feedforward");
        }

        @Override
        public String kind() {
            return "feedforward";
        }

        @Override
        public Map<String, Double> values() {
            return Map.of(
                    "static", feedforward.staticGain(),
                    "velocity", feedforward.velocityGain(),
                    "gravity", feedforward.gravityGain());
        }
    }

    public record Crt(EncoderRef coarse, EncoderRef fine, GearRatioRef ratio) implements ControlMethod {
        public Crt {
            Objects.requireNonNull(coarse, "coarse");
            Objects.requireNonNull(fine, "fine");
            Objects.requireNonNull(ratio, "ratio");
        }

        @Override
        public String kind() {
            return "crt";
        }

        @Override
        public List<Object> refs() {
            return List.of(coarse, fine, ratio);
        }
    }

    public record Custom(String kind, List<Object> refs) implements ControlMethod {
        public Custom {
            kind = kind == null || kind.isBlank() ? "custom" : kind;
            refs = refs == null ? List.of() : List.copyOf(refs);
        }
    }
}
