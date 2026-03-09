package ca.frc6390.athena.hardware.encoder;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;

/**
 * Simple stateful scalar filter used by derived encoder sources.
 */
public final class SignalFilter {
    public enum Type {
        LOW_PASS,
        MEDIAN,
        MOVING_AVERAGE
    }

    private final Type type;
    private final double alpha;
    private final int window;
    private final Deque<Double> history = new ArrayDeque<>();
    private boolean initialized;
    private double state;

    private SignalFilter(Type type, double alpha, int window) {
        this.type = type;
        this.alpha = alpha;
        this.window = window;
    }

    public static SignalFilter lowPass(double alpha) {
        if (!Double.isFinite(alpha) || alpha <= 0.0 || alpha > 1.0) {
            throw new IllegalArgumentException("lowPass alpha must be in (0, 1]");
        }
        return new SignalFilter(Type.LOW_PASS, alpha, 0);
    }

    public static SignalFilter median(int window) {
        if (window < 1) {
            throw new IllegalArgumentException("median window must be >= 1");
        }
        return new SignalFilter(Type.MEDIAN, Double.NaN, window);
    }

    public static SignalFilter movingAverage(int window) {
        if (window < 1) {
            throw new IllegalArgumentException("movingAverage window must be >= 1");
        }
        return new SignalFilter(Type.MOVING_AVERAGE, Double.NaN, window);
    }

    public double apply(double value) {
        return switch (type) {
            case LOW_PASS -> applyLowPass(value);
            case MEDIAN -> applyMedian(value);
            case MOVING_AVERAGE -> applyMovingAverage(value);
        };
    }

    public void reset() {
        initialized = false;
        state = 0.0;
        history.clear();
    }

    private double applyLowPass(double value) {
        if (!initialized) {
            initialized = true;
            state = value;
            return state;
        }
        state += alpha * (value - state);
        return state;
    }

    private double applyMedian(double value) {
        push(value);
        List<Double> sorted = new ArrayList<>(history);
        Collections.sort(sorted);
        int size = sorted.size();
        int mid = size / 2;
        if ((size & 1) == 1) {
            state = sorted.get(mid);
        } else {
            state = (sorted.get(mid - 1) + sorted.get(mid)) / 2.0;
        }
        initialized = true;
        return state;
    }

    private double applyMovingAverage(double value) {
        push(value);
        double sum = 0.0;
        for (double sample : history) {
            sum += sample;
        }
        state = history.isEmpty() ? 0.0 : sum / history.size();
        initialized = true;
        return state;
    }

    private void push(double value) {
        history.addLast(value);
        while (history.size() > window) {
            history.removeFirst();
        }
    }
}
