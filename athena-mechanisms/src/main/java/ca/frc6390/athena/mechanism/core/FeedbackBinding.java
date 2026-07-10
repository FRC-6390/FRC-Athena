package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Objects;
import java.util.Set;

/**
 * Explicit position and velocity inputs for a control binding.
 *
 * <p>Most controls use {@link ControlBinding#feedback(EncoderDevice)}, which
 * assigns one encoder to both channels. This type is the advanced path for
 * independently filtered or fused position and velocity signals.</p>
 *
 * @param position position input
 * @param velocity velocity input
 */
public record FeedbackBinding(PositionSignal position, VelocitySignal velocity) {
    public FeedbackBinding {
        Objects.requireNonNull(position, "position");
        Objects.requireNonNull(velocity, "velocity");
    }

    /**
     * Returns the physical declarations required by both signals.
     *
     * @return immutable signal dependencies
     */
    public Set<Object> dependencies() {
        Set<Object> dependencies = new LinkedHashSet<>();
        dependencies.addAll(position.dependencies());
        dependencies.addAll(velocity.dependencies());
        return Collections.unmodifiableSet(dependencies);
    }

    /**
     * Returns encoder dependencies used by this binding.
     *
     * @return immutable encoder dependencies
     */
    public Set<EncoderDevice> encoders() {
        Set<EncoderDevice> encoders = new LinkedHashSet<>();
        for (Object dependency : dependencies()) {
            if (dependency instanceof EncoderDevice encoder) {
                encoders.add(encoder);
            }
        }
        return Collections.unmodifiableSet(encoders);
    }
}
