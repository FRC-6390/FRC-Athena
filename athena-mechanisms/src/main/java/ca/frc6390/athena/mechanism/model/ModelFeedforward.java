package ca.frc6390.athena.mechanism.model;

import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.ControlLoop;
import ca.frc6390.athena.mechanism.core.ControlLoopBinding;
import ca.frc6390.athena.mechanism.core.ControlLoopContext;
import ca.frc6390.athena.mechanism.core.ControlLoopRole;
import ca.frc6390.athena.mechanism.core.ControlLoopRuntime;
import ca.frc6390.athena.mechanism.core.ControlLoopTrace;
import ca.frc6390.athena.mechanism.core.ControlOutput;
import ca.frc6390.athena.mechanism.motion.MotionReference;
import ca.frc6390.athena.mechanism.core.Output;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.function.Supplier;

/**
 * Feedforward loop that predicts volts from an identified {@link MechanismModel}. It replaces the
 * {@code FeedforwardGains} loop on a binding. Positions, velocities and accelerations are converted
 * from the binding unit to base units (rotations, meters) before the model is evaluated.
 *
 * <p>When the owning {@link Characterization} learns continuously, each cycle also reports the
 * applied motor voltage and the measured motion to it. The report uses the voltage applied in the
 * previous cycle, so it lags the measured motion by one cycle.</p>
 */
public final class ModelFeedforward implements ControlLoop {
    private final Supplier<MechanismModel> model;
    private final Characterization owner;

    public ModelFeedforward(Supplier<MechanismModel> model, Characterization owner) {
        this.model = java.util.Objects.requireNonNull(model, "model");
        this.owner = owner;
    }

    /** Returns the model the loop currently uses. */
    public MechanismModel currentModel() {
        return model.get();
    }

    /** Returns the characterization that learns from this loop, or null for a fixed model. */
    public Characterization owner() {
        return owner;
    }

    @Override
    public ControlLoopRuntime bind(ControlLoopBinding binding) {
        return new Runtime(binding);
    }

    @Override
    public ControlLoopRole role() {
        return ControlLoopRole.DEVICE_CONFIGURABLE;
    }

    private final class Runtime implements ControlLoopRuntime {
        private final ControlLoopBinding binding;
        private final AccelerationEstimator acceleration;
        private ControlLoopTrace trace = ControlLoopTrace.ZERO;

        private Runtime(ControlLoopBinding binding) {
            this.binding = binding;
            this.acceleration = new AccelerationEstimator(owner == null ? MonotonicClock.SYSTEM : owner.clock());
        }

        @Override
        public ControlOutput calculate(ControlLoopContext context) {
            ControlBinding<?> control = binding.control();
            double factor = Identification.baseFactor(control.unit());
            MotionReference reference = context.reference();
            double position = context.position() * factor;
            double velocity = reference.velocity() * factor;
            double accel = reference.acceleration() * factor;
            if (!Double.isFinite(position) || !Double.isFinite(velocity) || !Double.isFinite(accel)) {
                return ControlOutput.neutral();
            }
            double hint = context.request() instanceof Output.Position
                    ? context.target() - context.position()
                    : velocity;
            MechanismModel active = model.get();
            double volts = active.feedforwardVolts(position, velocity, accel, hint);
            double gravity = active.gravityVolts(position);
            double velocityVolts = active.kV() * velocity;
            double accelerationVolts = active.kA() * accel;
            trace = new ControlLoopTrace(0.0, 0.0, 0.0,
                    volts - velocityVolts - accelerationVolts - gravity,
                    velocityVolts, accelerationVolts, gravity);
            if (owner != null && owner.learnContinuously()) {
                observe(control, factor, hint);
            }
            return ControlOutput.voltage(volts);
        }

        private void observe(ControlBinding<?> control, double factor, double hint) {
            MotorDevice output = control.output();
            if (output == null || control.feedback() == null) {
                return;
            }
            double applied;
            try {
                MotorHandle handle = binding.actionContext().motor(output);
                applied = handle.appliedVoltage();
            } catch (RuntimeException unavailable) {
                return;
            }
            double measuredPosition = control.feedback().position().position() * factor;
            double measuredVelocity = control.feedback().velocity().velocity() * factor;
            double measuredAcceleration = acceleration.update(measuredVelocity);
            owner.observeNormalOperation(applied, measuredPosition, measuredVelocity,
                    measuredAcceleration, hint);
        }

        @Override
        public void reset(ControlLoopContext context) {
            trace = ControlLoopTrace.ZERO;
            acceleration.reset();
        }

        @Override
        public ControlLoopTrace trace() {
            return trace;
        }
    }
}
