package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.signal.ImuSource;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.Objects;

/** Compiled declared dependencies, with live acknowledgement checks at every action fence. */
final class HardwareReadiness {
    private final ActionContext context;
    private final Object[] dependencies;
    private final Object[] handles;
    private final IdentityHashMap<Object, Boolean> resolved = new IdentityHashMap<>();
    private long resolvedRevision = Long.MIN_VALUE;
    private int handleCount;

    HardwareReadiness(ActionContext context, Iterable<?> declarations) {
        this.context = Objects.requireNonNull(context, "context");
        ArrayList<Object> devices = new ArrayList<>();
        IdentityHashMap<Object, Boolean> seen = new IdentityHashMap<>();
        for (Object declaration : declarations) {
            add(devices, seen, declaration);
            if (declaration instanceof ControlBinding<?> control) {
                for (MotorDevice motor : control.motors()) add(devices, seen, motor);
            }
            if (declaration instanceof ImuSource source) {
                for (Object dependency : source.dependencies()) add(devices, seen, dependency);
            }
        }
        dependencies = devices.toArray();
        handles = new Object[dependencies.length];
    }

    private static void add(ArrayList<Object> devices, IdentityHashMap<Object, Boolean> seen, Object device) {
        if ((device instanceof MotorDevice || device instanceof EncoderDevice || device instanceof ImuDevice)
                && seen.put(device, Boolean.TRUE) == null) devices.add(device);
    }

    boolean ready() {
        if (dependencies.length == 0) return true;
        // Background binding may replace placeholders while dependencies are resolved. Retry once against
        // the new revision; continuous topology changes defer this evaluation instead of spinning the loop.
        for (int attempt = 0; attempt < 2; attempt++) {
            long revision = context.bindingRevision();
            if (revision < 0 || revision != resolvedRevision) {
                resolve();
                resolvedRevision = revision;
            }
            boolean ready = handlesReady();
            if (revision < 0 || context.bindingRevision() == revision) return ready;
            resolvedRevision = Long.MIN_VALUE;
        }
        return false;
    }

    private void resolve() {
        resolvedRevision = Long.MIN_VALUE;
        // Do not retain handles removed by a later rebind or by a partial context.
        java.util.Arrays.fill(handles, null);
        handleCount = 0;
        resolved.clear();
        for (Object dependency : dependencies) {
            Object handle;
            try {
                if (dependency instanceof MotorDevice motor) handle = context.motor(motor);
                else if (dependency instanceof EncoderDevice encoder) handle = context.encoder(encoder);
                else handle = context.imu((ImuDevice) dependency);
            } catch (UnsupportedOperationException unsupported) {
                // Explicit partial/test contexts may omit unused signals. Graphs return pending handles.
                continue;
            }
            // Equal declarations can resolve to a shared physical handle. Check that handle only once.
            if (resolved.put(handle, Boolean.TRUE) == null) handles[handleCount++] = handle;
        }
    }

    private boolean handlesReady() {
        for (int index = 0; index < handleCount; index++) {
            Object handle = handles[index];
            if (handle instanceof MotorHandle motor) {
                if (!motor.configurationStatus().ready()) return false;
            } else if (handle instanceof EncoderHandle encoder) {
                if (!encoder.configurationStatus().ready()) return false;
            } else if (!((ImuHandle) handle).configurationStatus().ready()) return false;
        }
        return true;
    }
}
