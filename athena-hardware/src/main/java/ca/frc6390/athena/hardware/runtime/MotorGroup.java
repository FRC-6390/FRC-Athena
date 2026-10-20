package ca.frc6390.athena.hardware.runtime;

import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

/**
 * Runtime health of one leader-and-followers motor group, owned by its {@link HardwareGraph}.
 *
 * <p>Each graph refresh judges every member alive or dead from the signals the hardware layer already has: a
 * refresh that throws (CAN lost, or stale signals on controllers that check staleness), a command that throws,
 * and {@link MotorHandle#faulted()}. When the group changes, it rewires once: it promotes a surviving follower
 * when the leader dies, re-points hardware followers to the new leader (or mirrors them in software when they
 * cannot hardware-follow it), stops commanding dead motors, and applies the declared derate. Steady-state cycles
 * only read flags; they do not allocate.</p>
 *
 * <p>Directions: every member has a raw sign, the direction its shaft turns for a positive group command. The
 * declared leader's is its inversion; a hardware follower's is its own {@code inverted()} times the leader's (it
 * opposes the leader when inverted); a software follower's is its own {@code inverted()}, as Athena already
 * mirrors it. A member driven directly receives the command times its raw sign times its controller's
 * inversion, so each motor keeps turning the way it did before the failure.</p>
 */
public final class MotorGroup {
    /** Graph operations the group needs while rewiring. Called with the graph monitor held. */
    interface Wiring {
        RuntimeException refreshFailure(HardwareIdentity identity);

        boolean supportsHardwareFollowing(MotorDevice follower, MotorDevice leader);

        void softwareFollower(ActionContext.SoftwareMotorFollower follower, boolean add);

        boolean absoluteEncoderOn(HardwareIdentity motor);
    }

    private enum Mode { LEADING, HARDWARE, SOFTWARE, DEAD }

    private static final class Member {
        private final MotorDevice device;
        private final HardwareIdentity identity;
        private final MotorHandle backend;
        private final double rawSign;
        private double controllerSign;
        private Mode mode;
        private ActionContext.SoftwareMotorFollower software;
        private Member following;
        private volatile boolean alive = true;
        private boolean lostAsLeader;
        private boolean justDied;
        private boolean recovered;
        private boolean operationFailed;
        private int healthySamples;
        private double limitScale = 1.0;
        private String reason = "";

        private Member(MotorDevice device, HardwareIdentity identity, MotorHandle backend,
                double rawSign, double controllerSign, Mode mode) {
            this.device = device;
            this.identity = identity;
            this.backend = backend;
            this.rawSign = rawSign;
            this.controllerSign = controllerSign;
            this.mode = mode;
        }
    }

    private final MotorGroupPolicy policy;
    private final HardwareIdentity leaderIdentity;
    private final Wiring wiring;
    private Member[] members = new Member[0];
    private Member declaredLeader;
    private Member leading;
    private FailoverMotorHandle leaderHandle;
    private boolean failed;
    private boolean degraded;
    private volatile MotorDevice leaderDevice;
    private volatile int aliveCount;
    private volatile boolean failedState;
    private volatile boolean degradedState;
    private volatile String aliveNames = "";
    private volatile String deadNames = "";
    private final List<MotorGroupEvent> events = new ArrayList<>();
    private volatile boolean eventsPending;

    MotorGroup(MotorGroupPolicy policy, HardwareIdentity leaderIdentity, Wiring wiring) {
        this.policy = Objects.requireNonNull(policy, "policy");
        this.leaderIdentity = Objects.requireNonNull(leaderIdentity, "leaderIdentity");
        this.wiring = Objects.requireNonNull(wiring, "wiring");
        leaderDevice = policy.leader();
    }

    /** Returns the declared policy. */
    public MotorGroupPolicy policy() { return policy; }

    /** Returns the motor currently leading the group. */
    public MotorDevice leader() { return leaderDevice; }

    /** Returns the number of alive motors. */
    public int aliveCount() { return aliveCount; }

    /** Returns whether the group cannot keep running and its motors' failure policy applies. */
    public boolean failed() { return failedState; }

    /** Returns whether any member is dead while the group keeps running. */
    public boolean degraded() { return degradedState; }

    /** Returns the comma-separated names of alive motors. */
    public String aliveMotors() { return aliveNames; }

    /** Returns the comma-separated names of dead motors. */
    public String deadMotors() { return deadNames; }

    /** Returns every member the graph has bound, leader first. */
    public synchronized List<MotorDevice> motors() {
        List<MotorDevice> devices = new ArrayList<>(members.length);
        for (Member member : members) devices.add(member.device);
        return List.copyOf(devices);
    }

    /** Returns whether a member is alive; false for motors outside the group. */
    public boolean alive(MotorDevice motor) {
        for (Member member : members) {
            if (member.device.equals(motor)) return member.alive;
        }
        return false;
    }

    /** Returns whether events are waiting to be drained. */
    public boolean hasEvents() { return eventsPending; }

    /** Delivers and clears the events recorded since the previous drain, oldest first. */
    public void drainEvents(Consumer<MotorGroupEvent> consumer) {
        Objects.requireNonNull(consumer, "consumer");
        if (!eventsPending) return;
        List<MotorGroupEvent> drained;
        synchronized (events) {
            drained = List.copyOf(events);
            events.clear();
            eventsPending = false;
        }
        drained.forEach(consumer);
    }

    HardwareIdentity leaderIdentity() { return leaderIdentity; }

    /** Wraps the declared leader's handle; the wrapper is what the graph stores for the leader. */
    synchronized FailoverMotorHandle adoptLeader(MotorHandle handle) {
        if (leaderHandle != null) return leaderHandle;
        MotorDevice device = handle.device();
        if (device.follower() != null) {
            throw new IllegalArgumentException("Motor group leader " + device.defaultName()
                    + " follows another motor; declare the group on the motor it follows.");
        }
        double sign = sign(device.isInverted());
        declaredLeader = new Member(device, leaderIdentity, handle, sign, sign, Mode.LEADING);
        leading = declaredLeader;
        leaderHandle = new FailoverMotorHandle(handle);
        add(declaredLeader);
        return leaderHandle;
    }

    /** Adds a follower created by the graph. {@code software} is non-null when Athena mirrors it in software. */
    synchronized void adoptFollower(
            MotorDevice device, HardwareIdentity identity, MotorHandle handle,
            ActionContext.SoftwareMotorFollower software) {
        for (Member member : members) {
            if (member.identity.equals(identity)) return;
        }
        Member member;
        if (software != null && handle instanceof SoftwareFollowerMotorHandle mirrored) {
            member = new Member(device, identity, mirrored.backendHandle(),
                    sign(device.isInverted()), 1.0, Mode.SOFTWARE);
            member.software = software;
        } else {
            double leaderSign = declaredLeader == null ? 1.0 : declaredLeader.rawSign;
            member = new Member(device, identity, handle, sign(device.isInverted()) * leaderSign, 1.0, Mode.HARDWARE);
            member.following = declaredLeader;
        }
        add(member);
    }

    boolean contains(HardwareIdentity identity) {
        for (Member member : members) {
            if (member.identity.equals(identity)) return true;
        }
        return false;
    }

    /** Records a command that threw. Commands to the declared leader reach whichever motor leads. */
    synchronized void operationFailed(HardwareIdentity identity) {
        if (identity.equals(leaderIdentity) && leading != null) {
            leading.operationFailed = true;
            return;
        }
        for (Member member : members) {
            if (member.identity.equals(identity)) member.operationFailed = true;
        }
    }

    /** Returns whether this group handles a failure of the given member, so it must not disable the mechanism. */
    boolean absorbs(HardwareIdentity identity) {
        return !failed && contains(identity);
    }

    /** Judges every member and rewires the group when one died or recovered. Called once per graph refresh. */
    synchronized void evaluate() {
        boolean changed = false;
        Member[] current = members;
        for (int index = 0; index < current.length; index++) {
            Member member = current[index];
            RuntimeException refreshFailure = wiring.refreshFailure(member.identity);
            boolean faulted = refreshFailure == null && faulted(member.backend);
            boolean dead = member.operationFailed || refreshFailure != null || faulted;
            boolean operationFailed = member.operationFailed;
            member.operationFailed = false;
            if (member.alive) {
                if (!dead) continue;
                member.alive = false;
                member.justDied = true;
                member.lostAsLeader = member == leading;
                member.healthySamples = 0;
                member.reason = refreshFailure != null
                        ? String.valueOf(refreshFailure.getMessage())
                        : faulted ? "controller reports a fault" : operationFailed ? "command failed" : "";
                changed = true;
            } else if (dead) {
                member.healthySamples = 0;
            } else if (policy.rejoin() && ++member.healthySamples >= policy.recoverySamples()) {
                member.alive = true;
                member.recovered = true;
                member.healthySamples = 0;
                changed = true;
            }
        }
        if (changed) rewire();
    }

    private void rewire() {
        Member trigger = null;
        for (Member member : members) {
            if (member.justDied && trigger == null) trigger = member;
        }
        if (!leading.alive && policy.promoteLeader() && !wiring.absoluteEncoderOn(leaderIdentity)) {
            promoteNext();
        }
        int alive = 0;
        boolean unhandled = false;
        for (Member member : members) {
            if (member.alive) {
                alive++;
            } else if (member.lostAsLeader ? !policy.promoteLeader() : !policy.dropFollowers()) {
                unhandled = true;
            }
        }
        boolean nowFailed = !leading.alive || alive < policy.minimumAlive() || unhandled;
        boolean nowDegraded = alive < members.length;
        double scale = nowDegraded ? policy.outputScale() : 1.0;
        for (Member member : members) {
            if (member.justDied) {
                unwire(member);
                if (!member.lostAsLeader && policy.dropFollowers() && !nowFailed) {
                    record(MotorGroupEvent.Kind.FOLLOWER_DROPPED, member, alive);
                }
            }
        }
        for (Member member : members) {
            if (!member.alive || member == leading) continue;
            boolean rejoined = member.recovered;
            if (member.mode == Mode.DEAD || member.mode == Mode.HARDWARE && member.following != leading) {
                wire(member);
            }
            if (rejoined) record(MotorGroupEvent.Kind.MOTOR_RECOVERED, member, alive);
        }
        leaderHandle.scale(scale);
        double limitScale = nowDegraded ? policy.currentLimitScale() : 1.0;
        for (Member member : members) {
            if (member.mode == Mode.SOFTWARE && member.software.handle() instanceof SoftwareFollowerMotorHandle mirrored) {
                mirrored.scale(scale);
            }
            if (member.alive && member.limitScale != limitScale) applyLimits(member, limitScale);
            member.justDied = false;
            member.recovered = false;
        }
        if (nowFailed && !failed) {
            record(MotorGroupEvent.Kind.GROUP_FAILED, trigger == null ? leading : trigger, alive);
        }
        failed = nowFailed;
        degraded = nowDegraded;
        publish(alive);
    }

    private void promoteNext() {
        for (Member candidate : candidates()) {
            if (!candidate.alive || candidate == leading) continue;
            if (candidate.mode == Mode.HARDWARE) {
                try {
                    candidate.backend.stopFollowing();
                    candidate.controllerSign = 1.0;
                } catch (RuntimeException exception) {
                    candidate.alive = false;
                    candidate.justDied = true;
                    candidate.reason = "could not leave follower mode: " + exception.getMessage();
                    continue;
                }
            } else if (candidate.mode == Mode.SOFTWARE) {
                wiring.softwareFollower(candidate.software, false);
                candidate.software = null;
            }
            double lastPosition = finite(leaderHandle::integratedPositionRotations);
            double direction = candidate.rawSign * candidate.controllerSign;
            double offset = lastPosition - direction * finite(candidate.backend::integratedPositionRotations);
            leaderHandle.lead(candidate.backend, direction, candidate.controllerSign < 0.0, offset);
            Member previous = leading;
            candidate.mode = Mode.LEADING;
            candidate.following = null;
            leading = candidate;
            record(MotorGroupEvent.Kind.LEADER_PROMOTED, previous, aliveAfterPromotion());
            return;
        }
    }

    private List<Member> candidates() {
        List<Member> ordered = new ArrayList<>(members.length);
        for (MotorDevice preferred : policy.promotionOrder()) {
            HardwareIdentity identity = HardwareIdentity.motor(preferred);
            for (Member member : members) {
                if (member.identity.equals(identity) && !ordered.contains(member)) ordered.add(member);
            }
        }
        List<Member> rest = new ArrayList<>(Arrays.asList(members));
        rest.removeAll(ordered);
        rest.sort(Comparator.comparingInt((Member member) -> member == declaredLeader ? 0 : 1)
                .thenComparingInt(member -> member.device.id())
                .thenComparing(member -> member.device.canbus()));
        ordered.addAll(rest);
        return ordered;
    }

    private int aliveAfterPromotion() {
        int alive = 0;
        for (Member member : members) {
            if (member.alive) alive++;
        }
        return alive;
    }

    /** Stops commanding a dead member and stops it, so a stale request cannot resume when it reconnects. */
    private void unwire(Member member) {
        if (member.mode == Mode.SOFTWARE && member.software != null) {
            wiring.softwareFollower(member.software, false);
            member.software = null;
        }
        member.mode = Mode.DEAD;
        member.following = null;
        try {
            member.backend.stop();
        } catch (RuntimeException ignored) {
            // The controller is already unreachable.
        }
    }

    /** Makes an alive member follow the current leader: in hardware when the backends allow it, else in software. */
    private void wire(Member member) {
        if (member == declaredLeader && member.mode == Mode.DEAD && member.controllerSign < 0.0
                && member.backend.supportsRuntimeConfiguration()) {
            MotorRuntimeConfig declared = MotorRuntimeConfig.declared(member.device);
            try {
                member.backend.applyRuntimeConfiguration(new MotorRuntimeConfig(declared.neutralMode(), false,
                        declared.supplyCurrentLimitAmps(), declared.statorCurrentLimitAmps()));
                member.controllerSign = 1.0;
            } catch (RuntimeException ignored) {
                // Keep its inversion; the software direction below accounts for it.
            }
        }
        if (member.controllerSign > 0.0 && wiring.supportsHardwareFollowing(member.device, leading.device)) {
            try {
                member.backend.follow(leading.backend, member.rawSign != leading.rawSign);
                member.mode = Mode.HARDWARE;
                member.following = leading;
                return;
            } catch (RuntimeException ignored) {
                // Fall back to mirroring the command in software.
            }
        }
        if (member.mode == Mode.HARDWARE) {
            try {
                member.backend.stopFollowing();
                member.controllerSign = 1.0;
            } catch (RuntimeException ignored) {
                // The direct command below still replaces the follow request on most controllers.
            }
        }
        SoftwareFollowerMotorHandle mirrored = new SoftwareFollowerMotorHandle(
                member.device, member.backend, member.rawSign * member.controllerSign);
        member.software = new ActionContext.SoftwareMotorFollower(member.device, mirrored);
        wiring.softwareFollower(member.software, true);
        member.mode = Mode.SOFTWARE;
        member.following = null;
    }

    private void applyLimits(Member member, double scale) {
        if (!member.backend.supportsRuntimeConfiguration()) {
            member.limitScale = scale;
            return;
        }
        MotorRuntimeConfig declared = MotorRuntimeConfig.declared(member.device);
        try {
            member.backend.applyRuntimeConfiguration(new MotorRuntimeConfig(
                    declared.neutralMode(),
                    member.controllerSign < 0.0,
                    (int) Math.round(declared.supplyCurrentLimitAmps() * scale),
                    (int) Math.round(declared.statorCurrentLimitAmps() * scale)));
            member.limitScale = scale;
        } catch (RuntimeException ignored) {
            // Retried at the next rewire; a failing controller is judged by its refresh.
        }
    }

    private void record(MotorGroupEvent.Kind kind, Member member, int alive) {
        String reason = kind == MotorGroupEvent.Kind.MOTOR_RECOVERED ? "" : member.reason;
        synchronized (events) {
            events.add(new MotorGroupEvent(kind, member.device, leading.device, alive, reason));
            eventsPending = true;
        }
    }

    private void publish(int alive) {
        StringBuilder aliveBuilder = new StringBuilder();
        StringBuilder deadBuilder = new StringBuilder();
        for (Member member : members) {
            StringBuilder target = member.alive ? aliveBuilder : deadBuilder;
            if (!target.isEmpty()) target.append(',');
            target.append(member.device.defaultName());
        }
        aliveNames = aliveBuilder.toString();
        deadNames = deadBuilder.toString();
        leaderDevice = leading.device;
        aliveCount = alive;
        failedState = failed;
        degradedState = degraded;
    }

    private void add(Member member) {
        Member[] next = Arrays.copyOf(members, members.length + 1);
        next[members.length] = member;
        members = next;
        int alive = 0;
        for (Member existing : members) {
            if (existing.alive) alive++;
        }
        publish(alive);
    }

    private static boolean faulted(MotorHandle handle) {
        try {
            return handle.faulted();
        } catch (RuntimeException exception) {
            return true;
        }
    }

    private static double finite(java.util.function.DoubleSupplier reading) {
        try {
            double value = reading.getAsDouble();
            return Double.isFinite(value) ? value : 0.0;
        } catch (RuntimeException exception) {
            return 0.0;
        }
    }

    private static double sign(boolean inverted) {
        return inverted ? -1.0 : 1.0;
    }
}
