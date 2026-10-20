package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.runtime.MotorGroup;
import ca.frc6390.athena.hardware.runtime.MotorGroupEvent;
import ca.frc6390.athena.hardware.runtime.MotorGroupPolicy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;

/**
 * How a mechanism responds when one motor of a leader-and-followers group dies. Declare it as a mechanism field;
 * the group is the leader and every motor declared with {@code .follow(leader)}.
 *
 * <pre>{@code
 * private final MotorFailover failover = MotorFailover.of(leader)
 *         .promoteLeader()            // the leader dies: a surviving follower leads
 *         .dropFailedFollowers()      // a follower dies: keep running without it
 *         .derateOutput(0.8)          // while degraded, scale percent and voltage commands
 *         .onFailure(event -> DataLogManager.log(event.toString()));
 * }</pre>
 *
 * <ul>
 *   <li>Without a declaration, or with {@link #disableMechanism()}, any member death applies the motor's own
 *       {@link ca.frc6390.athena.api.FailurePolicy} (by default the whole mechanism is disabled).</li>
 *   <li>{@link #promoteLeader()} moves commands, closed-loop targets and the leader's integrated-encoder
 *       feedback to the first alive follower, keeps position continuous, and re-points the other followers.</li>
 *   <li>{@link #dropFailedFollowers()} stops commanding a dead follower and keeps running.</li>
 *   <li>A recovered motor rejoins as a follower of the current leader; leadership never flips back on its own.</li>
 * </ul>
 *
 * <p>Telemetry: {@code Leader}, {@code Alive}, {@code AliveMotors}, {@code DeadMotors}, {@code Degraded} and
 * {@code Failed} under the field's name.</p>
 */
public final class MotorFailover implements TelemetrySource {
    private final MotorDevice leader;
    private final List<MotorDevice> promotionOrder = new ArrayList<>();
    private boolean promoteLeader;
    private boolean dropFollowers;
    private int minimumAlive = 1;
    private double outputScale = 1.0;
    private double currentLimitScale = 1.0;
    private boolean rejoin = true;
    private int recoverySamples = 5;
    private Consumer<MotorGroupEvent> onFailure = event -> { };
    private Consumer<MotorGroupEvent> onRecovery = event -> { };
    private MotorGroup group;

    private MotorFailover(MotorDevice leader) {
        this.leader = Objects.requireNonNull(leader, "leader");
        if (leader.follower() != null) {
            throw new IllegalArgumentException("Declare motor failover on the group leader, not on follower "
                    + leader.defaultName() + ".");
        }
    }

    /**
     * Starts a failover declaration for the group led by {@code leader}. It starts strict: any member death
     * applies the motor's failure policy, as if nothing were declared.
     *
     * @param leader group leader
     * @return failover declaration
     */
    public static MotorFailover of(MotorDevice leader) {
        return new MotorFailover(leader);
    }

    /**
     * Promotes the first alive follower when the leader dies. Followers are tried in ascending CAN id.
     *
     * @return this declaration
     */
    public MotorFailover promoteLeader() {
        requireUnbound();
        promoteLeader = true;
        return this;
    }

    /**
     * Promotes followers in the given order when the leader dies; unlisted followers come after, by CAN id.
     *
     * @param order preferred new leaders
     * @return this declaration
     */
    public MotorFailover promoteLeader(MotorDevice... order) {
        promoteLeader();
        promotionOrder.clear();
        promotionOrder.addAll(Arrays.asList(Objects.requireNonNull(order, "order")));
        return this;
    }

    /**
     * Keeps running without a dead follower instead of applying the failure policy.
     *
     * @return this declaration
     */
    public MotorFailover dropFailedFollowers() {
        requireUnbound();
        dropFollowers = true;
        return this;
    }

    /**
     * Restores the strict response: any member death applies the motors' failure policy, which disables the
     * mechanism by default.
     *
     * @return this declaration
     */
    public MotorFailover disableMechanism() {
        requireUnbound();
        promoteLeader = false;
        dropFollowers = false;
        promotionOrder.clear();
        return this;
    }

    /**
     * Applies the failure policy once fewer than {@code motors} are alive, even when promotion or dropping would
     * keep the group running.
     *
     * @param motors fewest alive motors, at least 1
     * @return this declaration
     */
    public MotorFailover minimumAlive(int motors) {
        requireUnbound();
        if (motors < 1) throw new IllegalArgumentException("A motor group needs at least one alive motor.");
        minimumAlive = motors;
        return this;
    }

    /**
     * Scales percent and voltage commands while any member is dead, so survivors are not asked for the dead
     * motor's share. Onboard closed-loop targets are not scaled; use {@link #derateCurrentLimit(double)} for them.
     *
     * @param scale output fraction in (0, 1]
     * @return this declaration
     */
    public MotorFailover derateOutput(double scale) {
        requireUnbound();
        outputScale = fraction(scale, "Output derate");
        return this;
    }

    /**
     * Scales the survivors' declared supply and stator current limits while any member is dead. Applied once when
     * the group degrades and restored when it is whole again; controllers without runtime configuration keep
     * their limits.
     *
     * @param scale limit fraction in (0, 1]
     * @return this declaration
     */
    public MotorFailover derateCurrentLimit(double scale) {
        requireUnbound();
        currentLimitScale = fraction(scale, "Current-limit derate");
        return this;
    }

    /**
     * Selects whether a recovered motor rejoins the group as a follower (default true).
     *
     * @param rejoin false to leave recovered motors out until the robot program restarts
     * @return this declaration
     */
    public MotorFailover rejoinOnRecovery(boolean rejoin) {
        requireUnbound();
        this.rejoin = rejoin;
        return this;
    }

    /**
     * Sets how many consecutive healthy hardware refreshes a dead motor needs before it rejoins (default 5).
     *
     * @param refreshes healthy refreshes, at least 1
     * @return this declaration
     */
    public MotorFailover recoverAfter(int refreshes) {
        requireUnbound();
        if (refreshes < 1) throw new IllegalArgumentException("Recovery needs at least one healthy refresh.");
        recoverySamples = refreshes;
        return this;
    }

    /**
     * Runs a callback on the robot loop for each failure: a promotion, a dropped follower, or the group failing.
     *
     * @param callback failure callback
     * @return this declaration
     */
    public MotorFailover onFailure(Consumer<MotorGroupEvent> callback) {
        onFailure = Objects.requireNonNull(callback, "callback");
        return this;
    }

    /**
     * Runs a callback on the robot loop when a dead motor rejoins.
     *
     * @param callback recovery callback
     * @return this declaration
     */
    public MotorFailover onRecovery(Consumer<MotorGroupEvent> callback) {
        onRecovery = Objects.requireNonNull(callback, "callback");
        return this;
    }

    /** Returns the declared group leader. */
    public MotorDevice declaredLeader() {
        return leader;
    }

    /** Returns the hardware policy this declaration describes. */
    public MotorGroupPolicy policy() {
        return new MotorGroupPolicy(leader, promotionOrder, promoteLeader, dropFollowers, minimumAlive,
                outputScale, currentLimitScale, rejoin, recoverySamples);
    }

    /** Returns the runtime group once a robot runtime has bound this declaration. */
    public Optional<MotorGroup> group() {
        return Optional.ofNullable(group);
    }

    /** Returns the motor currently leading the group: the declared leader until a promotion. */
    public MotorDevice leader() {
        MotorGroup bound = group;
        return bound == null ? leader : bound.leader();
    }

    /** Returns whether a member is alive. Unbound declarations report every motor alive. */
    public boolean alive(MotorDevice motor) {
        MotorGroup bound = group;
        return bound == null || bound.alive(motor);
    }

    @Override
    public Map<String, TelemetryValue> telemetry() {
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        values.put("Leader", TelemetryValue.string(() -> leader().defaultName()));
        values.put("Alive", TelemetryValue.number(() -> group == null ? 0.0 : group.aliveCount()));
        values.put("AliveMotors", TelemetryValue.string(() -> group == null ? "" : group.aliveMotors()));
        values.put("DeadMotors", TelemetryValue.string(() -> group == null ? "" : group.deadMotors()));
        values.put("Degraded", TelemetryValue.bool(() -> group != null && group.degraded()));
        values.put("Failed", TelemetryValue.bool(() -> group != null && group.failed()));
        return values;
    }

    /** Registers this declaration's group with the graph that owns its motors. */
    void bind(HardwareGraph graph) {
        if (group == null) group = graph.motorGroup(policy());
    }

    /** Delivers group events recorded since the previous cycle to the declared callbacks. */
    void dispatch() {
        MotorGroup bound = group;
        if (bound == null || !bound.hasEvents()) return;
        bound.drainEvents(event -> {
            try {
                (event.failure() ? onFailure : onRecovery).accept(event);
            } catch (RuntimeException exception) {
                FaultIsolation.report("Motor failover callback for " + leader.defaultName() + " threw: "
                        + exception);
            }
        });
    }

    private void requireUnbound() {
        if (group != null) {
            throw new IllegalStateException("Motor failover for " + leader.defaultName()
                    + " is already running; declare it before registering the mechanism.");
        }
    }

    private static double fraction(double value, String name) {
        if (!(value > 0.0 && value <= 1.0)) throw new IllegalArgumentException(name + " must be in (0, 1].");
        return value;
    }
}
