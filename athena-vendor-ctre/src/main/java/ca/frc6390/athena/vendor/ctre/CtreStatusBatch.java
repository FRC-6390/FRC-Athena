package ca.frc6390.athena.vendor.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * One Phoenix {@code refreshAll} per CAN bus per refresh pass, shared by every CTRE handle on that bus.
 *
 * <p>Each {@code refreshAll} is a JNI call, and Phoenix rejects a call whose signals span two CAN networks, so the
 * batches are keyed by bus name. Within a bus, the first handle to refresh in a pass refreshes the signals of every
 * member, and the other handles of that pass reuse the result.
 *
 * <p>The pass is an explicit token from the caller, never a clock. A handle refreshed with a token that the batch
 * has already refreshed reuses that refresh; any other token refreshes the bus. A handle that threw, or was skipped
 * by the graph's backoff, simply does not refresh in that pass; it can never reuse a refresh from an earlier pass,
 * because passes never repeat. Calls without a token ({@link #UNSHARED}) always refresh, as before this batch.
 *
 * <p>Failure isolation is unchanged: a handle still checks its own connection through {@link SignalFreshness} after
 * the shared refresh, and a refresh status is never thrown from here.
 */
final class CtreStatusBatch {
    /** Pass token for a refresh that is not part of a shared pass. Always refreshes the bus. */
    static final long UNSHARED = Long.MIN_VALUE;

    private static final Map<String, CtreStatusBatch> BUSES = new HashMap<>();

    /** Performs one batched refresh; Phoenix's {@code refreshAll} in production, a counting stub in tests. */
    interface Refresher {
        void refreshAll(BaseStatusSignal[] signals);
    }

    private final Refresher refresher;
    private final List<Member> members = new ArrayList<>();
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private long refreshedPass = UNSHARED;

    CtreStatusBatch(Refresher refresher) {
        this.refresher = refresher;
    }

    /** Returns the shared batch for a CAN bus name. */
    static synchronized CtreStatusBatch forBus(String canbus) {
        return BUSES.computeIfAbsent(canbus, bus -> new CtreStatusBatch(BaseStatusSignal::refreshAll));
    }

    /**
     * Adds a handle's signals to the batch.
     *
     * @param own signals the handle reads
     * @return the membership the handle refreshes through
     */
    synchronized Member join(BaseStatusSignal... own) {
        Member member = new Member(own.clone());
        members.add(member);
        rebuild();
        // The new signals were not part of the last refresh, so the next refresh must include them.
        refreshedPass = UNSHARED;
        return member;
    }

    /** Removes a closed handle's signals from the batch. */
    synchronized void leave(Member member) {
        if (members.remove(member)) {
            rebuild();
        }
    }

    /**
     * Refreshes the bus unless this pass already did.
     *
     * @param pass explicit refresh-pass token, or {@link #UNSHARED}
     */
    synchronized void refresh(long pass) {
        if (pass == UNSHARED || pass != refreshedPass) {
            refresher.refreshAll(signals);
        }
        refreshedPass = pass;
    }

    private void rebuild() {
        int total = 0;
        for (Member member : members) {
            total += member.own.length;
        }
        BaseStatusSignal[] all = new BaseStatusSignal[total];
        int index = 0;
        for (Member member : members) {
            System.arraycopy(member.own, 0, all, index, member.own.length);
            index += member.own.length;
        }
        signals = all;
    }

    /** One handle's signals as registered in its bus batch. */
    static final class Member {
        private final BaseStatusSignal[] own;

        private Member(BaseStatusSignal[] own) {
            this.own = own;
        }
    }
}
