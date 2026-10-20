package ca.frc6390.athena.vendor.ctre;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class CtreStatusBatchTest {
    /** Records each batched refresh and how many signals it covered. */
    private static final class CountingRefresher implements CtreStatusBatch.Refresher {
        private final List<Integer> covered = new ArrayList<>();

        @Override
        public void refreshAll(BaseStatusSignal[] signals) {
            covered.add(signals.length);
        }
    }

    private static BaseStatusSignal[] signals(int count) {
        return new BaseStatusSignal[count];
    }

    @Test
    void sharedPassRefreshesBusOnceForAllMembers() {
        CountingRefresher refresher = new CountingRefresher();
        CtreStatusBatch batch = new CtreStatusBatch(refresher);
        batch.join(signals(3));
        batch.join(signals(2));
        batch.join(signals(4));

        batch.refresh(1L);
        batch.refresh(1L);
        batch.refresh(1L);
        batch.refresh(2L);
        batch.refresh(2L);

        assertEquals(List.of(9, 9), refresher.covered);
    }

    @Test
    void skippedOrFailedMembersNeverReuseAnEarlierPass() {
        CountingRefresher refresher = new CountingRefresher();
        CtreStatusBatch batch = new CtreStatusBatch(refresher);
        batch.join(signals(1));
        batch.join(signals(1));

        batch.refresh(1L);
        // Pass 2: the second member was skipped (or threw). The first member's call must still refresh.
        batch.refresh(2L);
        // Pass 3: only the second member runs. It must refresh rather than reuse pass 2.
        batch.refresh(3L);

        assertEquals(List.of(2, 2, 2), refresher.covered);
    }

    @Test
    void unsharedCallsAlwaysRefresh() {
        CountingRefresher refresher = new CountingRefresher();
        CtreStatusBatch batch = new CtreStatusBatch(refresher);
        batch.join(signals(1));

        batch.refresh(CtreStatusBatch.UNSHARED);
        batch.refresh(CtreStatusBatch.UNSHARED);

        assertEquals(List.of(1, 1), refresher.covered);
    }

    @Test
    void joinForcesNextPassToIncludeNewSignals() {
        CountingRefresher refresher = new CountingRefresher();
        CtreStatusBatch batch = new CtreStatusBatch(refresher);
        CtreStatusBatch.Member first = batch.join(signals(2));
        batch.refresh(7L);
        batch.join(signals(3));
        batch.refresh(7L);

        assertEquals(List.of(2, 5), refresher.covered);
        // Leaving drops the closed handle's signals; the pass that already ran still covers the rest.
        batch.leave(first);
        batch.refresh(7L);
        assertEquals(List.of(2, 5), refresher.covered);
        batch.refresh(8L);
        assertEquals(List.of(2, 5, 3), refresher.covered);
    }
}
