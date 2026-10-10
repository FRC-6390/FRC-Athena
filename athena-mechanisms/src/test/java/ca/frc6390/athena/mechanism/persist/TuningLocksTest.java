package ca.frc6390.athena.mechanism.persist;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.Test;

class TuningLocksTest {
    @Test
    void undefinedGroupsBehaveAsInMatch() {
        AtomicBoolean inMatch = new AtomicBoolean(false);
        TuningLocks locks = new TuningLocks();
        assertFalse(locks.isLocked(Locks.InMatch.class));
        assertFalse(locks.isLocked(CustomGroup.class));

        locks.define(Locks.InMatch.class, inMatch::get);
        inMatch.set(true);
        assertTrue(locks.isLocked(Locks.InMatch.class));
        assertTrue(locks.isLocked(CustomGroup.class), "undefined groups follow InMatch");
        assertFalse(locks.defined(CustomGroup.class));
    }

    @Test
    void definedGroupsUseTheirOwnCondition() {
        AtomicBoolean drivetrain = new AtomicBoolean(true);
        TuningLocks locks = new TuningLocks().define(CustomGroup.class, drivetrain::get);
        assertTrue(locks.isLocked(CustomGroup.class));
        assertFalse(locks.isLocked(Locks.InMatch.class));
        drivetrain.set(false);
        assertFalse(locks.isLocked(CustomGroup.class));
    }

    @Test
    void builtInGroupsHaveFixedMeaning() {
        TuningLocks locks = new TuningLocks().define(Locks.InMatch.class, () -> true);
        assertFalse(locks.isLocked(Locks.Never.class));
        assertTrue(locks.isLocked(Locks.Always.class));
        assertThrows(IllegalArgumentException.class, () -> locks.define(Locks.Never.class, () -> true));
        assertThrows(IllegalArgumentException.class, () -> locks.define(Locks.Always.class, () -> false));
    }

    @Test
    void lockConditionIsLiveAndNullGroupIsUnlocked() {
        AtomicBoolean state = new AtomicBoolean(false);
        TuningLocks locks = new TuningLocks().define(CustomGroup.class, state::get);
        var condition = locks.lockCondition(CustomGroup.class);
        assertFalse(condition.getAsBoolean());
        state.set(true);
        assertTrue(condition.getAsBoolean());
        assertFalse(locks.isLocked(null));
    }

    private static final class CustomGroup {
    }
}
