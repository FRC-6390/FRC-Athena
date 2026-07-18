package ca.frc6390.athena.hardware.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

class RuntimeBindingsTest {
    @Test
    void declarationsUseIdentityInsteadOfEquality() throws Exception {
        RuntimeBindings<EqualKey, Object> bindings = new RuntimeBindings<>();
        EqualKey first = new EqualKey(1);
        EqualKey equalButDistinct = new EqualKey(1);
        Object firstValue = new Object();
        Object secondValue = new Object();

        AutoCloseable firstBinding = bindings.bind(first, new RuntimeScope("first"), firstValue);
        AutoCloseable secondBinding = bindings.bind(equalButDistinct, new RuntimeScope("second"), secondValue);
        try {
            assertSame(firstValue, bindings.get(first, "first"));
            assertSame(secondValue, bindings.get(equalButDistinct, "second"));
        } finally {
            secondBinding.close();
            firstBinding.close();
        }
    }

    @Test
    void multipleBindingsSelectTheCurrentRuntimeScope() throws Exception {
        RuntimeBindings<Object, String> bindings = new RuntimeBindings<>();
        Object declaration = new Object();
        RuntimeScope first = new RuntimeScope("first");
        RuntimeScope second = new RuntimeScope("second");

        AutoCloseable firstBinding = bindings.bind(declaration, first, "one");
        AutoCloseable secondBinding = bindings.bind(declaration, second, "two");
        try {
            assertEquals("one", first.call(() -> bindings.get(declaration, "device")));
            assertEquals("two", second.call(() -> bindings.find(declaration)));
            assertThrows(IllegalStateException.class, () -> bindings.get(declaration, "device"));
            assertThrows(IllegalStateException.class, () -> bindings.find(declaration));
        } finally {
            secondBinding.close();
            firstBinding.close();
        }
    }

    @Test
    void staleAndRepeatedClosesDoNotRemoveAReplacementBinding() throws Exception {
        RuntimeBindings<Object, Object> bindings = new RuntimeBindings<>();
        Object declaration = new Object();
        RuntimeScope scope = new RuntimeScope("runtime");
        Object sharedValue = new Object();
        AutoCloseable stale = bindings.bind(declaration, scope, sharedValue);
        AutoCloseable current = bindings.bind(declaration, scope, sharedValue);

        stale.close();
        stale.close();
        assertSame(sharedValue, bindings.get(declaration, "device"));

        current.close();
        current.close();
        assertNull(bindings.find(declaration));
    }

    @Test
    @Timeout(10)
    void concurrentReadersObserveConsistentScopeBindingsDuringPublication() throws Exception {
        RuntimeBindings<Object, String> bindings = new RuntimeBindings<>();
        Object declaration = new Object();
        RuntimeScope stableScope = new RuntimeScope("stable");
        RuntimeScope changingScope = new RuntimeScope("changing");
        AtomicBoolean running = new AtomicBoolean(true);
        CountDownLatch readersReady = new CountDownLatch(4);
        ExecutorService executor = Executors.newFixedThreadPool(5);

        AutoCloseable stableBinding = bindings.bind(declaration, stableScope, "stable");
        try {
            List<Future<?>> readers = new ArrayList<>();
            for (int index = 0; index < 4; index++) {
                readers.add(executor.submit(() -> stableScope.run(() -> {
                    readersReady.countDown();
                    while (running.get()) {
                        assertEquals("stable", bindings.get(declaration, "device"));
                        assertEquals("stable", bindings.find(declaration));
                    }
                })));
            }

            readersReady.await();
            Future<?> writer = executor.submit(() -> {
                for (int index = 0; index < 5_000; index++) {
                    AutoCloseable changing = bindings.bind(declaration, changingScope, "changing");
                    try {
                        assertEquals("changing", changingScope.call(() -> bindings.find(declaration)));
                    } catch (Exception exception) {
                        throw new RuntimeException(exception);
                    } finally {
                        try {
                            changing.close();
                        } catch (Exception exception) {
                            throw new RuntimeException(exception);
                        }
                    }
                }
            });

            writer.get(8, TimeUnit.SECONDS);
            running.set(false);
            for (Future<?> reader : readers) {
                reader.get(1, TimeUnit.SECONDS);
            }
        } finally {
            running.set(false);
            stableBinding.close();
            executor.shutdownNow();
            executor.awaitTermination(Duration.ofSeconds(1).toMillis(), TimeUnit.MILLISECONDS);
        }
    }

    private record EqualKey(int value) {}
}
