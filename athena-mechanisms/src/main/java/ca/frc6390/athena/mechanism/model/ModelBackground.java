package ca.frc6390.athena.mechanism.model;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/** Shared bounded workers: fitting must not wait for a blocked export provider, or vice versa. */
final class ModelBackground {
    static final ThreadPoolExecutor FITS = executor("athena-model-fit", 2);
    static final ThreadPoolExecutor EXPORTS = executor("athena-sysid-export", 32);
    private ModelBackground() { }
    private static ThreadPoolExecutor executor(String name, int capacity) {
        return new ThreadPoolExecutor(1, 1, 30, TimeUnit.SECONDS,
                new ArrayBlockingQueue<>(capacity), runnable -> {
                    Thread worker = new Thread(runnable, name);
                    worker.setDaemon(true);
                    worker.setPriority(Thread.MIN_PRIORITY);
                    return worker;
                });
    }
}
