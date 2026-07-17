package ca.frc6390.athena.wpilib.system;

import java.lang.management.BufferPoolMXBean;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryUsage;
import java.util.List;

record JvmMetrics(
        long heapUsed,
        long heapCommitted,
        long heapMaximum,
        long nonHeapUsed,
        long directBuffers,
        long allocatedBytes,
        long garbageCollections,
        long garbageCollectionMillis,
        double processCpuLoad,
        double systemCpuLoad,
        int liveThreads) {

    private static final java.lang.management.MemoryMXBean MEMORY = ManagementFactory.getMemoryMXBean();
    private static final List<BufferPoolMXBean> BUFFER_POOLS =
            ManagementFactory.getPlatformMXBeans(BufferPoolMXBean.class);
    private static final List<GarbageCollectorMXBean> GARBAGE_COLLECTORS =
            ManagementFactory.getGarbageCollectorMXBeans();
    private static final java.lang.management.OperatingSystemMXBean OPERATING_SYSTEM =
            ManagementFactory.getOperatingSystemMXBean();
    private static final java.lang.management.ThreadMXBean THREADS = ManagementFactory.getThreadMXBean();

    static JvmMetrics capture() {
        MemoryUsage heap = MEMORY.getHeapMemoryUsage();
        MemoryUsage nonHeap = MEMORY.getNonHeapMemoryUsage();
        long direct = 0;
        for (BufferPoolMXBean pool : BUFFER_POOLS) {
            if (pool.getName().equalsIgnoreCase("direct")) direct += pool.getMemoryUsed();
        }
        long collections = 0;
        long collectionMillis = 0;
        for (GarbageCollectorMXBean collector : GARBAGE_COLLECTORS) {
            if (collector.getCollectionCount() >= 0) collections += collector.getCollectionCount();
            if (collector.getCollectionTime() >= 0) collectionMillis += collector.getCollectionTime();
        }
        double processCpu = -1.0;
        double systemCpu = -1.0;
        if (OPERATING_SYSTEM instanceof com.sun.management.OperatingSystemMXBean extended) {
            processCpu = extended.getProcessCpuLoad();
            systemCpu = extended.getCpuLoad();
        }
        return new JvmMetrics(
                heap.getUsed(), heap.getCommitted(), heap.getMax(), nonHeap.getUsed(), direct,
                captureAllocatedBytes(), collections, collectionMillis, processCpu, systemCpu,
                THREADS.getThreadCount());
    }

    private static long captureAllocatedBytes() {
        java.lang.management.ThreadMXBean bean = THREADS;
        if (!(bean instanceof com.sun.management.ThreadMXBean extended)
                || !extended.isThreadAllocatedMemorySupported()) return -1;
        try {
            if (!extended.isThreadAllocatedMemoryEnabled()) extended.setThreadAllocatedMemoryEnabled(true);
            long total = 0;
            for (long id : bean.getAllThreadIds()) {
                long bytes = extended.getThreadAllocatedBytes(id);
                if (bytes > 0) total += bytes;
            }
            return total;
        } catch (RuntimeException ignored) {
            return -1;
        }
    }
}
