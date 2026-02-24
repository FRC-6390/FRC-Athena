package ca.frc6390.athena.core;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicLong;

final class NtPublisherPool {
    private final NetworkTableInstance nt;
    private final ConcurrentHashMap<String, BooleanPublisher> booleanPublishers = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, DoublePublisher> doublePublishers = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, StringPublisher> stringPublishers = new ConcurrentHashMap<>();
    private final AtomicLong droppedPublisherCreates = new AtomicLong(0L);

    NtPublisherPool(NetworkTableInstance nt) {
        this.nt = Objects.requireNonNull(nt, "nt");
    }

    BooleanPublisher ensureBooleanPublisher(String fullPath, int maxPublisherCount) {
        BooleanPublisher existing = booleanPublishers.get(fullPath);
        if (existing != null) {
            return existing;
        }
        if (publisherLimitReached(maxPublisherCount)) {
            droppedPublisherCreates.incrementAndGet();
            return null;
        }
        BooleanTopic topic = nt.getBooleanTopic(fullPath);
        BooleanPublisher created = topic.publish();
        BooleanPublisher raced = booleanPublishers.putIfAbsent(fullPath, created);
        if (raced != null) {
            created.close();
            return raced;
        }
        return created;
    }

    DoublePublisher ensureDoublePublisher(String fullPath, int maxPublisherCount) {
        DoublePublisher existing = doublePublishers.get(fullPath);
        if (existing != null) {
            return existing;
        }
        if (publisherLimitReached(maxPublisherCount)) {
            droppedPublisherCreates.incrementAndGet();
            return null;
        }
        DoubleTopic topic = nt.getDoubleTopic(fullPath);
        DoublePublisher created = topic.publish();
        DoublePublisher raced = doublePublishers.putIfAbsent(fullPath, created);
        if (raced != null) {
            created.close();
            return raced;
        }
        return created;
    }

    StringPublisher ensureStringPublisher(String fullPath, int maxPublisherCount) {
        StringPublisher existing = stringPublishers.get(fullPath);
        if (existing != null) {
            return existing;
        }
        if (publisherLimitReached(maxPublisherCount)) {
            droppedPublisherCreates.incrementAndGet();
            return null;
        }
        StringTopic topic = nt.getStringTopic(fullPath);
        StringPublisher created = topic.publish();
        StringPublisher raced = stringPublishers.putIfAbsent(fullPath, created);
        if (raced != null) {
            created.close();
            return raced;
        }
        return created;
    }

    long droppedPublisherCreates() {
        return droppedPublisherCreates.get();
    }

    int booleanPublisherCount() {
        return booleanPublishers.size();
    }

    int doublePublisherCount() {
        return doublePublishers.size();
    }

    int stringPublisherCount() {
        return stringPublishers.size();
    }

    int totalPublisherCount() {
        return booleanPublishers.size() + doublePublishers.size() + stringPublishers.size();
    }

    void closePublishers(String key) {
        BooleanPublisher booleanPublisher = booleanPublishers.remove(key);
        if (booleanPublisher != null) {
            booleanPublisher.close();
        }
        DoublePublisher doublePublisher = doublePublishers.remove(key);
        if (doublePublisher != null) {
            doublePublisher.close();
        }
        StringPublisher stringPublisher = stringPublishers.remove(key);
        if (stringPublisher != null) {
            stringPublisher.close();
        }
    }

    private boolean publisherLimitReached(int maxPublisherCount) {
        return totalPublisherCount() >= maxPublisherCount;
    }
}
