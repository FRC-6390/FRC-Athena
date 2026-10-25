package ca.frc6390.athena.wpilib.system;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Predicate;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.stream.Stream;

/**
 * Loads a package's classes on a background thread at boot, so the robot thread does not.
 *
 * <p>Classes load the first time code uses them. Much of Athena's output path is first used on the first
 * enabled cycle (disabled cycles only send neutral outputs), so that cycle read, parsed and defined dozens of
 * classes. On the roboRIO that is a hitch of a few hundred milliseconds at the start of autonomous. Loading
 * them while disabled moves that work off the robot thread. Classes are loaded without being initialized, so no
 * static initializer runs early.</p>
 */
public final class ClassPreloader {
    private final AtomicInteger loaded = new AtomicInteger();
    private final AtomicInteger skipped = new AtomicInteger();
    private volatile boolean finished;
    private volatile long elapsedNanos;

    private ClassPreloader() {
    }

    /**
     * Starts loading every class under {@code packagePrefix} found on the class path, on a daemon thread.
     *
     * @param packagePrefix dotted package prefix, such as {@code "ca.frc6390.athena."}
     * @param include further filter on class names
     * @param loader class loader to define the classes in
     * @return the running preload
     */
    public static ClassPreloader start(String packagePrefix, Predicate<String> include, ClassLoader loader) {
        Objects.requireNonNull(packagePrefix, "packagePrefix");
        Objects.requireNonNull(include, "include");
        Objects.requireNonNull(loader, "loader");
        ClassPreloader preloader = new ClassPreloader();
        Thread thread = new Thread(() -> preloader.run(packagePrefix, include, loader), "athena-class-preload");
        thread.setDaemon(true);
        thread.start();
        return preloader;
    }

    private void run(String packagePrefix, Predicate<String> include, ClassLoader loader) {
        long start = System.nanoTime();
        try {
            for (String name : classNames(System.getProperty("java.class.path", ""), packagePrefix)) {
                if (!include.test(name)) continue;
                try {
                    Class.forName(name, false, loader);
                    loaded.incrementAndGet();
                } catch (ClassNotFoundException | LinkageError missing) {
                    // An optional vendor library that is not deployed: its adapter classes cannot load. Skip them.
                    skipped.incrementAndGet();
                }
            }
        } finally {
            elapsedNanos = System.nanoTime() - start;
            finished = true;
        }
    }

    /** Class names under a dotted package prefix in every jar and directory of a class path. */
    static List<String> classNames(String classPath, String packagePrefix) {
        String prefix = packagePrefix.replace('.', '/');
        List<String> names = new ArrayList<>();
        for (String entry : classPath.split(File.pathSeparator)) {
            if (entry.isBlank()) continue;
            Path path = Path.of(entry);
            if (Files.isDirectory(path)) {
                Path root = path.resolve(prefix);
                if (!Files.isDirectory(root)) continue;
                try (Stream<Path> files = Files.walk(root)) {
                    files.filter(file -> file.toString().endsWith(".class"))
                            .map(file -> className(path.relativize(file).toString().replace(File.separatorChar, '/')))
                            .filter(Objects::nonNull)
                            .forEach(names::add);
                } catch (IOException exception) {
                    throw new UncheckedIOException(exception);
                }
            } else if (entry.endsWith(".jar") && Files.isRegularFile(path)) {
                try (JarFile jar = new JarFile(path.toFile())) {
                    Enumeration<JarEntry> entries = jar.entries();
                    while (entries.hasMoreElements()) {
                        String file = entries.nextElement().getName();
                        if (file.startsWith(prefix) && file.endsWith(".class")) {
                            String name = className(file);
                            if (name != null) names.add(name);
                        }
                    }
                } catch (IOException ignored) {
                    // An unreadable class path entry has nothing to preload.
                }
            }
        }
        return names;
    }

    private static String className(String file) {
        if (file.endsWith("module-info.class") || file.endsWith("package-info.class")) return null;
        return file.substring(0, file.length() - ".class".length()).replace('/', '.');
    }

    /** Classes loaded so far. */
    public int loadedClasses() { return loaded.get(); }

    /** Classes that could not load, for example adapters of a vendor library that is not deployed. */
    public int skippedClasses() { return skipped.get(); }

    /** Whether the preload has finished. */
    public boolean finished() { return finished; }

    /** Time the preload took, in seconds, once finished. */
    public double elapsedSeconds() { return elapsedNanos / 1.0e9; }
}
