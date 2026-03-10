package ca.frc6390.athena.jdtls;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.Arrays;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IResourceChangeEvent;
import org.eclipse.core.resources.IResourceChangeListener;
import org.eclipse.core.resources.IResourceDelta;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IConfigurationElement;
import org.eclipse.core.runtime.IExtensionPoint;
import org.eclipse.core.runtime.Platform;
import org.eclipse.jdt.core.compiler.CompilationParticipant;
import org.osgi.framework.BundleActivator;
import org.osgi.framework.BundleContext;

public final class AthenaJdtlsActivator implements BundleActivator {
    private static final String EXTENSION_POINT = "org.eclipse.jdt.core.compilationParticipant";
    private static final Path LOG_PATH = Path.of(System.getProperty("java.io.tmpdir"), "athena-jdtls.log");
    private IResourceChangeListener resourceChangeListener;

    @Override
    public void start(BundleContext context) {
        log("bundleStart symbolicName=" + context.getBundle().getSymbolicName());
        log("registryRegistered=" + isRegistryRegistered());
        registerParticipantDirectly();
        installMarkerCleaner();
        AthenaMarkerCleaner.sweepWorkspace();
    }

    @Override
    public void stop(BundleContext context) {
        if (resourceChangeListener != null) {
            ResourcesPlugin.getWorkspace().removeResourceChangeListener(resourceChangeListener);
            resourceChangeListener = null;
        }
        log("bundleStop symbolicName=" + context.getBundle().getSymbolicName());
    }

    private static boolean isRegistryRegistered() {
        IExtensionPoint point = Platform.getExtensionRegistry().getExtensionPoint(EXTENSION_POINT);
        if (point == null) {
            return false;
        }
        for (IConfigurationElement element : point.getConfigurationElements()) {
            if ("ca.frc6390.athena.jdtls.AthenaCompilationParticipant".equals(element.getAttribute("class"))) {
                return true;
            }
        }
        return false;
    }

    private static void registerParticipantDirectly() {
        try {
            Class<?> managerClass = Class.forName("org.eclipse.jdt.internal.core.JavaModelManager");
            Method getManager = managerClass.getDeclaredMethod("getJavaModelManager");
            Object manager = getManager.invoke(null);
            Field compilationParticipantsField = managerClass.getDeclaredField("compilationParticipants");
            compilationParticipantsField.setAccessible(true);
            Object compilationParticipants = compilationParticipantsField.get(manager);
            if (compilationParticipants == null) {
                log("directRegister skipped: compilationParticipants missing");
                return;
            }

            Method getRegisteredParticipants = compilationParticipants.getClass().getDeclaredMethod("getRegisteredParticipants");
            getRegisteredParticipants.setAccessible(true);
            Object[][] current = (Object[][]) getRegisteredParticipants.invoke(compilationParticipants);
            if (current == null) {
                log("directRegister skipped: no current participant table");
                return;
            }

            boolean alreadyPresent = false;
            for (Object[] bucket : current) {
                for (Object entry : bucket) {
                    if (entry instanceof AthenaCompilationParticipant) {
                        alreadyPresent = true;
                        break;
                    }
                }
                if (alreadyPresent) {
                    break;
                }
            }
            if (alreadyPresent) {
                log("directRegister skipped: participant already present");
                return;
            }

            Object[][] updated = new Object[current.length][];
            CompilationParticipant participant = new AthenaCompilationParticipant();
            for (int i = 0; i < current.length; i++) {
                Object[] bucket = current[i];
                Object[] nextBucket = Arrays.copyOf(bucket, bucket.length + 1);
                nextBucket[bucket.length] = participant;
                updated[i] = nextBucket;
            }

            Field registeredParticipantsField =
                    compilationParticipants.getClass().getDeclaredField("registeredParticipants");
            registeredParticipantsField.setAccessible(true);
            registeredParticipantsField.set(compilationParticipants, updated);
            log("directRegister success buckets=" + updated.length);
        } catch (ReflectiveOperationException exception) {
            log("directRegister failed: " + exception);
        }
    }

    private void installMarkerCleaner() {
        resourceChangeListener = event -> {
            IResourceDelta delta = event.getDelta();
            if (delta == null) {
                return;
            }
            try {
                delta.accept(node -> {
                    IResource resource = node.getResource();
                    if (resource instanceof IFile file
                            && "java".equals(file.getFileExtension())
                            && (((node.getFlags() & IResourceDelta.MARKERS) != 0)
                                    || node.getKind() == IResourceDelta.CHANGED
                                    || node.getKind() == IResourceDelta.ADDED)) {
                        AthenaMarkerCleaner.clearFalseAthenaStateMarkers(file);
                    }
                    return true;
                });
            } catch (CoreException exception) {
                log("markerCleaner delta failure: " + exception);
            }
        };
        ResourcesPlugin.getWorkspace().addResourceChangeListener(
                resourceChangeListener, IResourceChangeEvent.POST_CHANGE | IResourceChangeEvent.POST_BUILD);
        log("markerCleaner installed");
    }

    private static void log(String message) {
        try {
            Files.writeString(
                    LOG_PATH,
                    Instant.now() + " " + message + System.lineSeparator(),
                    StandardOpenOption.CREATE,
                    StandardOpenOption.APPEND);
        } catch (Exception ignored) {
            // no-op
        }
    }
}
