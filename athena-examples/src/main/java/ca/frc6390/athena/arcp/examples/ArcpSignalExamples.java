package ca.frc6390.athena.arcp.examples;

import ca.frc6390.athena.core.arcp.ARCP;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Full signal publish examples for ARCP.
 */
public final class ArcpSignalExamples {
    private ArcpSignalExamples() {}

    public static void publishPrimitiveSignals(ARCP arcp) {
        arcp.put("Athena/Drive/Enabled", true);
        arcp.put("Athena/Drive/TargetRpm", 4200.0);
        arcp.put("Athena/Drive/ProfileIndex", 2L);
        arcp.put("Athena/Drive/Mode", "closed_loop");
    }

    public static void publishArraySignals(ARCP arcp) {
        arcp.put("Athena/Drive/ModuleConnected", new boolean[] {true, true, true, true});
        arcp.put("Athena/Drive/ModuleCanIds", new long[] {1, 2, 3, 4});
        arcp.put("Athena/Drive/ModuleAnglesDeg", new double[] {12.2, 190.8, -40.0, 91.3});
        arcp.put("Athena/Drive/ModuleNames", new String[] {"FL", "FR", "BL", "BR"});
    }

    public static void publishViaObjectOverload(ARCP arcp) {
        arcp.put("Athena/Object/Bool", Boolean.TRUE);
        arcp.put("Athena/Object/F64", Float.valueOf(3.14159f));
        arcp.put("Athena/Object/I64", Integer.valueOf(9));
        arcp.put("Athena/Object/String", "ready");
        arcp.put("Athena/Object/F64Array", new double[] {0.1, 0.2, 0.3});
    }

    public static void publishWithWidgetAndMetadata(ARCP arcp) {
        arcp.put("Athena/Drive/Kp", 0.12)
                .widget(ARCP.Widgets.numberInput().min(0.0).max(2.0).step(0.001))
                .meta("label", "Drive kP")
                .meta("unit", "")
                .meta("kind", "telemetry")
                .meta("x.team6390.tuningGroup", "drive");

        arcp.put("Athena/Drive/BrakeMode", false)
                .widget(ARCP.Widgets.toggle())
                .meta(Map.of(
                        "label", "Brake Mode",
                        "x.team6390.ui.group", "motor"));

        arcp.put("Athena/Drive/ZeroGyro", false)
                .widget(ARCP.Widgets.button("Zero Gyro"))
                .meta("kind", "command");
    }

    public static void patchMetadata(ARCP arcp) {
        LinkedHashMap<String, Object> patch = new LinkedHashMap<>();
        patch.put("ui.min", 0.0);
        patch.put("ui.max", 1.0);
        patch.put("ui.step", 0.0005);
        patch.put("x.team6390.audit", "retuned");
        arcp.meta("Athena/Drive/Kp").patch(patch);
    }

    public static void highRateTopicHandles(ARCP arcp, double velocityMps, double accelMps2, long loopTick) {
        ARCP.TopicDouble velocityTopic = arcp.topicDouble("Athena/Drive/VelocityMps").bind();
        ARCP.TopicDouble accelTopic = arcp.topicDouble("Athena/Drive/AccelMps2").bind();
        ARCP.TopicI64 tickTopic = arcp.topicI64("Athena/Drive/LoopTick").bind();

        velocityTopic.set(velocityMps);
        accelTopic.set(accelMps2);
        tickTopic.set(loopTick);
    }

    public static Map<String, Integer> resolveSignalIds(ARCP arcp) {
        int explicitF64 = arcp.signalId("Athena/Drive/VelocityMps");
        int implicitAny = arcp.signalId("Athena/Drive/LatencyMs");
        int existingF64 = arcp.existingSignalId("Athena/Drive/VelocityMps");
        int existingAny = arcp.existingSignalId("Athena/Drive/LatencyMs");

        return Map.of(
                "explicitF64", explicitF64,
                "implicitAny", implicitAny,
                "existingF64", existingF64,
                "existingAny", existingAny);
    }
}
