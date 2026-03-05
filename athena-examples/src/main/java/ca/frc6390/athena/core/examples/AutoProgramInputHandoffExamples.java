package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.RobotAuto.AutoBuildCtx;
import ca.frc6390.athena.core.RobotAuto.AutoProgram;
import ca.frc6390.athena.core.RobotAuto.AutoRegisterCtx;
import ca.frc6390.athena.core.RobotAuto.AutoRoutine;
import ca.frc6390.athena.core.RobotAuto.TrajectorySource;

/**
 * Example patterns for passing data between chained auto programs.
 */
public final class AutoProgramInputHandoffExamples {
    private AutoProgramInputHandoffExamples() {}

    public static final String TARGET_MODE_KEY = "handoff.targetMode";
    public static final String FIRE_TRIGGER_KEY = "handoff.fireTrigger";

    /**
     * Caller program: sets ConsumerProgram-scoped inputs, calls it, then clears only that scope.
     */
    public static final class ProducerProgram implements AutoProgram {
        @Override
        public String id() {
            return "ProducerProgram";
        }

        @Override
        public void register(AutoRegisterCtx ctx) {
            ctx.path("ExamplePath", TrajectorySource.CHOREO, "approach");
        }

        @Override
        public AutoRoutine build(AutoBuildCtx ctx) {
            return custom(() -> ctx.sequence(
                    ctx.auto("approach"),
                    ctx.input("ConsumerProgram").string(TARGET_MODE_KEY, "amp"),
                    ctx.input("ConsumerProgram").bool(FIRE_TRIGGER_KEY, true),
                    ctx.auto("ConsumerProgram"),
                    ctx.input("ConsumerProgram").clear()));
        }
    }

    /**
     * Callee program: reads values through ctx.inputValue(...) just like normal inputs.
     */
    public static final class ConsumerProgram implements AutoProgram {
        @Override
        public String id() {
            return "ConsumerProgram";
        }

        @Override
        public void register(AutoRegisterCtx ctx) {
            ctx.input().string(TARGET_MODE_KEY, () -> "speaker");
            ctx.input().bool(FIRE_TRIGGER_KEY, () -> false);
            ctx.path("ExamplePath2", TrajectorySource.CHOREO, "finish");
        }

        @Override
        public AutoRoutine build(AutoBuildCtx ctx) {
            return custom(() -> ctx.sequence(
                    ctx.waitUntil(() -> ctx.input().bool(FIRE_TRIGGER_KEY)),
                    ctx.auto("finish"),
                    ctx.input().resetBool(FIRE_TRIGGER_KEY)));
        }
    }
}
