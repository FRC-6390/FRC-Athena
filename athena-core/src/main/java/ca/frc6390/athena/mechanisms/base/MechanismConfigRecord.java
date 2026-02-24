package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.MotionLimits;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.ArrayList;
import java.util.List;

public record MechanismConfigRecord(
        List<MotorControllerConfig> motors,
        EncoderConfig encoder,
        PIDController pidController,
        ProfiledPIDController profiledPIDController,
        boolean useAbsolute,
        boolean useVoltage,
        OutputType outputType,
        boolean useSetpointAsOutput,
        boolean customPIDCycle,
        boolean pidContinous,
        double motorCurrentLimit,
        MotorNeutralMode motorNeutralMode,
        String canbus,
        double encoderGearRatio,
        double encoderConversion,
        double encoderConversionOffset,
        double encoderOffset,
        double encoderDiscontinuityPoint,
        double encoderDiscontinuityRange,
        double tolerance,
        double stateMachineDelay,
        double pidPeriod,
        double hardwareUpdatePeriodSeconds,
        double pidIZone,
        double continousMin,
        double continousMax,
        double minBound,
        double maxBound,
        MotionLimits.AxisLimits motionLimits,
        List<GenericLimitSwitchConfig> limitSwitches) {

    public MechanismConfigRecord {
        if (motors == null) {
            motors = new ArrayList<>();
        }
        if (limitSwitches == null) {
            limitSwitches = new ArrayList<>();
        }
        if (motorNeutralMode == null) {
            motorNeutralMode = MotorNeutralMode.Brake;
        }
        if (canbus == null) {
            canbus = "rio";
        }
        if (outputType == null) {
            outputType = useVoltage ? OutputType.VOLTAGE : OutputType.PERCENT;
        }
    }

    public static MechanismConfigRecord defaults() {
        return new MechanismConfigRecord(
                new ArrayList<>(),
                null,
                null,
                null,
                false,
                false,
                OutputType.PERCENT,
                false,
                false,
                false,
                40.0,
                MotorNeutralMode.Brake,
                "rio",
                1.0,
                1.0,
                0.0,
                0.0,
                Double.NaN,
                Double.NaN,
                0.0,
                0.02,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Double.NaN,
                Double.NaN,
                null,
                new ArrayList<>());
    }

    public Builder toBuilder() {
        return new Builder(this);
    }

    public static final class Builder {
        private List<MotorControllerConfig> motors;
        private EncoderConfig encoder;
        private PIDController pidController;
        private ProfiledPIDController profiledPIDController;
        private boolean useAbsolute;
        private boolean useVoltage;
        private OutputType outputType;
        private boolean useSetpointAsOutput;
        private boolean customPIDCycle;
        private boolean pidContinous;
        private double motorCurrentLimit;
        private MotorNeutralMode motorNeutralMode;
        private String canbus;
        private double encoderGearRatio;
        private double encoderConversion;
        private double encoderConversionOffset;
        private double encoderOffset;
        private double encoderDiscontinuityPoint;
        private double encoderDiscontinuityRange;
        private double tolerance;
        private double stateMachineDelay;
        private double pidPeriod;
        private double hardwareUpdatePeriodSeconds;
        private double pidIZone;
        private double continousMin;
        private double continousMax;
        private double minBound;
        private double maxBound;
        private MotionLimits.AxisLimits motionLimits;
        private List<GenericLimitSwitchConfig> limitSwitches;

        public Builder(MechanismConfigRecord base) {
            this.motors = base.motors();
            this.encoder = base.encoder();
            this.pidController = base.pidController();
            this.profiledPIDController = base.profiledPIDController();
            this.useAbsolute = base.useAbsolute();
            this.useVoltage = base.useVoltage();
            this.outputType = base.outputType();
            this.useSetpointAsOutput = base.useSetpointAsOutput();
            this.customPIDCycle = base.customPIDCycle();
            this.pidContinous = base.pidContinous();
            this.motorCurrentLimit = base.motorCurrentLimit();
            this.motorNeutralMode = base.motorNeutralMode();
            this.canbus = base.canbus();
            this.encoderGearRatio = base.encoderGearRatio();
            this.encoderConversion = base.encoderConversion();
            this.encoderConversionOffset = base.encoderConversionOffset();
            this.encoderOffset = base.encoderOffset();
            this.encoderDiscontinuityPoint = base.encoderDiscontinuityPoint();
            this.encoderDiscontinuityRange = base.encoderDiscontinuityRange();
            this.tolerance = base.tolerance();
            this.stateMachineDelay = base.stateMachineDelay();
            this.pidPeriod = base.pidPeriod();
            this.hardwareUpdatePeriodSeconds = base.hardwareUpdatePeriodSeconds();
            this.pidIZone = base.pidIZone();
            this.continousMin = base.continousMin();
            this.continousMax = base.continousMax();
            this.minBound = base.minBound();
            this.maxBound = base.maxBound();
            this.motionLimits = base.motionLimits();
            this.limitSwitches = base.limitSwitches();
        }

        public Builder motors(List<MotorControllerConfig> motors) {
            this.motors = motors;
            return this;
        }

        public Builder encoder(EncoderConfig encoder) {
            this.encoder = encoder;
            return this;
        }

        public Builder pidController(PIDController pidController) {
            this.pidController = pidController;
            return this;
        }

        public Builder profiledPIDController(ProfiledPIDController profiledPIDController) {
            this.profiledPIDController = profiledPIDController;
            return this;
        }

        public Builder useAbsolute(boolean useAbsolute) {
            this.useAbsolute = useAbsolute;
            return this;
        }

        public Builder useVoltage(boolean useVoltage) {
            this.useVoltage = useVoltage;
            this.outputType = useVoltage ? OutputType.VOLTAGE : OutputType.PERCENT;
            return this;
        }

        public Builder outputType(OutputType outputType) {
            this.outputType = outputType;
            this.useVoltage = outputType == OutputType.VOLTAGE;
            return this;
        }

        public Builder useSetpointAsOutput(boolean useSetpointAsOutput) {
            this.useSetpointAsOutput = useSetpointAsOutput;
            return this;
        }

        public Builder customPIDCycle(boolean customPIDCycle) {
            this.customPIDCycle = customPIDCycle;
            return this;
        }

        public Builder pidContinous(boolean pidContinous) {
            this.pidContinous = pidContinous;
            return this;
        }

        public Builder motorCurrentLimit(double motorCurrentLimit) {
            this.motorCurrentLimit = motorCurrentLimit;
            return this;
        }

        public Builder motorNeutralMode(MotorNeutralMode motorNeutralMode) {
            this.motorNeutralMode = motorNeutralMode;
            return this;
        }

        public Builder canbus(String canbus) {
            this.canbus = canbus;
            return this;
        }

        public Builder encoderGearRatio(double encoderGearRatio) {
            this.encoderGearRatio = encoderGearRatio;
            return this;
        }

        public Builder encoderConversion(double encoderConversion) {
            this.encoderConversion = encoderConversion;
            return this;
        }

        public Builder encoderConversionOffset(double encoderConversionOffset) {
            this.encoderConversionOffset = encoderConversionOffset;
            return this;
        }

        public Builder encoderOffset(double encoderOffset) {
            this.encoderOffset = encoderOffset;
            return this;
        }

        public Builder encoderDiscontinuityPoint(double encoderDiscontinuityPoint) {
            this.encoderDiscontinuityPoint = encoderDiscontinuityPoint;
            return this;
        }

        public Builder encoderDiscontinuityRange(double encoderDiscontinuityRange) {
            this.encoderDiscontinuityRange = encoderDiscontinuityRange;
            return this;
        }

        public Builder tolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public Builder stateMachineDelay(double stateMachineDelay) {
            this.stateMachineDelay = stateMachineDelay;
            return this;
        }

        public Builder pidPeriod(double pidPeriod) {
            this.pidPeriod = pidPeriod;
            return this;
        }

        public Builder hardwareUpdatePeriodSeconds(double hardwareUpdatePeriodSeconds) {
            this.hardwareUpdatePeriodSeconds = hardwareUpdatePeriodSeconds;
            return this;
        }

        public Builder pidIZone(double pidIZone) {
            this.pidIZone = pidIZone;
            return this;
        }

        public Builder continousMin(double continousMin) {
            this.continousMin = continousMin;
            return this;
        }

        public Builder continousMax(double continousMax) {
            this.continousMax = continousMax;
            return this;
        }

        public Builder minBound(double minBound) {
            this.minBound = minBound;
            return this;
        }

        public Builder maxBound(double maxBound) {
            this.maxBound = maxBound;
            return this;
        }

        public Builder motionLimits(MotionLimits.AxisLimits motionLimits) {
            this.motionLimits = motionLimits;
            return this;
        }

        public Builder limitSwitches(List<GenericLimitSwitchConfig> limitSwitches) {
            this.limitSwitches = limitSwitches;
            return this;
        }

        public MechanismConfigRecord build() {
            return new MechanismConfigRecord(
                    motors,
                    encoder,
                    pidController,
                    profiledPIDController,
                    useAbsolute,
                    useVoltage,
                    outputType,
                    useSetpointAsOutput,
                    customPIDCycle,
                    pidContinous,
                    motorCurrentLimit,
                    motorNeutralMode,
                    canbus,
                    encoderGearRatio,
                    encoderConversion,
                    encoderConversionOffset,
                    encoderOffset,
                    encoderDiscontinuityPoint,
                    encoderDiscontinuityRange,
                    tolerance,
                    stateMachineDelay,
                    pidPeriod,
                    hardwareUpdatePeriodSeconds,
                    pidIZone,
                    continousMin,
                    continousMax,
                    minBound,
                    maxBound,
                    motionLimits,
                    limitSwitches);
        }
    }
}
